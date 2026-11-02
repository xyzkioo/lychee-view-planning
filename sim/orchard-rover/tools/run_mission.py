"""Execute preselected camera stations with Gazebo world-pose feedback.

A single-car, known-clear-space demonstration, not an obstacle/path planner.
Height is nominal gimbal height; tilt in the mission file is positive upwards.
"""
import argparse
import copy
from datetime import datetime
import json
import math
from pathlib import Path
import re
import signal
import tempfile
import threading
import time
from zoneinfo import ZoneInfo

from PIL import Image as PILImage
from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double
from gz.msgs10.image_pb2 import Image
from gz.msgs10.model_pb2 import Model
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist

ROOT = Path(__file__).resolve().parents[1]
POSITION_TOL = 0.06
YAW_TOL = math.radians(3)
JOINT_TOL = math.radians(1)


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def stamp(m):
    return m.header.stamp.sec + m.header.stamp.nsec * 1e-9


def yaw(q):
    return math.atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z))


def segment_distance(a, b, center):
    dx, dy = b[0]-a[0], b[1]-a[1]
    t = max(0, min(1, ((center[0]-a[0])*dx + (center[1]-a[1])*dy) / (dx*dx+dy*dy))) if dx*dx+dy*dy else 0
    return math.hypot(a[0]+t*dx-center[0], a[1]+t*dy-center[1])


def load_plan(path, config):
    plan = json.loads(path.read_text())
    if not plan.get('waypoints') or not isinstance(plan['waypoints'], list):
        raise ValueError('任务必须包含非空 waypoints 列表')
    for key in ['world', 'model']:
        if not re.fullmatch(r'[A-Za-z0-9_]+', plan.get(key, '')):
            raise ValueError(f'无效的 {key} 名称')
    center = plan['tree_center_xy_m']
    if len(center) != 2 or not all(math.isfinite(float(v)) for v in center):
        raise ValueError('无效的树中心')
    radius = float(plan['exclusion_radius_m'])
    if not math.isfinite(radius) or radius < 2.5:
        raise ValueError('演示禁入区半径不得小于 2.5 米')
    center = plan['tree_center_xy_m'] = [float(v) for v in center]
    plan['exclusion_radius_m'] = radius
    names = set()
    for w in plan['waypoints']:
        name = w['name']
        if not re.fullmatch(r'[A-Za-z0-9_-]+', name) or name in names:
            raise ValueError('拍摄点名称须唯一且仅含字母、数字、下划线或短横线')
        names.add(name)
        for k in ['x_m', 'y_m', 'yaw_deg', 'height_m', 'pan_deg', 'tilt_deg']:
            w[k] = float(w[k])
            if not math.isfinite(w[k]):
                raise ValueError(f'{name}: {k} 必须为有限数')
        if not config['camera_min_height_m'] <= w['height_m'] <= config['camera_max_height_m']:
            raise ValueError(f'{name}: 高度超出相机范围')
        if abs(w['pan_deg']) > config['camera_pan_limit_degrees']:
            raise ValueError(f'{name}: 水平角度超出范围')
        if not config['camera_tilt_min_degrees'] <= -w['tilt_deg'] <= config['camera_tilt_max_degrees']:
            raise ValueError(f'{name}: 俯仰角度超出范围')
    points = [(w['x_m'], w['y_m']) for w in plan['waypoints']]
    for a, b in zip(points, points[1:] + points[-1:]):
        if segment_distance(a, b, center) < radius + 0.1:
            raise ValueError('目标或点间直线段过于接近树的演示禁入区')
    return plan


class Mission:
    def __init__(self, plan, config, output):
        self.plan, self.config, self.output = plan, config, output
        self.node = Node()
        self.lock = threading.Lock()
        self.latest = {}
        self.trajectory = []
        self.last_clock = None
        self.clock_wall = time.monotonic()
        self.node.subscribe(Pose_V, f"/world/{plan['world']}/dynamic_pose/info", self.on_pose)
        self.node.subscribe(Model, '/orchard/camera/joint_state', self.on_joints)
        self.node.subscribe(Image, '/orchard/camera/image', self.on_image)
        self.drive = self.node.advertise('/orchard/cmd_vel', Twist)
        self.camera = {j: self.node.advertise(f'/model/camera_rig/joint/{j}/0/cmd_pos', Double)
                       for j in ['lift_joint', 'pan_joint', 'tilt_joint']}
        self.result = {'status': 'running', 'localization': 'Gazebo world pose (ideal feedback)',
                       'position_tolerance_m': POSITION_TOL, 'yaw_tolerance_deg': 3,
                       'camera_height_tolerance_m': .01, 'camera_angle_tolerance_deg': 1,
                       'settle_sim_seconds': 1.0, 'max_sensor_time_difference_s': .2,
                       'captures': []}

    def on_pose(self, msg):
        for p in msg.pose:
            if p.name != self.plan['model']:
                continue
            t = stamp(msg)
            pose = {'x_m': p.position.x, 'y_m': p.position.y, 'z_m': p.position.z,
                    'yaw_rad': yaw(p.orientation), 'stamp_s': t,
                    'orientation_xyzw': [p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]}
            with self.lock:
                prev = self.latest.get('pose')
                dt = t-prev['stamp_s'] if prev else 0
                if dt > 0:
                    pose['speed_m_s'] = math.hypot(pose['x_m']-prev['x_m'], pose['y_m']-prev['y_m'])/dt
                    pose['yaw_rate_rad_s'] = wrap(pose['yaw_rad']-prev['yaw_rad'])/dt
                elif prev:
                    pose['speed_m_s'] = prev['speed_m_s']
                    pose['yaw_rate_rad_s'] = prev['yaw_rate_rad_s']
                else:
                    pose['speed_m_s'] = pose['yaw_rate_rad_s'] = float('inf')
                self.latest['pose'] = pose
                self.latest['pose_wall'] = time.monotonic()
                if not self.trajectory or t-self.trajectory[-1]['stamp_s'] >= .1:
                    self.trajectory.append({k: pose[k] for k in ['stamp_s', 'x_m', 'y_m', 'yaw_rad']})

    def on_joints(self, msg):
        with self.lock:
            self.latest['joints'] = {j.name: j.axis1.position for j in msg.joint}
            self.latest['joint_stamp'] = stamp(msg)
            self.latest['joint_wall'] = time.monotonic()

    def on_image(self, msg):
        with self.lock:
            self.latest['image'] = msg
            self.latest['image_wall'] = time.monotonic()

    def send_drive(self, v=0, w=0):
        msg = Twist(); msg.linear.x = v; msg.angular.z = w
        self.drive.publish(msg)

    def send_camera(self, targets):
        for j, v in targets.items():
            msg = Double(); msg.data = v; self.camera[j].publish(msg)

    def snapshot(self, startup=False):
        with self.lock:
            # Messages are immutable once received; retain Image without copying pixel data.
            s = dict(self.latest)
        now = time.monotonic()
        if 'pose' not in s:
            return None if startup else self.fail('尚未收到车辆位姿')
        if now-s['pose_wall'] > 5:
            return None if startup else self.fail('车辆位姿反馈已中断')
        t = s['pose']['stamp_s']
        if not all(math.isfinite(s['pose'][k]) for k in ['x_m', 'y_m', 'yaw_rad', 'stamp_s']):
            self.fail('车辆位姿反馈包含无效数值')
        if self.last_clock is None or t > self.last_clock:
            self.last_clock = t; self.clock_wall = now
        if now-self.clock_wall > 12:
            return None if startup else self.fail('仿真时间停滞，请确认场景正在运行')
        x, y = s['pose']['x_m'], s['pose']['y_m']
        cx, cy = self.plan['tree_center_xy_m']
        if math.hypot(x-cx, y-cy) < self.plan['exclusion_radius_m']:
            self.fail('车辆进入演示禁入区，任务已停止')
        return s

    def fail(self, message):
        self.send_drive()
        raise RuntimeError(message)

    def wait_ready(self):
        deadline = time.monotonic()+60
        while time.monotonic() < deadline:
            self.send_drive()
            s = self.snapshot(startup=True)
            if s and all(k in s for k in ['joints', 'image']):
                if all(time.monotonic()-s[k+'_wall'] < 5 for k in ['joint', 'image']):
                    return s
            time.sleep(.1)
        self.fail('60 秒内未收到有效的位姿、相机关节和图像；请先启动并运行场景')

    def errors(self, s, goal):
        p = s['pose']
        return math.hypot(goal['x_m']-p['x_m'], goal['y_m']-p['y_m']), wrap(math.radians(goal['yaw_deg'])-p['yaw_rad'])

    def navigate(self, goal):
        s = self.snapshot(); p = s['pose']; start = p['stamp_s']
        if segment_distance((p['x_m'], p['y_m']), (goal['x_m'], goal['y_m']), self.plan['tree_center_xy_m']) < self.plan['exclusion_radius_m']+.1:
            self.fail('当前位置到目标的直线段过于接近树，请先手动移到示例起点附近')
        wall_end = time.monotonic()+240
        while time.monotonic() < wall_end:
            s = self.snapshot(); p = s['pose']; d, e_yaw = self.errors(s, goal)
            if p['stamp_s']-start > 65:
                self.fail('车辆到点超过 65 秒仿真时间')
            if d <= .045:
                if abs(e_yaw) <= math.radians(2):
                    self.send_drive(); return
                self.send_drive(0, max(-.6, min(.6, 1.8*e_yaw)))
            else:
                heading = math.atan2(goal['y_m']-p['y_m'], goal['x_m']-p['x_m'])
                e = wrap(heading-p['yaw_rad'])
                v = min(.25, .7*d)*max(0, math.cos(e)) if abs(e) < .35 else 0
                self.send_drive(v, max(-.6, min(.6, 1.8*e)))
            time.sleep(.05)
        self.fail('车辆到点等待过久，任务已停车')

    def capture(self, index, goal):
        targets = {'lift_joint': goal['height_m']-self.config['camera_min_height_m'],
                   'pan_joint': math.radians(goal['pan_deg']), 'tilt_joint': -math.radians(goal['tilt_deg'])}
        start = self.snapshot()['pose']['stamp_s']; settled = None
        deadline = time.monotonic()+150
        while time.monotonic() < deadline:
            self.send_drive(); self.send_camera(targets)
            s = self.snapshot(); p = s['pose']; t = p['stamp_s']
            if t-start > 35:
                self.fail('相机调整或停车稳定超过 35 秒仿真时间')
            if any(k not in s for k in ['joints', 'image']) or any(time.monotonic()-s[k+'_wall'] > 5 for k in ['joint', 'image']):
                self.fail('相机状态或图像反馈中断')
            d, e_yaw = self.errors(s, goal)
            q = s['joints']
            good_joints = all(j in q and abs(q[j]-v) <= (.01 if j=='lift_joint' else JOINT_TOL) for j, v in targets.items())
            good = (d <= POSITION_TOL and abs(e_yaw) <= YAW_TOL and good_joints
                    and p['speed_m_s'] < .02 and abs(p['yaw_rate_rad_s']) < .03
                    and abs(s['joint_stamp']-t) <= .2)
            if not good:
                settled = None
            elif settled is None:
                settled = t
            img = s['image']; it = stamp(img)
            if (settled is not None and t-settled >= 1 and it >= settled+1
                    and abs(it-t) <= .2 and abs(it-s['joint_stamp']) <= .2):
                filename = f"{index:02d}_{goal['name']}.png"
                if img.pixel_format_type != 3:  # gz.msgs.PixelFormatType.RGB_INT8
                    self.fail(f'不支持的图像像素格式 {img.pixel_format_type}')
                frame = PILImage.frombytes('RGB', (img.width, img.height), bytes(img.data), 'raw', 'RGB', img.step)
                frame.save(self.output/filename)
                record = {'file': filename, 'goal': copy.deepcopy(goal), 'base_world_pose': p,
                          'position_error_m': d, 'yaw_error_deg': math.degrees(e_yaw),
                          'camera_joints': q, 'camera_joint_stamp_s': s['joint_stamp'], 'image_stamp_s': it,
                          'image_size_px': [img.width, img.height], 'settle_start_stamp_s': settled,
                          'nominal_camera_height_m': self.config['camera_min_height_m']+q['lift_joint'],
                          'camera_pan_deg': math.degrees(q['pan_joint']), 'camera_tilt_up_deg': -math.degrees(q['tilt_joint'])}
                self.result['captures'].append(record); self.save()
                print(f"已保存 {filename}；到点误差 {d:.3f} 米，偏航误差 {math.degrees(e_yaw):.2f}°", flush=True)
                return
            time.sleep(.05)
        self.fail('等待停车拍照过久，任务已停止')

    def save(self):
        with self.lock:
            route = list(self.trajectory)
        self.result['physical_path_length_m'] = sum(math.hypot(b['x_m']-a['x_m'], b['y_m']-a['y_m']) for a,b in zip(route,route[1:]))
        self.result['execution_sim_seconds'] = route[-1]['stamp_s']-route[0]['stamp_s'] if route else 0
        for filename, data in [('mission.json', self.result), ('trajectory.json', route)]:
            temp = self.output/(filename+'.tmp'); temp.write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False)); temp.replace(self.output/filename)

    def run(self):
        try:
            s = self.wait_ready()
            points = [(s['pose']['x_m'], s['pose']['y_m'])]+[(w['x_m'],w['y_m']) for w in self.plan['waypoints']]
            for a,b in zip(points,points[1:]):
                if segment_distance(a,b,self.plan['tree_center_xy_m']) < self.plan['exclusion_radius_m']+.1:
                    self.fail('示例起点或点间直线路段过于接近树的禁入区')
            self.result['start_sim_stamp_s'] = s['pose']['stamp_s']; self.save()
            for i,w in enumerate(self.plan['waypoints'],1):
                print(f"前往拍摄点 {i}/{len(self.plan['waypoints'])}：{w['name']}", flush=True)
                self.navigate(w); self.capture(i,w)
            self.result['status'] = 'complete'
            print(f'任务完成，照片和记录：{self.output}', flush=True)
        except BaseException as e:
            self.result['status'] = 'interrupted' if isinstance(e,KeyboardInterrupt) else 'failed'
            self.result['error'] = str(e) or type(e).__name__
            raise
        finally:
            for _ in range(6):
                self.send_drive(); time.sleep(.05)
            self.result['stop_command_sent'] = True
            self.save()


def main():
    parser = argparse.ArgumentParser(description='依次到达拍摄点，调整相机，停车保存照片。请先运行 start.sh。')
    parser.add_argument('--mission', type=Path, default=ROOT/'missions/three_points.json')
    parser.add_argument('--output', type=Path, default=ROOT/'outputs/missions', help='每次在此目录内新建独立运行文件夹')
    args = parser.parse_args()
    config = json.loads((ROOT/'config.json').read_text()); plan = load_plan(args.mission, config)
    args.output.mkdir(parents=True, exist_ok=True)
    # Prevent concurrent mission executors on the same Gazebo partition / topic set.
    import fcntl, hashlib, os
    lock_key = hashlib.sha256((os.environ.get('GZ_PARTITION','')+'/orchard/cmd_vel').encode()).hexdigest()[:20]
    with open(Path(tempfile.gettempdir())/f'orchard_mission_{lock_key}.lock','w') as lockfile:
        try:
            fcntl.flock(lockfile, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            parser.error('同一场景已有自动拍摄任务在运行')
        prefix = datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S_')
        output = Path(tempfile.mkdtemp(prefix=prefix, dir=args.output))
        (output/'plan.json').write_text(json.dumps(plan, indent=2, ensure_ascii=False))
        (output/'camera_config.json').write_text(json.dumps(config, indent=2))
        print(f'本次输出：{output}；Ctrl+C 停车并结束任务。任务期间请勿同时手动控制。', flush=True)
        def interrupt(*_):
            raise KeyboardInterrupt()
        signal.signal(signal.SIGTERM, interrupt)
        runner = Mission(plan,config,output)
        try:
            runner.run()
        except KeyboardInterrupt:
            print('任务已中止，已发送停车指令。', flush=True); return 130
        except Exception as e:
            print(f'任务失败，已发送停车指令：{e}', flush=True); return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
