"""Run the actual three-stop mission in an isolated Gazebo instance."""
import importlib.util
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from PIL import Image, ImageChops, ImageStat

root = Path(__file__).resolve().parents[1]
(root/'logs').mkdir(exist_ok=True)
# An invalid route must be rejected before connecting to or commanding a car.
spec = importlib.util.spec_from_file_location('mission',root/'tools/run_mission.py')
module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
config = json.loads((root/'config.json').read_text())
bad = json.loads((root/'missions/three_points.json').read_text())
bad['waypoints'][1]['x_m'] = -4; bad['waypoints'][1]['y_m'] = 0
bad_path = root/'logs/rejected_mission.json'; bad_path.write_text(json.dumps(bad))
try:
    module.load_plan(bad_path,config)
except ValueError:
    pass
else:
    raise AssertionError('A route crossing the tree exclusion disk was accepted')
env = dict(os.environ,GZ_PARTITION=f'orchard_mission_validation_{os.getpid()}',QT_QPA_PLATFORM='offscreen')
env.pop('DISPLAY',None)
with (root/'logs/mission_server.log').open('w') as log:
    server = subprocess.Popen([str(root/'start.sh'),'-s','--headless-rendering','-v','2'],env=env,stdout=log,stderr=log,start_new_session=True)
    task = None
    try:
        out = root/'outputs/mission-validation'
        task = subprocess.Popen([sys.executable,str(root/'tools/run_mission.py'),'--output',str(out)],env=env,start_new_session=True)
        assert task.wait(timeout=600)==0,'Mission execution failed'
        run = max(out.iterdir(),key=lambda p:p.stat().st_mtime)
        report = json.loads((run/'mission.json').read_text())
        assert report['status']=='complete' and len(report['captures'])==3,report
        assert report['physical_path_length_m']>2,report['physical_path_length_m']
        frames = []
        for r in report['captures']:
            assert r['position_error_m']<=.06 and abs(r['yaw_error_deg'])<=3,r
            assert r['image_stamp_s']>=r['settle_start_stamp_s']+1,r
            assert abs(r['image_stamp_s']-r['base_world_pose']['stamp_s'])<=.2,r
            assert abs(r['image_stamp_s']-r['camera_joint_stamp_s'])<=.2,r
            frame = Image.open(run/r['file']);frame.load();assert frame.size==(640,480)
            frames.append(frame)
        changes = [sum(ImageStat.Stat(ImageChops.difference(frames[0],f)).mean)/3 for f in frames[1:]]
        assert all(v>2 for v in changes),changes
        result = {'all_checks_passed':True,'tree_crossing_route_rejected':True,'captures':3,
                  'run_directory':str(run),'physical_path_length_m':report['physical_path_length_m'],
                  'image_changes_mean_rgb':changes,'execution_sim_seconds':report['execution_sim_seconds'],
                  'maximum_position_error_m':max(r['position_error_m'] for r in report['captures']),
                  'maximum_yaw_error_deg':max(abs(r['yaw_error_deg']) for r in report['captures'])}
        (root/'mission_verification.json').write_text(json.dumps(result,indent=2))
        print(json.dumps(result,indent=2),flush=True)
    finally:
        for p in [task,server]:
            if p is not None and p.poll() is None:
                os.killpg(p.pid,signal.SIGINT)
                try:p.wait(timeout=10)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
