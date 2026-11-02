"""Integration checks: physical translation, rotation, stopped RGB capture."""
import json,math,os,signal,subprocess,threading,time
from pathlib import Path
from PIL import Image as PILImage
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.pose_v_pb2 import Pose_V

root=Path(__file__).resolve().parents[1];(root/'outputs').mkdir(exist_ok=True);(root/'logs').mkdir(exist_ok=True)
os.environ['GZ_PARTITION']='orchard_rover_validation';os.environ['QT_QPA_PLATFORM']='offscreen'
node=Node();lock=threading.Lock();latest={};first_frame=threading.Event();odom_ready=threading.Event()
def rgb(topic):
    def callback(msg):
        with lock:latest[topic]=msg
        if topic=='rover':first_frame.set()
    return callback
def odom(msg):
    with lock:latest['odom']=msg
    odom_ready.set()
def poses(msg):
    for p in msg.pose:
        if p.name=='orchard_rover':
            with lock:latest['physical_pose']=p
node.subscribe(Pose_V,'/world/orchard/dynamic_pose/info',poses)
node.subscribe(Image,'/orchard/camera/image',rgb('rover'));node.subscribe(Image,'/orchard/overview',rgb('overview'));node.subscribe(Odometry,'/orchard/odom',odom)
pub=node.advertise('/orchard/cmd_vel',Twist)
def command(v,w,duration):
    with lock:m=latest.get('odom')
    stamp=lambda m:m.header.stamp.sec+m.header.stamp.nsec*1e-9
    start=stamp(m) if m else None;deadline=time.monotonic()+45
    while time.monotonic()<deadline:
        msg=Twist();msg.linear.x=v;msg.angular.z=w;pub.publish(msg);time.sleep(.05)
        with lock:current=latest.get('odom')
        if current and start is not None and stamp(current)-start>=duration:return
        if start is None and current:start=stamp(current)
    raise RuntimeError('Simulation clock stalled')
def state():
    with lock:m=latest['odom']
    q=m.pose.orientation
    result={'x':m.pose.position.x,'y':m.pose.position.y,'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))}
    with lock:p=latest.get('physical_pose')
    if p:
        q=p.orientation;result['world']={'x':p.position.x,'y':p.position.y,'z':p.position.z,'yaw':math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))}
    return result
def capture(tag):
    for key in ['rover','overview']:
        with lock:m=latest.get(key)
        if m:PILImage.frombytes('RGB',(m.width,m.height),bytes(m.data),'raw','RGB',m.step).save(root/'outputs'/f'{tag}_{key}.png')
with (root/'logs/validation.log').open('w') as log:
    proc=subprocess.Popen([str(root/'start.sh'),'-s','--headless-rendering','-v','2'],stdout=log,stderr=log,start_new_session=True)
    try:
        assert first_frame.wait(40),'RGB sensor did not publish'
        assert odom_ready.wait(10),'Odometry did not publish'
        command(0,0,1);initial=state();capture('initial')
        command(.25,0,2);command(0,0,.7);moved=state();capture('forward')
        command(0,.6,1.8);command(0,0,.7);turned=state();capture('turned')
        command(0,0,.7);stopped=state()
        distance=math.hypot(moved['x']-initial['x'],moved['y']-initial['y'])
        angle=math.atan2(math.sin(turned['yaw']-moved['yaw']),math.cos(turned['yaw']-moved['yaw']))
        drift=math.hypot(stopped['x']-turned['x'],stopped['y']-turned['y'])
        assert 'world' in moved and 'world' in initial,'Physical pose topic missing'
        physical_distance=math.hypot(moved['world']['x']-initial['world']['x'],moved['world']['y']-initial['world']['y'])
        physical_angle=math.atan2(math.sin(turned['world']['yaw']-moved['world']['yaw']),math.cos(turned['world']['yaw']-moved['world']['yaw']))
        assert distance>.20,('Insufficient translation',distance)
        assert abs(angle)>.3,('Insufficient yaw rotation',angle)
        assert physical_distance>.20,('Physical robot did not translate',physical_distance)
        assert abs(physical_angle)>.3,('Physical robot did not turn',physical_angle)
        assert drift<.05,('Robot did not stop',drift)
        result={'gazebo_version':'8.15.0','camera_height_m':1.0,'rgb_frames_received':True,'translation_m':distance,'rotation_rad':angle,'physical_translation_m':physical_distance,'physical_rotation_rad':physical_angle,'stop_drift_m':drift,'initial':initial,'moved':moved,'turned':turned,'all_checks_passed':True}
        (root/'verification.json').write_text(json.dumps(result,indent=2));print(json.dumps(result))
    finally:
        for _ in range(4):pub.publish(Twist());time.sleep(.05)
        os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
