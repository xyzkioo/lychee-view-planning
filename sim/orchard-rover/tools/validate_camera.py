import json,math,os,signal,subprocess,threading,time
from pathlib import Path
from PIL import Image as PILImage
from gz.transport13 import Node
from gz.msgs10.image_pb2 import Image
from gz.msgs10.model_pb2 import Model
from gz.msgs10.double_pb2 import Double
from gz.msgs10.pose_v_pb2 import Pose_V

root=Path(__file__).resolve().parents[1];(root/'outputs').mkdir(exist_ok=True);(root/'logs').mkdir(exist_ok=True)
os.environ.update(GZ_PARTITION='orchard_moving_camera_validation',QT_QPA_PLATFORM='offscreen')
node=Node();lock=threading.Lock();latest={};ready=threading.Event();images={};records=[]
def stamp(m):return m.header.stamp.sec+m.header.stamp.nsec*1e-9
def joints(m):
    with lock:latest['joints']={j.name:j.axis1.position for j in m.joint};latest['time']=stamp(m)
    ready.set()
def image(m):
    with lock:latest['image']=m
def overview(m):
    with lock:latest['overview']=m
def poses(m):
    with lock:latest['poses']={p.name:{'x':p.position.x,'y':p.position.y,'z':p.position.z} for p in m.pose}
node.subscribe(Model,'/orchard/camera/joint_state',joints);node.subscribe(Image,'/orchard/camera/image',image);node.subscribe(Image,'/orchard/overview',overview);node.subscribe(Pose_V,'/world/orchard/dynamic_pose/info',poses)
pubs={n:node.advertise(f'/model/camera_rig/joint/{n}/0/cmd_pos',Double) for n in ['lift_joint','pan_joint','tilt_joint']}
def move(tag,goal):
    deadline=time.monotonic()+70;settled=None
    while time.monotonic()<deadline:
        for n,v in goal.items():m=Double();m.data=v;pubs[n].publish(m)
        with lock:q=dict(latest.get('joints',{}));t=latest.get('time',0);im=latest.get('image');ov=latest.get('overview');ps=dict(latest.get('poses',{}))
        good=all(n in q and abs(q[n]-v)<.008 for n,v in goal.items())
        if good and settled is None:settled=t
        if not good:settled=None
        if settled is not None and t-settled>1.0 and im and stamp(im)>=settled+.15:
            arr=PILImage.frombytes('RGB',(im.width,im.height),bytes(im.data),'raw','RGB',im.step);arr.save(root/'outputs'/f'camera_{tag}.png');images[tag]=arr
            if ov:PILImage.frombytes('RGB',(ov.width,ov.height),bytes(ov.data),'raw','RGB',ov.step).save(root/'outputs'/f'camera_{tag}_overview.png')
            rec={'stage':tag,'targets':goal,'joint_positions':q,'poses':ps,'image_stamp':stamp(im)};records.append(rec);print(tag,q,flush=True);return
        time.sleep(.08)
    raise RuntimeError(f'{tag}: target not reached; latest joint positions: {q}')
with (root/'logs/camera_validation.log').open('w') as log:
    proc=subprocess.Popen([str(root/'start.sh'),'-s','--headless-rendering','-v','2'],stdout=log,stderr=log,start_new_session=True)
    try:
        assert ready.wait(40),'No camera joint state received'
        initial={'lift_joint':.3,'pan_joint':0.,'tilt_joint':-math.radians(8)}
        move('initial',initial);move('raised',{**initial,'lift_joint':.85});move('panned',{**initial,'lift_joint':.85,'pan_joint':math.radians(30)});move('tilted',{**initial,'lift_joint':.85,'pan_joint':0.,'tilt_joint':math.radians(20)});move('reset',initial)
        # Independent image check catches a camera left fixed on the chassis.
        from PIL import ImageChops,ImageStat
        changes={k:sum(ImageStat.Stat(ImageChops.difference(images['initial'],images[k])).mean)/3 for k in ['raised','panned','tilted']}
        assert all(v>2 for v in changes.values()),changes
        result={'all_checks_passed':True,'minimum_height_m':.7,'nominal_initial_height_m':1.0,'raised_height_target_m':1.55,'image_changes_mean_rgb':changes,'records':records,'actuation':'PID joint effort controllers with speed limits and nominal lift gravity compensation; not calibrated motor dynamics'}
        (root/'camera_verification.json').write_text(json.dumps(result,indent=2));print('Camera motion checks passed',changes,flush=True)
    finally:
        os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=8)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
