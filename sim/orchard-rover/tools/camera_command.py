"""User coordinates: height in m; pan left positive; tilt up positive, degrees."""
import argparse,json,math,time
from pathlib import Path
from gz.transport13 import Node
from gz.msgs10.double_pb2 import Double
c=json.loads((Path(__file__).resolve().parents[1]/'config.json').read_text())
p=argparse.ArgumentParser();p.add_argument('--height',type=float);p.add_argument('--pan',type=float);p.add_argument('--tilt',type=float);p.add_argument('--home',action='store_true');a=p.parse_args()
if a.home:a.height=c['camera_height_m'];a.pan=0.;a.tilt=c['camera_elevation_degrees']
targets={}
if a.height is not None:
    if not c['camera_min_height_m']<=a.height<=c['camera_max_height_m']:p.error('height outside configured range')
    targets['lift_joint']=a.height-c['camera_min_height_m']
if a.pan is not None:
    if not math.isfinite(a.pan) or abs(a.pan)>c['camera_pan_limit_degrees']:p.error('pan outside configured range')
    targets['pan_joint']=math.radians(a.pan)
if a.tilt is not None:
    value=-a.tilt
    if not c['camera_tilt_min_degrees']<=value<=c['camera_tilt_max_degrees']:p.error('tilt outside configured range')
    targets['tilt_joint']=math.radians(value)
if not targets:p.error('provide --height, --pan, --tilt or --home')
n=Node();pubs={j:n.advertise(f'/model/camera_rig/joint/{j}/0/cmd_pos',Double) for j in targets}
time.sleep(.3)
for _ in range(12):
    for j,v in targets.items():m=Double();m.data=v;pubs[j].publish(m)
    time.sleep(.05)
print('Camera target sent. The simulation must be running to reach the target.')
