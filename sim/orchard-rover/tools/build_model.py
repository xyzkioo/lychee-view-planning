"""Build an original primitive differential-drive camera rover, in SI units."""
import json, math
from pathlib import Path
from xml.etree import ElementTree as X

root=Path(__file__).resolve().parents[1]; c=json.loads((root/'config.json').read_text())
def el(parent,tag,text=None,**attrs):
    x=X.SubElement(parent,tag,attrs)
    if text is not None:x.text=str(text)
    return x
sdf=X.Element('sdf',version='1.9');m=el(sdf,'model',name='orchard_rover');el(m,'self_collide','false')
def link(name,pose,mass,shape,size,color,friction=1):
    l=el(m,'link',name=name);el(l,'pose',pose);i=el(l,'inertial');el(i,'mass',mass)
    ii=el(i,'inertia')
    for k,v in {'ixx':.03*mass,'iyy':.03*mass,'izz':.04*mass,'ixy':0,'ixz':0,'iyz':0}.items():el(ii,k,v)
    for typ in ['visual','collision']:
        part=el(l,typ,name=name+'_'+typ);geo=el(part,'geometry');g=el(geo,shape)
        for k,v in size.items():el(g,k,v)
        if typ=='visual':
            mat=el(part,'material');el(mat,'ambient',color);el(mat,'diffuse',color)
        else:
            ode=el(el(el(part,'surface'),'friction'),'ode');el(ode,'mu',friction);el(ode,'mu2',friction)
    return l
def joint(name,parent,child,typ='fixed'):
    j=el(m,'joint',name=name,type=typ);el(j,'parent',parent);el(j,'child',child);return j
link('base_link','0 0 0.22 0 0 0',8,'box',{'size':'0.55 0.40 0.16'},'0.08 0.32 0.65 1')
r=c['wheel_radius_m'];track=c['wheel_separation_m']
for side,y in [('left',track/2),('right',-track/2)]:
    l=link(side+'_wheel',f'0 {y} {r} {math.pi/2} 0 0',.6,'cylinder',{'radius':r,'length':.065},'0.025 0.025 0.025 1')
    j=joint(side+'_wheel_joint','base_link',side+'_wheel','revolute');axis=el(j,'axis');el(axis,'xyz','0 1 0',expressed_in='__model__');lim=el(axis,'limit');el(lim,'lower',-1e16);el(lim,'upper',1e16)
for name,x in [('front_support',.22),('rear_support',-.22)]:
    link(name,f'{x} 0 0.045 0 0 0',.15,'sphere',{'radius':.045},'0.15 0.15 0.15 1',.001);joint(name+'_joint','base_link',name)
h=c['camera_height_m'];minimum=c['camera_min_height_m'];maximum=c['camera_max_height_m'];assert .4<minimum<=h<=maximum
# A nested model lets the native GUI display only the three camera joints.
base=m; m=el(base,'model',name='camera_rig');el(m,'pose','0.12 0 0.30 0 0 0')
rail_length=maximum-.30
link('mount',f'0 0 {rail_length/2} 0 0 0',.35,'box',{'size':f'0.03 0.035 {rail_length}'},'0.60 0.65 0.70 1')
z=minimum-.30
link('carriage',f'0 0 {z} 0 0 0',.18,'box',{'size':'0.09 0.09 0.06'},'0.95 0.58 0.06 1')
link('pan_head',f'0 0 {z} 0 0 0',.08,'cylinder',{'radius':.055,'length':.045},'0.18 0.18 0.20 1')
cam=link('camera_link',f'0 0 {z} 0 0 0',.12,'box',{'size':'0.075 0.09 0.055'},'0.04 0.04 0.045 1')
def position_joint(name,parent,child,typ,axis,lo,hi,initial,speed):
    j=joint(name,parent,child,typ);ax=el(j,'axis');el(ax,'xyz',axis);lim=el(ax,'limit');el(lim,'lower',lo);el(lim,'upper',hi);el(lim,'effort',100);el(lim,'velocity',speed)
    p=el(m,'plugin',filename='gz-sim-joint-position-controller-system',name='gz::sim::systems::JointPositionController')
    # Lift supports carriage + pan head + camera: .18 + .08 + .12 kg.
    # Effort PID avoids the velocity-mode return overshoot observed on this host.
    for k,v in {'joint_name':name,'initial_position':initial,'p_gain':200 if typ=='prismatic' else 8,'i_gain':0 if typ=='prismatic' else .5,'d_gain':20 if typ=='prismatic' else .4,'cmd_offset':.38*9.81 if typ=='prismatic' else 0,'i_max':10,'i_min':-10,'cmd_max':100 if typ=='prismatic' else 10,'cmd_min':-100 if typ=='prismatic' else -10}.items():el(p,k,v)
position_joint('lift_joint','mount','carriage','prismatic','0 0 1',0,maximum-minimum,h-minimum,.25)
pan=math.radians(c['camera_pan_limit_degrees'])
position_joint('pan_joint','carriage','pan_head','revolute','0 0 1',-pan,pan,0,.8)
# Positive pitch rotation around +Y points the camera down.
position_joint('tilt_joint','pan_head','camera_link','revolute','0 1 0',math.radians(c['camera_tilt_min_degrees']),math.radians(c['camera_tilt_max_degrees']),-math.radians(c['camera_elevation_degrees']),.6)
states=el(m,'plugin',filename='gz-sim-joint-state-publisher-system',name='gz::sim::systems::JointStatePublisher');el(states,'topic','/orchard/camera/joint_state')
sensor=el(cam,'sensor',name='rgb_camera',type='camera');el(sensor,'pose','0.045 0 0 0 0 0');el(sensor,'always_on','true');el(sensor,'update_rate',c['camera_rate_hz']);el(sensor,'topic','/orchard/camera/image')
ca=el(sensor,'camera');el(ca,'horizontal_fov',math.radians(c['horizontal_fov_degrees']));im=el(ca,'image');el(im,'width',c['camera_width']);el(im,'height',c['camera_height']);el(im,'format','R8G8B8');cl=el(ca,'clip');el(cl,'near',.05);el(cl,'far',30)
m=base;joint('camera_mount_joint','base_link','camera_rig::mount')
p=el(m,'plugin',filename='gz-sim-diff-drive-system',name='gz::sim::systems::DiffDrive')
for k,v in {'left_joint':'left_wheel_joint','right_joint':'right_wheel_joint','wheel_separation':track,'wheel_radius':r,'topic':'/orchard/cmd_vel','odom_topic':'/orchard/odom','tf_topic':'/orchard/tf','frame_id':'odom','child_frame_id':'base_link','odom_publish_frequency':30,'max_linear_velocity':.5,'min_linear_velocity':-.5,'max_angular_velocity':1.0,'min_angular_velocity':-1.0,'max_linear_acceleration':.5,'min_linear_acceleration':-.5}.items():el(p,k,v)
X.indent(sdf);out=root/'models/orchard_rover';out.mkdir(parents=True,exist_ok=True);X.ElementTree(sdf).write(out/'model.sdf',encoding='unicode',xml_declaration=True)
print('Built',out/'model.sdf')
