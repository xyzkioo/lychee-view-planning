"""Check that the native GUI resolves and locks the camera-only nested model."""
import os,signal,subprocess,time,json,re
from xml.etree import ElementTree as X
from pathlib import Path
root=Path(__file__).resolve().parents[1]
(root/'logs').mkdir(exist_ok=True)
config=root/'gui/camera-test.config'
config.write_text('<window><width>600</width><height>700</height></window><plugin filename="JointPositionController" name="JointPositionController"><model_name>camera_rig</model_name></plugin>')
env=dict(os.environ,GZ_PARTITION='rover_camera_gui_test',QT_QPA_PLATFORM='offscreen',QT_QUICK_BACKEND='software',QT_OPENGL='software',LIBGL_ALWAYS_SOFTWARE='1');env.pop('DISPLAY',None)
with (root/'logs/camera_gui.log').open('w') as gl:
    env['GZ_SIM_RESOURCE_PATH']=str(root/'models')
    # No rendering or external tree asset is needed to resolve the camera joints.
    sdf=X.parse(root/'worlds/orchard.sdf');world=sdf.getroot().find('world')
    for inc in list(world.findall('include')):
        if 'tree' in inc.findtext('uri'):world.remove(inc)
    for model in list(world.findall('model')):
        if model.get('name')=='overview_camera':world.remove(model)
    for plugin in list(world.findall('plugin')):
        if 'sensors' in plugin.get('filename',''):world.remove(plugin)
    test_world=root/'gui/camera-test.sdf';sdf.write(test_world)
    gui=subprocess.Popen(['stdbuf','-oL','-eL','gz','sim','-r',str(test_world),'--gui-config',str(config),'-v','4'],env=env,stdout=gl,stderr=gl,start_new_session=True)
    try:
        deadline=time.monotonic()+30;success=False
        while time.monotonic()<deadline:
            time.sleep(.3)
            text=re.sub(r'\x1b\[[0-9;]*m','',(root/'logs/camera_gui.log').read_text(errors='replace'))
            if 'Controller locked on [camera_rig]' in text:success=True;break
        assert success,'Native GUI did not lock camera_rig; inspect logs/camera_gui.log'
        (root/'gui/camera_verification.json').write_text(json.dumps({'native_gui_plugin_loaded':True,'model_camera_rig_resolved_and_locked':True,'manual_mouse_slider_tested':False},indent=2))
        print('Native camera joint panel loaded and locked camera_rig')
    finally:
        for proc in [gui]:
            if proc.poll() is None:os.killpg(proc.pid,signal.SIGINT)
        for proc in [gui]:
            try:proc.wait(timeout=6)
            except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGKILL);proc.wait()
