"""Observe startup and shutdown stop messages in an isolated partition, without a server."""
import os
os.environ['GZ_PARTITION']=f'mission_interrupt_check_{os.getpid()}'
from gz.transport13 import Node
from gz.msgs10.twist_pb2 import Twist
import json,subprocess,time,signal
from pathlib import Path
root=Path(__file__).resolve().parents[1]
node=Node();received=[]
def cb(m):received.append((m.linear.x,m.angular.z))
node.subscribe(Twist,'/orchard/cmd_vel',cb)
p=subprocess.Popen([str(root/'run_mission.sh'),'--output',str(root/'outputs/interrupt-validation')])
try:
 deadline=time.monotonic()+8
 while len(received)<3 and time.monotonic()<deadline:time.sleep(.05)
 assert len(received)>=3,'Startup did not publish stop commands'
 before=len(received);p.send_signal(signal.SIGTERM)
 assert p.wait(timeout=8)==130
 time.sleep(.2)
 run=max((root/'outputs/interrupt-validation').iterdir(),key=lambda p:p.stat().st_mtime)
 d=json.loads((run/'mission.json').read_text())
 assert d['status']=='interrupted' and d['stop_command_sent']
 assert len(received)>before and all(v==0 and w==0 for v,w in received)
 result={'interruption_recorded':True,'startup_stop_messages_observed':before,'shutdown_stop_messages_observed':len(received)-before,'isolated_no_server_test':True}
 (root/'mission_interrupt_verification.json').write_text(json.dumps(result,indent=2))
 print(result)
finally:
 if p.poll() is None:p.kill();p.wait()
