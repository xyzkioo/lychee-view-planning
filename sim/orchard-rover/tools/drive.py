import argparse,select,sys,termios,time,tty
from gz.transport13 import Node
from gz.msgs10.twist_pb2 import Twist
p=argparse.ArgumentParser();p.add_argument('--linear',type=float,default=0);p.add_argument('--angular',type=float,default=0);p.add_argument('--seconds',type=float);a=p.parse_args()
node=Node();pub=node.advertise('/orchard/cmd_vel',Twist)
def send(v,w):
    msg=Twist();msg.linear.x=v;msg.angular.z=w;pub.publish(msg)
time.sleep(.3)
try:
    if a.seconds is not None:
        end=time.monotonic()+a.seconds
        while time.monotonic()<end:send(a.linear,a.angular);time.sleep(.05)
    else:
        if not sys.stdin.isatty():raise RuntimeError('Open control.sh in a terminal')
        print('W forward / S reverse / A left / D right / SPACE stop / Q quit. Hold/repeat a key to continue.');settings=termios.tcgetattr(sys.stdin);tty.setcbreak(sys.stdin);v=w=0;last=0
        try:
            while True:
                if select.select([sys.stdin],[],[],.05)[0]:
                    key=sys.stdin.read(1).lower();last=time.monotonic()
                    if key in ('q','\x03'):break
                    v,w={'w':(.25,0),'s':(-.2,0),'a':(0,.6),'d':(0,-.6),' ':(0,0)}.get(key,(0,0))
                if time.monotonic()-last>.4:v=w=0
                send(v,w)
        finally:termios.tcsetattr(sys.stdin,termios.TCSADRAIN,settings)
except KeyboardInterrupt:
    pass
finally:
    for _ in range(5):send(0,0);time.sleep(.05)
