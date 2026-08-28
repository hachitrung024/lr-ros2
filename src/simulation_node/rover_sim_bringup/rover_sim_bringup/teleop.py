import select,sys,termios,tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
HELP='''ROS 2 rover teleop: W/S forward/back, A/D curve, J/L rotate, SPACE stop, +/- speed, Q quit'''
class Teleop(Node):
    def __init__(self):super().__init__('rover_teleop');self.pub=self.create_publisher(Twist,'/cmd_vel',10);self.linear=.7;self.angular=.8
    def send(self,v,w):m=Twist();m.linear.x=v;m.angular.z=w;self.pub.publish(m)
def main():
    rclpy.init();n=Teleop();old=termios.tcgetattr(sys.stdin);v=w=0.;print(HELP)
    try:
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            rclpy.spin_once(n,timeout_sec=0);key=sys.stdin.read(1).lower() if select.select([sys.stdin],[],[],.04)[0] else ''
            if key=='w':v,w=n.linear,0.
            elif key=='s':v,w=-n.linear,0.
            elif key=='a':v,w=n.linear*.6,n.angular
            elif key=='d':v,w=n.linear*.6,-n.angular
            elif key=='j':v,w=0.,n.angular
            elif key=='l':v,w=0.,-n.angular
            elif key==' ':v=w=0.
            elif key in '+=':n.linear=min(1.5,n.linear+.1);print('linear',n.linear)
            elif key=='-':n.linear=max(.1,n.linear-.1);print('linear',n.linear)
            elif key in ('q','\x03'):break
            n.send(v,w)
    finally:n.send(0.,0.);termios.tcsetattr(sys.stdin,termios.TCSADRAIN,old);n.destroy_node();rclpy.shutdown() if rclpy.ok() else None
if __name__=='__main__':main()
