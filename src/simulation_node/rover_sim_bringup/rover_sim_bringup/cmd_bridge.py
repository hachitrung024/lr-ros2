import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist as RosTwist
from gz.msgs10.twist_pb2 import Twist as GzTwist
from gz.transport13 import Node as GzNode

class CmdBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_bridge')
        self.declare_parameter('tau_s',.25); self.declare_parameter('max_linear_accel',.8); self.declare_parameter('max_angular_accel',1.5)
        self.target=[0.,0.]; self.output=[0.,0.]; self.gz=GzNode(); self.pub=self.gz.advertise('/model/rover_v2/cmd_vel',GzTwist)
        self.create_subscription(RosTwist,'/cmd_vel',self.command,10); self.create_timer(.02,self.tick)
    def command(self,m):self.target=[float(m.linear.x),float(m.angular.z)]
    def tick(self):
        dt=.02; tau=max(.01,float(self.get_parameter('tau_s').value)); limits=[float(self.get_parameter('max_linear_accel').value),float(self.get_parameter('max_angular_accel').value)]
        for i in range(2):
            desired=(self.target[i]-self.output[i])*dt/tau; limit=limits[i]*dt
            self.output[i]+=max(-limit,min(limit,desired))
        m=GzTwist();m.linear.x=self.output[0];m.angular.z=self.output[1];self.pub.publish(m)
def main():
    rclpy.init();n=CmdBridge()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.pub.publish(GzTwist());n.destroy_node();rclpy.shutdown() if rclpy.ok() else None
