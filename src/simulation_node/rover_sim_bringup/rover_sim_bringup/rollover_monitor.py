import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from safety_perception_msgs.msg import ExternalWrenchArray
from diagnostic_msgs.msg import DiagnosticArray,DiagnosticStatus,KeyValue
class Monitor(Node):
    def __init__(self):
        super().__init__('rollover_monitor');self.odom=None;self.loads={};self.pub=self.create_publisher(DiagnosticArray,'/diagnostics',10)
        self.create_subscription(Odometry,'/odom',lambda m:setattr(self,'odom',m),20);self.create_subscription(ExternalWrenchArray,'/external_wrenches',self.wrench,20);self.create_timer(.05,self.tick)
    def wrench(self,m):self.loads={w.source:w.wrench.force.z for w in m.wrenches}
    def tick(self):
        if self.odom is None:return
        q=self.odom.pose.pose.orientation;roll=math.atan2(2*(q.w*q.x+q.y*q.z),1-2*(q.x*q.x+q.y*q.y));pitch=math.asin(max(-1.,min(1.,2*(q.w*q.y-q.z*q.x))))
        left=max(0.,self.loads.get('left_track',0.));right=max(0.,self.loads.get('right_track',0.));total=left+right;ltr=abs(left-right)/total if total>1 else 1.
        risk=max(abs(roll)/math.radians(45),abs(pitch)/math.radians(45),ltr/.85)
        if risk<.7:level,message=DiagnosticStatus.OK,'stable'
        elif risk<1:level,message=DiagnosticStatus.WARN,'near_limit'
        else:level,message=DiagnosticStatus.ERROR,'rollover_risk'
        s=DiagnosticStatus(level=level,name='rover/rollover_stability',hardware_id='rover_v2',message=message)
        vals={'roll_rad':roll,'pitch_rad':pitch,'left_load_n':left,'right_load_n':right,'load_transfer_ratio':ltr,'risk_ratio':risk}
        s.values=[KeyValue(key=k,value=f'{v:.6f}') for k,v in vals.items()];out=DiagnosticArray();out.header=self.odom.header;out.status=[s];self.pub.publish(out)
def main():
    rclpy.init();n=Monitor()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown() if rclpy.ok() else None
