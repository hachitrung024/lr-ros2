import threading
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry as RosOdom
from sensor_msgs.msg import Imu as RosImu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster,StaticTransformBroadcaster
from safety_perception_msgs.msg import RoverState
from rosgraph_msgs.msg import Clock as RosClock
from gz.transport13 import Node as GzNode
from gz.msgs10.odometry_pb2 import Odometry
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.clock_pb2 import Clock
from .common import gz_time,set_stamp

class StateBridge(Node):
    def __init__(self):
        super().__init__('rover_state_bridge');self.lock=threading.Lock();self.latest={'odom':None,'imu':None};self.last=None;self.prev=None
        self.odom_pub=self.create_publisher(RosOdom,'/odom',20);self.imu_pub=self.create_publisher(RosImu,'/imu/data',20);self.state_pub=self.create_publisher(RoverState,'/rover/state',20)
        self.clock_pub=self.create_publisher(RosClock,'/clock',10)
        self.tf=TransformBroadcaster(self);self.static_tf=StaticTransformBroadcaster(self);self.gz=GzNode()
        self.gz.subscribe(Odometry,'/model/rover_v2/ground_truth',lambda m:self.update('odom',m));self.gz.subscribe(IMU,'/model/rover_v2/imu',lambda m:self.update('imu',m))
        self.gz.subscribe(Clock,'/clock',self.publish_clock)
        self.publish_static();self.create_timer(.005,self.tick)
    def update(self,k,m):
        c=type(m)();c.CopyFrom(m)
        with self.lock:self.latest[k]=c
    def publish_static(self):
        map_odom=TransformStamped();map_odom.header.frame_id='map';map_odom.child_frame_id='odom';map_odom.transform.rotation.w=1.
        transforms=[map_odom]
        camera_specs=(('left', .06),('right', -.06),('depth', .06))
        for name, lateral_y in camera_specs:
            camera=TransformStamped();camera.header.frame_id='base_link';camera.child_frame_id=f'zed2i_{name}_frame'
            camera.transform.translation.x=.74
            camera.transform.translation.y=lateral_y
            camera.transform.translation.z=.20
            camera.transform.rotation.y=.069756
            camera.transform.rotation.w=.997564
            optical=TransformStamped();optical.header.frame_id=f'zed2i_{name}_frame';optical.child_frame_id=f'zed2i_{name}_optical_frame'
            optical.transform.rotation.x=-.5;optical.transform.rotation.y=.5
            optical.transform.rotation.z=-.5;optical.transform.rotation.w=.5
            transforms.extend((camera,optical))
        self.static_tf.sendTransform(transforms)
    def publish_clock(self,m):
        out=RosClock();out.clock.sec=m.sim.sec;out.clock.nanosec=m.sim.nsec;self.clock_pub.publish(out)
    def tick(self):
        with self.lock:o,i=self.latest['odom'],self.latest['imu']
        if o is None or i is None:return
        ts=gz_time(o.header)
        if ts==self.last:return
        self.last=ts;hframe='odom';child='base_link'
        ro=RosOdom();set_stamp(ro.header.stamp,ts);ro.header.frame_id=hframe;ro.child_frame_id=child
        ro.pose.pose.position.x=o.pose.position.x;ro.pose.pose.position.y=o.pose.position.y;ro.pose.pose.position.z=o.pose.position.z
        ro.pose.pose.orientation.x=o.pose.orientation.x;ro.pose.pose.orientation.y=o.pose.orientation.y;ro.pose.pose.orientation.z=o.pose.orientation.z;ro.pose.pose.orientation.w=o.pose.orientation.w
        ro.twist.twist.linear.x=o.twist.linear.x;ro.twist.twist.linear.y=o.twist.linear.y;ro.twist.twist.linear.z=o.twist.linear.z
        ro.twist.twist.angular.x=i.angular_velocity.x;ro.twist.twist.angular.y=i.angular_velocity.y;ro.twist.twist.angular.z=i.angular_velocity.z;self.odom_pub.publish(ro)
        ri=RosImu();set_stamp(ri.header.stamp,gz_time(i.header));ri.header.frame_id='base_link'
        ri.orientation.x=i.orientation.x;ri.orientation.y=i.orientation.y;ri.orientation.z=i.orientation.z;ri.orientation.w=i.orientation.w
        ri.angular_velocity.x=i.angular_velocity.x;ri.angular_velocity.y=i.angular_velocity.y;ri.angular_velocity.z=i.angular_velocity.z
        ri.linear_acceleration.x=i.linear_acceleration.x;ri.linear_acceleration.y=i.linear_acceleration.y;ri.linear_acceleration.z=i.linear_acceleration.z;self.imu_pub.publish(ri)
        tf=TransformStamped();tf.header=ro.header;tf.child_frame_id=child;tf.transform.translation.x=o.pose.position.x;tf.transform.translation.y=o.pose.position.y;tf.transform.translation.z=o.pose.position.z;tf.transform.rotation=ro.pose.pose.orientation;self.tf.sendTransform(tf)
        state=RoverState();state.header.stamp=ro.header.stamp;state.header.frame_id='map';state.pose=ro.pose.pose;state.pose_valid=True;state.twist=ro.twist.twist;state.twist_valid=True;state.acceleration_valid=False
        cur=(ts,o.twist.linear.x,o.twist.linear.y,o.twist.linear.z,i.angular_velocity.x,i.angular_velocity.y,i.angular_velocity.z)
        if self.prev and ts>self.prev[0]:
            dt=ts-self.prev[0];state.acceleration.linear.x=(cur[1]-self.prev[1])/dt;state.acceleration.linear.y=(cur[2]-self.prev[2])/dt;state.acceleration.linear.z=(cur[3]-self.prev[3])/dt
            state.acceleration.angular.x=(cur[4]-self.prev[4])/dt;state.acceleration.angular.y=(cur[5]-self.prev[5])/dt;state.acceleration.angular.z=(cur[6]-self.prev[6])/dt;state.acceleration_valid=True
        self.prev=cur;self.state_pub.publish(state)
def main():
    rclpy.init();n=StateBridge()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown() if rclpy.ok() else None
