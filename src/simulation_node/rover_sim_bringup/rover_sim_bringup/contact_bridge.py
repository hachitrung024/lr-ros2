import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped
from safety_perception_msgs.msg import ExternalWrench,ExternalWrenchArray
from gz.transport13 import Node as GzNode
from gz.msgs10.contacts_pb2 import Contacts
from gz.msgs10.odometry_pb2 import Odometry
from .common import gz_time,set_stamp

SOURCES={
 'left_track':'/model/rover_v2/contact/left_track',
 'right_track':'/model/rover_v2/contact/right_track',
 'chassis':'/model/rover_v2/contact/chassis'}
class ContactBridge(Node):
    def __init__(self):
        super().__init__('contact_wrench_bridge');self.lock=threading.Lock();self.latest={k:None for k in SOURCES};self.odom=None;self.last=-1.;self.gz=GzNode()
        self.array_pub=self.create_publisher(ExternalWrenchArray,'/external_wrenches',20)
        self.wrench_pubs={k:self.create_publisher(WrenchStamped,f'/contacts/{k}/wrench',20) for k in SOURCES}
        for k,t in SOURCES.items():self.gz.subscribe(Contacts,t,lambda m,key=k:self.update(key,m))
        self.gz.subscribe(Odometry,'/model/rover_v2/ground_truth',self.update_odom)
        self.create_timer(.01,self.tick)
    def update(self,k,m):
        c=Contacts();c.CopyFrom(m)
        with self.lock:self.latest[k]=c
    def update_odom(self,m):
        c=Odometry();c.CopyFrom(m)
        with self.lock:self.odom=c
    def cg_world(self,odom):
        q=odom.pose.orientation;z=.28
        # Rotate local [0,0,z] by quaternion, then translate.
        return (odom.pose.position.x+2*z*(q.x*q.z+q.w*q.y),
                odom.pose.position.y+2*z*(q.y*q.z-q.w*q.x),
                odom.pose.position.z+z*(1-2*(q.x*q.x+q.y*q.y)))
    def summarize(self,source,msg,stamp,cg):
        force=[0.,0.,0.];torque=[0.,0.,0.];points=[]
        for contact in msg.contact:
            points.extend((p.x,p.y,p.z) for p in contact.position)
            rover_is_1='rover_v2' in contact.collision1.name
            for index,w in enumerate(contact.wrench):
                wr=w.body_1_wrench if rover_is_1 else w.body_2_wrench;f=(wr.force.x,wr.force.y,wr.force.z)
                force[0]+=f[0];force[1]+=f[1];force[2]+=f[2]
                p=contact.position[min(index,len(contact.position)-1)] if contact.position else None
                if p is None:r=(0.,0.,0.)
                else:r=(p.x-cg[0],p.y-cg[1],p.z-cg[2])
                torque[0]+=wr.torque.x+r[1]*f[2]-r[2]*f[1]
                torque[1]+=wr.torque.y+r[2]*f[0]-r[0]*f[2]
                torque[2]+=wr.torque.z+r[0]*f[1]-r[1]*f[0]
        ext=ExternalWrench();ext.header.stamp=stamp;ext.header.frame_id='map';ext.source=source
        ext.wrench.force.x,ext.wrench.force.y,ext.wrench.force.z=force;ext.wrench.torque.x,ext.wrench.torque.y,ext.wrench.torque.z=torque
        if points:
            ext.application_point.x=sum(p[0] for p in points)/len(points);ext.application_point.y=sum(p[1] for p in points)/len(points);ext.application_point.z=sum(p[2] for p in points)/len(points);ext.application_point_valid=True
        ext.confidence=1.;ext.confidence_valid=True
        ws=WrenchStamped();ws.header=ext.header;ws.wrench=ext.wrench;self.wrench_pubs[source].publish(ws)
        return ext
    def tick(self):
        with self.lock:data=dict(self.latest);odom=self.odom
        available=[m for m in data.values() if m is not None]
        if not available:return
        ts=max(gz_time(m.header) for m in available)
        if ts==self.last:return
        self.last=ts;arr=ExternalWrenchArray();set_stamp(arr.header.stamp,ts);arr.header.frame_id='map'
        cg=self.cg_world(odom) if odom is not None else (0.,0.,.28)
        for source,msg in data.items():
            if msg is not None:arr.wrenches.append(self.summarize(source,msg,arr.header.stamp,cg))
        self.array_pub.publish(arr)
def main():
    rclpy.init();n=ContactBridge()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown() if rclpy.ok() else None
