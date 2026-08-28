import math,threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as RosImage,CameraInfo
from gz.transport13 import Node as GzNode
from gz.msgs10.image_pb2 import Image
from .common import gz_time,set_stamp

TOPICS = {
    'left': '/zed2i/left/image',
    'right': '/zed2i/right/image',
    'depth': '/zed2i/rgbd/depth_image',
}
FRAMES = {
    'left': 'zed2i_left_optical_frame',
    'right': 'zed2i_right_optical_frame',
    'depth': 'zed2i_depth_optical_frame',
}
class CameraBridge(Node):
    def __init__(self):
        super().__init__('zed2i_bridge');self.gz=GzNode();self.lock=threading.Lock();self.latest={k:None for k in TOPICS};self.last={k:-1. for k in TOPICS}
        self.image_pubs={k:self.create_publisher(RosImage,f'/zed2i/{k}/image_raw',5) for k in TOPICS}
        self.info_pubs={k:self.create_publisher(CameraInfo,f'/zed2i/{k}/camera_info',5) for k in TOPICS}
        for k,t in TOPICS.items():self.gz.subscribe(Image,t,lambda m,key=k:self.update(key,m))
        self.create_timer(.005,self.tick)
    def update(self,k,m):
        c=Image();c.CopyFrom(m)
        with self.lock:self.latest[k]=c
    def tick(self):
        with self.lock:data=dict(self.latest)
        for name,m in data.items():
            if m is None:continue
            ts=gz_time(m.header)
            if ts==self.last[name]:continue
            self.last[name]=ts;out=RosImage();set_stamp(out.header.stamp,ts);out.header.frame_id=FRAMES[name]
            out.height=m.height;out.width=m.width;out.is_bigendian=False;out.step=m.step;out.data=m.data
            out.encoding='32FC1' if m.pixel_format_type==13 else ('rgb8' if m.pixel_format_type==3 else 'mono8')
            self.image_pubs[name].publish(out)
            info=CameraInfo();info.header=out.header;info.height=m.height;info.width=m.width
            fx=m.width/(2*math.tan(1.919862/2));fy=fx;cx=m.width/2;cy=m.height/2
            info.k=[fx,0.,cx,0.,fy,cy,0.,0.,1.];info.p=[fx,0.,cx,0.,0.,fy,cy,0.,0.,0.,1.,0.]
            if name=='right':info.p[3]=-fx*.12
            self.info_pubs[name].publish(info)
def main():
    rclpy.init();n=CameraBridge()
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    finally:n.destroy_node();rclpy.shutdown() if rclpy.ok() else None
