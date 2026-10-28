"""Translate replay UDP clock into the sole ROS /clock publisher on Jetson."""

import socket
import sys

import rclpy
from rclpy.clock import Clock as RosClock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock

from .replay_clock import ReplayClock


class ReplayClockBridge(Node):
    """Publish only received timestamps; never infer or advance replay time."""

    def __init__(self, **kwargs):
        super().__init__('replay_clock_bridge', **kwargs)
        self.socket = None
        if self.get_parameter('use_sim_time').value:
            raise ValueError('replay_clock_bridge must use_sim_time:=false')
        address = self.declare_parameter('bind_address', Parameter.Type.STRING).value.strip()
        port = self.declare_parameter('bind_port', Parameter.Type.INTEGER).value
        if not address or not 1 <= port <= 65535:
            raise ValueError('Explicit clock bind_address and bind_port in [1, 65535] required')
        self.state = ReplayClock()
        self.publisher = None
        self.check_publishers()
        udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            udp.bind((address, port))
            udp.setblocking(False)
            self.publisher = self.create_publisher(Clock, '/clock', 10)
            self.timer = self.create_timer(
                0.01, self.poll, clock=RosClock(clock_type=ClockType.STEADY_TIME),
            )
            self.socket = udp
        except BaseException:
            udp.close()
            raise
        self.get_logger().info(f'Replay clock UDP {address}:{port}; ROS publisher on Jetson')

    def check_publishers(self):
        """Abort on another clock publisher, including one discovered after startup."""
        publishers = self.get_publishers_info_by_topic('/clock')
        foreign = [p for p in publishers if (
            p.node_name != self.get_name() or p.node_namespace != self.get_namespace()
        )]
        if foreign or len(publishers) > (1 if self.publisher is not None else 0):
            names = ', '.join(p.node_namespace.rstrip('/') + '/' + p.node_name
                              for p in publishers)
            raise RuntimeError('Another /clock publisher detected: ' + names)

    def poll(self):
        """Drain a bounded UDP batch; missing packets leave ROS time unchanged."""
        self.check_publishers()
        for _ in range(256):
            try:
                packet, _peer = self.socket.recvfrom(4097)
            except BlockingIOError:
                break
            stamp = self.state.accept(packet)
            if stamp is not None:
                message = Clock()
                message.clock.sec, message.clock.nanosec = divmod(stamp, 1_000_000_000)
                self.publisher.publish(message)

    def destroy_node(self):
        if self.socket is not None:
            self.socket.close()
            self.socket = None
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    result = 0
    try:
        node = ReplayClockBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as error:
        print(f'Replay clock bridge failed: {error}', file=sys.stderr, flush=True)
        result = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result
