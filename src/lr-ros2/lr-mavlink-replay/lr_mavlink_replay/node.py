"""Replay sensor messages over UDP, paced exclusively by the source /clock."""

import socket

from pymavlink.dialects.v20 import common as mavlink
import rclpy
from rclpy.clock import Clock as RosClock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock

from lr_future_path.mavlink import select_session
from .replay import Recording, encode


class UdpWriter:
    def __init__(self, bind_address, bind_port, destination, port):
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.bind((bind_address, bind_port))
        self.socket.setblocking(False)
        self.destination = (destination, port)

    def write(self, packet):
        return self.socket.sendto(packet, self.destination)


class TelemetryReplay(Node):
    def __init__(self):
        super().__init__("mavlink_replay")

        def param(name, value):
            return self.declare_parameter(name, value).value

        self.db_path = param("db_path", "")
        self.db_dir = param("mavlink_dir", "/zed/lr-ros2/mavlink")
        bind_address = param("bind_address", Parameter.Type.STRING).strip()
        bind_port = param("bind_port", Parameter.Type.INTEGER)
        destination = param("destination", Parameter.Type.STRING).strip()
        port = param("port", Parameter.Type.INTEGER)
        if not bind_address or not destination:
            raise ValueError("bind_address and destination must be nonempty")
        if not all(1 <= value <= 65535 for value in (bind_port, port)):
            raise ValueError("bind_port and port must be in [1, 65535]")
        self.writer = UdpWriter(bind_address, bind_port, destination, port)
        self.mav = mavlink.MAVLink(self.writer, srcSystem=1, srcComponent=1)
        self.parser = mavlink.MAVLink(None)
        self.parser.robust_parsing = True
        self.recording = None
        self.sent = 0
        self.create_subscription(Clock, "/clock", self.clock, 100)
        self.create_timer(1.0, self.heartbeat, clock=RosClock(clock_type=ClockType.STEADY_TIME))
        self.heartbeat()

    def heartbeat(self):
        self.mav.heartbeat_send(
            mavlink.MAV_TYPE_GROUND_ROVER,
            mavlink.MAV_AUTOPILOT_GENERIC,
            0,
            0,
            mavlink.MAV_STATE_ACTIVE,
        )
        # Support discovery/version requests; actuator commands are explicitly unsupported.
        while True:
            try:
                packet, _ = self.writer.socket.recvfrom(65535)
                for request in self.parser.parse_buffer(packet) or []:
                    if request.get_type() != "COMMAND_LONG":
                        continue
                    supported = (
                        request.command == mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES
                        or (
                            request.command == mavlink.MAV_CMD_REQUEST_MESSAGE
                            and int(request.param1) == 148
                        )
                    )
                    if supported:
                        self.mav.autopilot_version_send(
                            mavlink.MAV_PROTOCOL_CAPABILITY_MAVLINK2,
                            0,
                            0,
                            0,
                            0,
                            [0] * 8,
                            [0] * 8,
                            [0] * 8,
                            0,
                            0,
                            0,
                        )
                    self.mav.command_ack_send(
                        request.command,
                        (
                            mavlink.MAV_RESULT_ACCEPTED
                            if supported
                            else mavlink.MAV_RESULT_UNSUPPORTED
                        ),
                        target_system=request.get_srcSystem(),
                        target_component=request.get_srcComponent(),
                    )
            except BlockingIOError:
                break

    def clock(self, message):
        stamp = message.clock.sec * 1_000_000_000 + message.clock.nanosec
        if stamp <= 0:
            return
        if self.recording is None:
            summary = select_session(
                stamp, directory=self.db_dir, explicit_path=self.db_path, require_start_match=False
            )
            self.recording = Recording(summary.path)
            self.get_logger().info(f"Replaying {summary.path}; epoch_us={self.recording.epoch_us}")
            self.heartbeat()
        now_us = stamp // 1000
        boot_ms = (now_us - self.recording.epoch_us) // 1000
        if boot_ms < 0:
            return
        # Round both sides to the same millisecond: this pair defines an exact constant offset.
        self.mav.system_time_send(self.recording.epoch_us + boot_ms * 1000, boot_ms)
        for record in self.recording.advance(stamp):
            message = encode(record, self.recording.epoch_us, self.mav)
            if message is not None:
                self.mav.send(message)
                self.sent += 1

    def destroy_node(self):
        self.writer.socket.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TelemetryReplay()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
