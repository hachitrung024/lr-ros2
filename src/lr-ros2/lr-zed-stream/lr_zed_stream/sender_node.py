"""ROS endpoints for a single host-side ZED NET source."""

from dataclasses import fields

import rclpy
from rclpy.clock import Clock as RosClock, ClockType
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger
from zed_msgs.msg import SvoStatus
from zed_msgs.srv import SetSvoFrame

from lr_zed_stream.source import SourceOptions, StreamSource


class StreamSender(Node):
    """Run all SDK operations in one mutually exclusive executor thread."""

    def __init__(self, source_factory=StreamSource, **kwargs):
        """Open a source and advertise SVO-only clock and playback controls."""
        super().__init__('zed_node', **kwargs)
        self.source = None
        if self.get_parameter('use_sim_time').value:
            raise ValueError('The source must use_sim_time:=false; it produces /clock')
        values = {
            field.name: self.declare_parameter(field.name, field.default).value
            for field in fields(SourceOptions)
        }
        enable_controls = self.declare_parameter('enable_svo_controls', False).value
        if values['start_paused'] and not enable_controls:
            raise ValueError('start_paused requires experimental enable_svo_controls:=true')
        self.source = source_factory(SourceOptions(**values))
        self._clock_publisher = None
        if self.source.svo_mode:
            self._clock_publisher = self.create_publisher(Clock, '/clock', 10)
            self._status_publisher = self.create_publisher(SvoStatus, '~/status/svo', 10)
            if enable_controls:
                self.get_logger().warning(
                    'Experimental SVO controls: pause/seek NET has not passed SDK 5.4.1 '
                    'acceptance testing; a paused receiver can time out.'
                )
                self._toggle_service = self.create_service(
                    Trigger, '~/toggle_svo_pause', self._toggle
                )
                self._seek_service = self.create_service(
                    SetSvoFrame, '~/set_svo_frame', self._seek
                )
            self._status_timer = self.create_timer(
                0.5, self._publish_status, clock=RosClock(clock_type=ClockType.STEADY_TIME)
            )
        self._capture_timer = self.create_timer(
            self.source.period, self._capture,
            clock=RosClock(clock_type=ClockType.STEADY_TIME),
        )
        self._publish_clock()
        self.get_logger().info(
            f'ZED NET source {values["svo_path"]}: port={values["port"]} '
            f'{values["codec"]} {values["bitrate"]} Kbit/s, '
            f'native_fps={self.source.fps:g}; depth/tracking disabled'
        )

    def _capture(self):
        self.source.tick()
        self._publish_clock()

    def _publish_clock(self):
        if self._clock_publisher is not None and self.source.stamp_ns:
            message = Clock()
            message.clock.sec, message.clock.nanosec = divmod(self.source.stamp_ns, 1_000_000_000)
            self._clock_publisher.publish(message)

    def _publish_status(self):
        message = SvoStatus()
        message.file_name = self.source.options.svo_path
        message.frame_id = self.source.frame_id
        message.total_frames = self.source.total_frames
        message.frame_ts = self.source.stamp_ns
        message.real_time_mode = False
        message.status = (
            SvoStatus.STATUS_END if self.source.ended else
            SvoStatus.STATUS_PAUSED if self.source.paused else SvoStatus.STATUS_PLAYING
        )
        self._status_publisher.publish(message)

    def _toggle(self, _request, response):
        try:
            response.message = self.source.toggle()
            response.success = True
            self._publish_clock()
            self._publish_status()
        except (ValueError, RuntimeError) as error:
            response.message = str(error)
        return response

    def _seek(self, request, response):
        try:
            self.source.seek(request.frame_id)
            self._capture_timer.reset()
            self._publish_clock()
            self._publish_status()
            response.success = True
            response.message = f'SVO frame set to {self.source.frame_id}'
        except (ValueError, RuntimeError) as error:
            response.message = str(error)
        return response

    def destroy_node(self):
        """Close the SDK before destroying ROS publishers and timers."""
        if self.source is not None:
            self.source.close()
            self.source = None
        return super().destroy_node()


def main(args=None):
    """Start the source without a GUI, TF broadcaster or processing pipeline."""
    rclpy.init(args=args)
    node = None
    executor = SingleThreadedExecutor()
    try:
        node = StreamSender()
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
