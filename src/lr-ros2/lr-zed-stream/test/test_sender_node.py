"""Exercise the actual ROS service, status and clock endpoints without a GPU."""

from dataclasses import replace
import time

import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rosgraph_msgs.msg import Clock
from std_srvs.srv import Trigger
from zed_msgs.msg import SvoStatus
from zed_msgs.srv import SetSvoFrame

from lr_zed_stream.sender_node import StreamSender
from lr_zed_stream.source import StreamSource


def test_ros_pause_seek_clock_and_cleanup(sdk, svo):
    context = Context()
    # colcon runs package tests concurrently; never inject our test /clock
    # into other packages' replay tests or the operational domain 42.
    rclpy.init(context=context, domain_id=157)
    executor = SingleThreadedExecutor(context=context)
    sender = StreamSender(
        source_factory=lambda options: StreamSource(
            replace(options, svo_path=svo.svo_path, start_paused=True), sdk
        ),
        context=context, namespace='/stream_test',
        parameter_overrides=[Parameter('enable_svo_controls', value=True)],
    )
    observer = rclpy.create_node('stream_observer', context=context)
    stamps, statuses = [], []
    observer.create_subscription(Clock, '/clock', lambda m: stamps.append(m.clock), 10)
    observer.create_subscription(
        SvoStatus, '/stream_test/zed_node/status/svo', statuses.append, 10
    )
    seek = observer.create_client(SetSvoFrame, '/stream_test/zed_node/set_svo_frame')
    toggle = observer.create_client(Trigger, '/stream_test/zed_node/toggle_svo_pause')
    executor.add_node(sender)
    executor.add_node(observer)

    def until(predicate):
        deadline = time.monotonic() + 5
        while not predicate() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.05)
        assert predicate()

    try:
        until(lambda: len(stamps) >= 3 and statuses and seek.service_is_ready())
        assert len({(s.sec, s.nanosec) for s in stamps}) == 1
        assert statuses[-1].status == SvoStatus.STATUS_PAUSED
        future = seek.call_async(SetSvoFrame.Request(frame_id=2))
        until(future.done)
        assert future.result().success
        until(lambda: statuses[-1].frame_id == 2)
        assert statuses[-1].status == SvoStatus.STATUS_PAUSED
        assert sender.source.paused
        future = seek.call_async(SetSvoFrame.Request(frame_id=-1))
        until(future.done)
        assert not future.result().success
        future = toggle.call_async(Trigger.Request())
        until(future.done)
        assert future.result().success
        until(lambda: statuses[-1].status == SvoStatus.STATUS_END)
    finally:
        camera = sender.source.camera
        executor.shutdown()
        observer.destroy_node()
        sender.destroy_node()
        context.shutdown()
    assert camera.closed


def test_default_svo_source_exposes_status_without_unverified_controls(sdk, svo):
    """The supported sequential mode must not advertise pause/seek services."""
    context = Context()
    rclpy.init(context=context, domain_id=157)
    sender = StreamSender(
        source_factory=lambda options: StreamSource(replace(options, svo_path=svo.svo_path), sdk),
        context=context, namespace='/sequential_test',
    )
    try:
        names = [name for name, _types in sender.get_service_names_and_types()]
        assert '/sequential_test/zed_node/set_svo_frame' not in names
        assert '/sequential_test/zed_node/toggle_svo_pause' not in names
        assert sender._clock_publisher is not None
    finally:
        sender.destroy_node()
        context.shutdown()
