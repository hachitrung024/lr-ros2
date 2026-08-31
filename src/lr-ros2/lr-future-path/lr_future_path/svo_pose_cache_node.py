"""Record ZED SVO map poses into a reusable rosbag2 cache."""

from __future__ import annotations

from datetime import datetime, timezone
import json
import os
from pathlib import Path
import shutil
import sys
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
import rosbag2_py
from zed_msgs.msg import SvoStatus

from .bag_io import POSE_TYPE, stamp_to_nanoseconds
from .cache import (
    CACHE_TOPIC,
    build_cache_spec,
    cache_is_valid,
)


class SvoPoseCacheNode(Node):
    """Write every monotonic ZED map pose until SVO playback reaches END."""

    def __init__(self) -> None:
        """Configure cache output and subscribe to ZED playback topics."""
        super().__init__("svo_pose_cache")
        svo_path = str(self.declare_parameter("svo_path", "").value)
        camera_model = str(
            self.declare_parameter("camera_model", "").value
        )
        requested_cache_path = Path(
            str(self.declare_parameter("cache_path", "").value)
        ).expanduser()
        rebuild = bool(self.declare_parameter("rebuild", False).value)
        pose_topic = str(
            self.declare_parameter(
                "pose_topic", "/zed/zed_node/pose"
            ).value
        )
        status_topic = str(
            self.declare_parameter(
                "status_topic", "/zed/zed_node/status/svo"
            ).value
        )
        self._startup_timeout_s = float(
            self.declare_parameter("startup_timeout_s", 60.0).value
        )
        self._stall_timeout_s = float(
            self.declare_parameter("stall_timeout_s", 30.0).value
        )
        self._end_drain_s = float(
            self.declare_parameter("end_drain_s", 1.0).value
        )
        if not svo_path:
            raise ValueError("svo_path is required")
        if not camera_model:
            raise ValueError("camera_model is required")
        if not str(requested_cache_path):
            raise ValueError("cache_path is required")

        spec = build_cache_spec(
            svo_path, camera_model, requested_cache_path.parent
        )
        if requested_cache_path.resolve() != spec.path.resolve():
            raise ValueError(
                "cache_path does not match the current SVO fingerprint: "
                f"expected {spec.path}, got {requested_cache_path}"
            )
        self._spec = spec
        self._cache_path = spec.path
        self._rebuild = rebuild
        self._writer = None
        self._temporary_path = (
            self._cache_path.parent
            / f".{self._cache_path.name}.tmp-{os.getpid()}"
        )
        self._pose_count = 0
        self._first_stamp_ns = None
        self._last_stamp_ns = None
        self._total_frames = 0
        self._last_progress_bucket = -1
        self._started_at = time.monotonic()
        self._last_activity = self._started_at
        self._received_status = False
        self._ending = False
        self._done = False
        self.exit_code = 0
        self._shutdown_timer = None
        self._end_timer = None

        if cache_is_valid(self._cache_path, self._spec.identity) and not rebuild:
            self._done = True
            self.get_logger().info(
                f"Future-path cache hit: {self._cache_path}"
            )
            self._schedule_shutdown()
            return

        self._prepare_writer()
        self._pose_subscription = self.create_subscription(
            PoseStamped,
            pose_topic,
            self._pose_callback,
            qos_profile_sensor_data,
        )
        self._status_subscription = self.create_subscription(
            SvoStatus,
            status_topic,
            self._status_callback,
            10,
        )
        self._watchdog_timer = self.create_timer(1.0, self._watchdog)
        self.get_logger().info(
            "Recording SVO poses: source=%s status=%s cache=%s"
            % (pose_topic, status_topic, self._cache_path)
        )

    def _prepare_writer(self) -> None:
        self._cache_path.parent.mkdir(parents=True, exist_ok=True)
        if self._temporary_path.exists():
            shutil.rmtree(self._temporary_path)
        self._writer = rosbag2_py.SequentialWriter()
        self._writer.open(
            rosbag2_py.StorageOptions(
                uri=str(self._temporary_path), storage_id="sqlite3"
            ),
            rosbag2_py.ConverterOptions("", ""),
        )
        self._writer.create_topic(
            rosbag2_py.TopicMetadata(
                name=CACHE_TOPIC,
                type=POSE_TYPE,
                serialization_format="cdr",
            )
        )

    def _pose_callback(self, message: PoseStamped) -> None:
        if self._done or self._writer is None:
            return
        timestamp = stamp_to_nanoseconds(message.header.stamp)
        if timestamp <= 0:
            self.get_logger().warning("Dropping pose with a zero timestamp")
            return
        if self._last_stamp_ns is not None and timestamp <= self._last_stamp_ns:
            self.get_logger().warning(
                "Dropping non-monotonic pose timestamp %d" % timestamp
            )
            return
        try:
            self._writer.write(
                CACHE_TOPIC, serialize_message(message), timestamp
            )
        except Exception as exception:
            self._fail(f"failed to write pose to rosbag: {exception}")
            return
        self._pose_count += 1
        if self._first_stamp_ns is None:
            self._first_stamp_ns = timestamp
        self._last_stamp_ns = timestamp
        self._last_activity = time.monotonic()

    def _status_callback(self, message: SvoStatus) -> None:
        if self._done:
            return
        self._received_status = True
        self._last_activity = time.monotonic()
        self._total_frames = max(self._total_frames, int(message.total_frames))
        if message.total_frames:
            percent = int(
                100 * min(message.frame_id + 1, message.total_frames)
                / message.total_frames
            )
            bucket = percent // 10
            if bucket > self._last_progress_bucket:
                self._last_progress_bucket = bucket
                self.get_logger().info(
                    "SVO cache progress: %d%% (%d/%d frames, %d poses)"
                    % (
                        percent,
                        message.frame_id + 1,
                        message.total_frames,
                        self._pose_count,
                    )
                )
        if message.status == SvoStatus.STATUS_END and not self._ending:
            self._ending = True
            self.get_logger().info(
                "SVO reached END; draining final pose callbacks"
            )
            self._end_timer = self.create_timer(
                self._end_drain_s, self._finish_successfully
            )

    def _watchdog(self) -> None:
        if self._done or self._ending:
            return
        now = time.monotonic()
        if (
            not self._received_status
            and now - self._started_at > self._startup_timeout_s
        ):
            self._fail("timed out waiting for ZED SVO status")
        elif (
            self._received_status
            and now - self._last_activity > self._stall_timeout_s
        ):
            self._fail("SVO playback stopped making progress")

    def _close_writer(self) -> None:
        if self._writer is not None:
            self._writer.close()
            self._writer = None

    def _finish_successfully(self) -> None:
        if self._done:
            return
        if self._pose_count < 2:
            self._fail(
                f"SVO ended with only {self._pose_count} recorded poses"
            )
            return
        try:
            self._close_writer()
            manifest = {
                "complete": True,
                "identity": self._spec.identity,
                "pose_topic": CACHE_TOPIC,
                "pose_count": self._pose_count,
                "first_stamp_ns": self._first_stamp_ns,
                "last_stamp_ns": self._last_stamp_ns,
                "total_svo_frames": self._total_frames,
                "created_utc": datetime.now(timezone.utc).isoformat(),
            }
            (
                self._temporary_path / "manifest.json"
            ).write_text(
                json.dumps(manifest, indent=2, sort_keys=True) + "\n",
                encoding="utf-8",
            )
            self._install_completed_cache()
            if not cache_is_valid(self._cache_path, self._spec.identity):
                raise RuntimeError("completed cache failed validation")
        except Exception as exception:
            self._fail(f"failed to finalize cache: {exception}")
            return
        self._done = True
        self.get_logger().info(
            "Future-path cache ready: %s (%d poses)"
            % (self._cache_path, self._pose_count)
        )
        self._schedule_shutdown()

    def _install_completed_cache(self) -> None:
        backup = (
            self._cache_path.parent
            / f".{self._cache_path.name}.old-{os.getpid()}"
        )
        if backup.exists():
            shutil.rmtree(backup)
        moved_old = False
        if self._cache_path.exists():
            self._cache_path.rename(backup)
            moved_old = True
        try:
            self._temporary_path.rename(self._cache_path)
        except Exception:
            if moved_old and not self._cache_path.exists():
                backup.rename(self._cache_path)
            raise
        if moved_old:
            shutil.rmtree(backup)

    def _fail(self, reason: str) -> None:
        if self._done:
            return
        self._done = True
        self.exit_code = 1
        if rclpy.ok():
            self.get_logger().fatal(reason)
        else:
            print(f"FATAL: {reason}", file=sys.stderr)
        try:
            self._close_writer()
        except Exception as exception:
            message = f"Failed to close cache writer: {exception}"
            if rclpy.ok():
                self.get_logger().error(message)
            else:
                print(f"ERROR: {message}", file=sys.stderr)
        if self._temporary_path.exists():
            shutil.rmtree(self._temporary_path, ignore_errors=True)
        self._schedule_shutdown()

    def _schedule_shutdown(self) -> None:
        if self._shutdown_timer is not None or not rclpy.ok():
            return

        def shutdown() -> None:
            if rclpy.ok():
                rclpy.shutdown()

        self._shutdown_timer = self.create_timer(0.05, shutdown)


def main(args=None) -> None:
    """Run the SVO pose cache recorder."""
    rclpy.init(args=args)
    node = None
    exit_code = 1
    try:
        node = SvoPoseCacheNode()
        rclpy.spin(node)
        exit_code = node.exit_code
    except (KeyboardInterrupt, ExternalShutdownException):
        if node is not None and not node._done:
            node._fail("SVO pose cache recording was interrupted")
        exit_code = node.exit_code if node is not None else 1
    except Exception as exception:
        if node is not None and rclpy.ok():
            node.get_logger().fatal(str(exception))
        else:
            print(f"Failed to start svo_pose_cache_node: {exception}")
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        sys.exit(exit_code)


if __name__ == "__main__":
    main()
