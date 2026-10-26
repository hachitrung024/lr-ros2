"""Compare received MAVROS telemetry with its DB; use verify_tf_owner for DDS ownership."""

import argparse
from collections import Counter
import ctypes
import json
from pathlib import Path
import time

from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, NavSatFix
from tf2_msgs.msg import TFMessage
from lr_future_path.mavlink import mavlink_attitude_to_ros_quaternion
from lr_mavlink_replay.replay import Recording


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--db", required=True)
    parser.add_argument("--duration", type=float, default=30)
    parser.add_argument("--output", default="/tmp/lr-mavros-telemetry.json")
    args = parser.parse_args()
    rows = {"gps": {}, "attitude": {}, "imu": {}}
    for record in Recording(args.db).records:
        stamp = record.stamp_us * 1000
        if record.kind == "attitude":
            stamp = stamp // 1_000_000 * 1_000_000
        rows[record.kind][stamp] = record.values
    lib = ctypes.CDLL(str(Path(get_package_prefix("lr_mavros_bridge")) / "lib/liblr_geoid.so"))
    geoid = lib.lr_geoid_height
    geoid.argtypes, geoid.restype = [ctypes.c_double, ctypes.c_double], ctypes.c_double
    rclpy.init()
    node = rclpy.create_node("verify_mavros_telemetry")
    counts, errors, maxima = Counter(), Counter(), {}
    camera_tf_seen = set()
    clocks, poses, diagnostics = set(), set(), []

    def check(label, message):
        counts[label] += 1
        stamp = ns(message.header.stamp)
        row = rows[label].get(stamp)
        if row is None:
            errors[label + "_timestamp"] += 1
            return
        if label == "gps":
            error = max(abs(message.latitude - row["lat"]), abs(message.longitude - row["lon"]))
            maxima["gps_degrees"] = max(error, maxima.get("gps_degrees", 0))
            height_error = abs(
                message.altitude - geoid(message.latitude, message.longitude) - row["alt"]
            )
            maxima["gps_altitude_m"] = max(height_error, maxima.get("gps_altitude_m", 0))
        if label == "attitude":
            expected = mavlink_attitude_to_ros_quaternion(row["roll"], row["pitch"], row["yaw"])
            q = message.orientation
            received = np.array([q.x, q.y, q.z, q.w])
            error = min(np.linalg.norm(received - expected), np.linalg.norm(received + expected))
            maxima["quaternion_norm_error"] = max(
                float(error), maxima.get("quaternion_norm_error", 0)
            )
        if label == "imu":
            v = message.linear_acceleration
            expected = np.array([row["acc_x"], -row["acc_y"], -row["acc_z"]]) * 9.80665 / 1000
            error = np.linalg.norm(np.array([v.x, v.y, v.z]) - expected)
            maxima["acceleration_error_mps2"] = max(
                float(error), maxima.get("acceleration_error_mps2", 0)
            )

    for label, kind, topic in [
        ("gps", NavSatFix, "/mavros/global_position/raw/fix"),
        ("attitude", Imu, "/mavros/imu/data"),
        ("imu", Imu, "/mavros/imu/data_raw"),
    ]:
        node.create_subscription(
            kind, topic, lambda m, l=label: check(l, m), qos_profile_sensor_data
        )
    node.create_subscription(
        Clock, "/clock", lambda m: clocks.add(ns(m.clock)), qos_profile_sensor_data
    )
    node.create_subscription(
        PoseStamped,
        "/lr/mavlink/pose",
        lambda m: poses.add(ns(m.header.stamp)),
        qos_profile_sensor_data,
    )

    def tf(message):
        for transform in message.transforms:
            if (
                transform.header.frame_id == "map"
                and transform.child_frame_id == "zed_camera_link"
            ):
                camera_tf_seen.add("map->zed_camera_link")

    node.create_subscription(TFMessage, "/tf", tf, qos_profile_sensor_data)

    def diag(message):
        for status in message.status:
            if status.name == "lr/mavros_pose":
                diagnostics.append(
                    {"status": status.message, **{v.key: v.value for v in status.values}}
                )

    node.create_subscription(DiagnosticArray, "/diagnostics", diag, 10)
    start = time.monotonic()
    while time.monotonic() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.02)
    elapsed = time.monotonic() - start
    passed = (
        all(counts[k] > 5 for k in rows) and not errors and bool(camera_tf_seen) and len(poses) > 5
    )
    passed = (
        passed
        and maxima.get("gps_altitude_m", 1) < 0.002
        and maxima.get("quaternion_norm_error", 1) < 1e-5
    )
    passed = (
        passed
        and maxima.get("acceleration_error_mps2", 1) < 1e-4
        and node.count_publishers("/clock") == 1
    )
    report = {
        "passed": passed,
        "duration_s": elapsed,
        "counts": dict(counts),
        "hz": {k: v / elapsed for k, v in counts.items()},
        "timestamp_errors": dict(errors),
        "max_errors": maxima,
        "camera_tf_observed": bool(camera_tf_seen),
        "tf_publishers": [i.node_name for i in node.get_publishers_info_by_topic("/tf")],
        "pose_clock_matches": len(poses & clocks),
        "clock_frames": len(clocks),
        "pose_frames": len(poses),
        "diagnostics": diagnostics,
    }
    Path(args.output).write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
