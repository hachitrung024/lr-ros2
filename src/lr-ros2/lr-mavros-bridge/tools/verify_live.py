"""Observe an actual always-on deployment without changing the source."""

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import statistics
import subprocess
import time

from ament_index_python.packages import get_package_prefix
from geometry_msgs.msg import PoseStamped
from grid_map_msgs.msg import GridMap
import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSProfile, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import String


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=25)
    parser.add_argument('--timeout', type=float, default=60)
    parser.add_argument('--output', required=True)
    parser.add_argument('--without-segmentation', action='store_true')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('verify_live_net')
    stamps, ages, counts = defaultdict(set), defaultdict(list), defaultdict(int)
    state = {}
    calibration = {}
    raw_arrivals, public_arrivals = {}, []
    first = {}
    def sensor(name, message):
        counts[name] += 1
        stamp = ns(message.header.stamp)
        stamps[name].add(stamp)
        ages[name].append((time.time_ns() - stamp) / 1e6)
        first.setdefault(name, time.monotonic())
        if name == 'rgb':
            expected = stamp
            if calibration.get('session_id'):
                expected = (stamp - calibration['offset_ns'] - calibration['boot_anchor_ns']
                            + calibration['record_anchor_ns'])
            public_arrivals.append((expected, time.monotonic()))
    for name, kind, topic in (
        ('rgb', CameraInfo, '/zed/zed_node/rgb/color/rect/camera_info'),
        ('cloud', PointCloud2, '/zed/zed_node/point_cloud/cloud_registered'),
        ('pose', PoseStamped, '/lr/mavlink/pose'),
        ('overlay', Image, '/segmentation/overlay'),
    ):
        node.create_subscription(kind, topic, lambda m, n=name: sensor(n, m), qos_profile_sensor_data)
    def terrain(_message):
        if 'state' not in _message.layers:
            return
        values = _message.data[_message.layers.index('state')].data
        if not any(math.isfinite(v) and v != 0 for v in values):
            return  # Ignore reset/empty snapshots when measuring recovery.
        counts['terrain'] += 1
        first.setdefault('terrain', time.monotonic())
    node.create_subscription(GridMap, '/terrain_geometry/grid_map', terrain,
                             qos_profile_sensor_data)
    node.create_subscription(String, '/lr/source/generation',
                             lambda m: state.update(json.loads(m.data)),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(String, '/lr/source/time_state',
                             lambda m: calibration.update(json.loads(m.data)),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(CameraInfo, '/zed/zed_node/_raw/rgb/color/rect/camera_info',
                             lambda m: raw_arrivals.update({ns(m.header.stamp): time.monotonic()}),
                             qos_profile_sensor_data)
    required = ['rgb', 'cloud', 'pose', 'terrain']
    if not args.without_segmentation:
        required.append('overlay')
    began = time.monotonic()
    deadline = began + args.timeout
    while not all(counts[k] for k in required) and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=.05)
    startup = {name: round(value - began, 3) for name, value in first.items()}
    counts.clear()
    stamps.clear()
    ages.clear()
    raw_arrivals.clear()
    public_arrivals.clear()
    started = time.monotonic()
    while time.monotonic() - started < args.duration:
        rclpy.spin_once(node, timeout_sec=.05)
    elapsed = time.monotonic() - started
    tf_probe = subprocess.run([
        get_package_prefix('lr_mavros_bridge') + '/lib/lr_mavros_bridge/verify_tf_owner',
    ], capture_output=True, text=True, timeout=15)
    tf_report = json.loads(tf_probe.stdout.strip().splitlines()[-1])
    hop_ms = []
    for expected, arrival in public_arrivals:
        if raw_arrivals:
            closest = min(raw_arrivals, key=lambda raw: abs(raw - expected))
            if abs(closest - expected) < 5_000_000:
                hop_ms.append((arrival - raw_arrivals[closest]) * 1000)
    checks = {
        'all_outputs': all(counts[k] for k in required),
        'no_clock_publisher': node.count_publishers('/clock') == 0,
        'no_ground_truth': node.count_publishers('/lr/future_path/ground_truth') == 0,
        'no_prediction': node.count_publishers('/predict_output') == 0,
        'one_camera_tf_owner': tf_report['passed'],
        'rgb_cloud_shared_stamps': len(stamps['rgb'] & stamps['cloud']) > 5,
        'pose_frame_shared_stamps': len(stamps['pose'] & stamps['rgb']) > 5,
        'wall_timestamps': bool(ages['rgb']) and all(-500 <= a < 2000 for a in ages['rgb']),
    }
    report = {'passed': all(checks.values()), 'checks': checks, 'source': state,
              'calibration': calibration, 'tf': tf_report,
              'startup_s': startup, 'counts': dict(counts),
              'rates_hz': {k: round(v / elapsed, 3) for k, v in counts.items()},
              'shared_stamps': len(stamps['rgb'] & stamps['cloud']),
              'age_ms_p50': {k: round(statistics.median(v), 3) for k, v in ages.items() if v}}
    if hop_ms:
        report['camera_info_relay_observed_ms'] = {
            'samples': len(hop_ms), 'p50': round(statistics.median(hop_ms), 3),
            'p95': round(sorted(hop_ms)[min(len(hop_ms) - 1, int(.95 * len(hop_ms)))], 3),
        }
    Path(args.output).write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
