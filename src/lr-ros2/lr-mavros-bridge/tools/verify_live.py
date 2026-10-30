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
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import Path as PathMessage
from safety_perception_msgs.msg import PredictionOutput, Trajectory
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
    parser.add_argument('--with-prediction', action='store_true')
    parser.add_argument('--mode', choices=('camera', 'fusion', 'deployment'), default='fusion')
    parser.add_argument('--source-profile', choices=('native', 'host_replay'), default='native')
    parser.add_argument('--camera-name', default='zed')
    parser.add_argument('--map-frame', default='map')
    parser.add_argument('--expect-degraded', action='store_true')
    parser.add_argument('--expect-no-prediction', action='store_true',
                        help='Fusion stays ready while the mission route is unavailable')
    args = parser.parse_args()
    if args.mode == 'deployment':
        args.with_prediction = True
    if args.expect_degraded and args.mode != 'camera':
        parser.error('--expect-degraded requires --mode camera')
    if args.expect_no_prediction and (args.mode != 'fusion' or not args.with_prediction):
        parser.error('--expect-no-prediction requires --mode fusion --with-prediction')
    rclpy.init()
    node = rclpy.create_node('verify_live_net')
    stamps, ages, counts = defaultdict(set), defaultdict(list), defaultdict(int)
    state, camera_state = {}, {}
    calibration = {}
    diagnostics = {}
    def diagnostic(message):
        for status in message.status:
            level = status.level
            diagnostics[status.name] = {
                'level': int.from_bytes(level, 'little') if isinstance(level, bytes) else int(level),
                'message': status.message,
                'values': {value.key: value.value for value in status.values},
            }
    for topic in ('/diagnostics', '/prediction/diagnostics', '/lr/future_path/diagnostics'):
        node.create_subscription(DiagnosticArray, topic, diagnostic, 10)
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
            if args.source_profile == 'host_replay' and calibration.get('session_id'):
                expected = (stamp - calibration['offset_ns'] - calibration['boot_anchor_ns']
                            + calibration['record_anchor_ns'])
            elif camera_state.get('stamp_is_system'):
                expected -= camera_state.get('stamp_offset_ns', 0)
            public_arrivals.append((expected, time.monotonic()))
    for name, kind, topic in (
        ('rgb', CameraInfo, f'/{args.camera_name}/zed_node/rgb/color/rect/camera_info'),
        ('cloud', PointCloud2, f'/{args.camera_name}/zed_node/point_cloud/cloud_registered'),
        ('pose', PoseStamped, '/lr/mavlink/pose'),
        ('overlay', Image, '/segmentation/overlay'),
    ):
        node.create_subscription(kind, topic, lambda m, n=name: sensor(n, m), qos_profile_sensor_data)
    if args.with_prediction:
        for name, kind, topic in (
            ('future_path', PathMessage, '/lr/future_path/' + (
                'mission' if args.source_profile == 'native' else 'mavlink')),
            ('trajectory', Trajectory, '/trajectory'),
            ('prediction', PredictionOutput, '/predict_output'),
            ('reference_path', PathMessage, '/lr/path_prediction/reference_path'),
        ):
            def path_product(message, key=name):
                if hasattr(message, 'poses') and len(message.poses) < 2:
                    return
                sensor(key, message)
            node.create_subscription(kind, topic, path_product, qos_profile_sensor_data)
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
    node.create_subscription(String, '/lr/source/camera_generation',
                             lambda m: camera_state.update(json.loads(m.data)),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(String, '/lr/source/time_state',
                             lambda m: calibration.update(json.loads(m.data)),
                             QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(CameraInfo, f'/{args.camera_name}/zed_node/_raw/rgb/color/rect/camera_info',
                             lambda m: raw_arrivals.update({ns(m.header.stamp): time.monotonic()}),
                             qos_profile_sensor_data)
    required = ['rgb', 'cloud'] + ([] if args.mode == 'camera' else ['pose', 'terrain'])
    if not args.without_segmentation:
        required.append('overlay')
    if args.with_prediction and args.mode != 'camera' and not args.expect_no_prediction:
        required.extend(['future_path', 'trajectory', 'prediction', 'reference_path'])
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
        '--ros-args', '-p', 'map_frame:='+args.map_frame,
        '-p', 'camera_frame:='+args.camera_name+'_camera_link',
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
        'prediction_publishers': node.count_publishers('/predict_output') == (
            1 if args.with_prediction else 0),
        'camera_ready': camera_state.get('ready') is True,
        'rgb_cloud_shared_stamps': len(stamps['rgb'] & stamps['cloud']) > 5,

    }
    if args.mode != 'camera':
        checks.update({
            'fusion_ready': state.get('ready') is True,
            'one_camera_tf_owner': tf_report['passed'],
            'pose_frame_shared_stamps': len(stamps['pose'] & stamps['rgb']) > 5,
            'wall_timestamps': bool(ages['rgb']) and all(-500 <= a < 2000 for a in ages['rgb']),
            'camera_time_verified': state.get('camera_time_verified') is True,
        })
    if args.mode == 'deployment':
        checks['native_profile'] = state.get('source_profile') == 'native'
        checks['mission_prediction'] = args.source_profile == 'native' and counts['future_path'] > 0
        checks['no_private_path_publisher'] = node.count_publishers('/lr/future_path/mavlink') == 0
        if state.get('camera_time_mode') == 'fcu_utc':
            checks['shared_utc_anchor'] = calibration.get('utc_ready') is True and (
                0 <= calibration.get('utc_age_sec', -1) < 2.)
    if args.expect_degraded:
        checks['fusion_blocked'] = state.get('ready') is False
        checks['no_fused_outputs'] = not any(counts[k] for k in (
            'pose', 'terrain', 'prediction', 'trajectory', 'future_path'))
        checks['no_camera_map_tf'] = tf_report['camera_tf_owner_count'] == 0
    if args.expect_no_prediction:
        checks['prediction_blocked'] = not any(counts[k] for k in (
            'future_path', 'trajectory', 'prediction', 'reference_path'))
    report = {'passed': all(checks.values()), 'checks': checks, 'source': state,
              'diagnostics': diagnostics,
              'camera': camera_state, 'mode': args.mode, 'calibration': calibration, 'tf': tf_report,
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
