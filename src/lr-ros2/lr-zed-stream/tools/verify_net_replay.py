"""Measure a running NET deployment and optionally exercise its SVO controls.

Run inside the host ROS environment. --exercise changes playback position.
CameraInfo tracks RGB frame stamps without adding a raw RGB subscription.
The existing RViz preset should be running for representative network load.
"""

import argparse
from collections import defaultdict, deque
import json
from pathlib import Path
import statistics
import time

from grid_map_msgs.msg import GridMap
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMessage
import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from safety_perception_msgs.msg import PredictionOutput
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_srvs.srv import Trigger
from zed_msgs.msg import SvoStatus
from zed_msgs.srv import SetSvoFrame


def stamp_ns(stamp):
    """Convert a ROS timestamp without losing nanoseconds to float rounding."""
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def main():
    """Observe frame timing on the host's monotonic clock, then check controls."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=20)
    parser.add_argument('--exercise', action='store_true')
    parser.add_argument('--require-prediction', action='store_true')
    parser.add_argument('--pause-seconds', type=float, default=65)
    parser.add_argument('--interface', default='enp4s0')
    parser.add_argument('--output', default='/tmp/lr-net-verification.json')
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('verify_net_replay')
    counts = defaultdict(int)
    stamps = defaultdict(lambda: deque(maxlen=4096))
    latencies = defaultdict(list)
    clock_arrivals = {}
    state = {'clock': 0, 'status': None}
    source = '/zed_source/zed_node'
    camera = '/zed/zed_node'

    def on_clock(message):
        value = stamp_ns(message.clock)
        state['clock'] = value
        clock_arrivals.setdefault(value, time.monotonic())
        if len(clock_arrivals) > 4096:
            del clock_arrivals[next(iter(clock_arrivals))]

    def on_frame(name, message):
        counts[name] += 1
        value = stamp_ns(message.header.stamp)
        stamps[name].append(value)
        if value in clock_arrivals:
            delay = (time.monotonic() - clock_arrivals[value]) * 1000
            if 0 <= delay < 10000:
                latencies[name].append(delay)

    node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
    node.create_subscription(
        SvoStatus, source + '/status/svo', lambda m: state.update(status=m), 10
    )
    for name, kind, topic in [
        ('rgb', CameraInfo, camera + '/rgb/color/rect/camera_info'),
        ('cloud', PointCloud2, camera + '/point_cloud/cloud_registered'),
        ('overlay', Image, '/segmentation/overlay'),
        ('pose', PoseStamped, '/lr/mavlink/pose'),
        ('future_path', PathMessage, '/lr/future_path/ground_truth'),
        ('prediction', PredictionOutput, '/predict_output'),
    ]:
        node.create_subscription(
            kind, topic, lambda m, n=name: on_frame(n, m), qos_profile_sensor_data
        )
    node.create_subscription(
        GridMap, '/terrain_geometry/grid_map',
        lambda m: counts.update(terrain=counts['terrain'] + 1), qos_profile_sensor_data,
    )
    toggle = node.create_client(Trigger, source + '/toggle_svo_pause')
    seek = node.create_client(SetSvoFrame, source + '/set_svo_frame')

    def until(predicate, seconds=20):
        deadline = time.monotonic() + seconds
        while not predicate() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        if not predicate():
            raise RuntimeError('Timed out waiting for replay state or received frame')

    def spin(seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)

    def call(client, request):
        until(client.service_is_ready)
        future = client.call_async(request)
        until(future.done)
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)
        return response

    def set_paused(paused):
        until(lambda: state['status'] is not None)
        if (state['status'].status == SvoStatus.STATUS_PAUSED) != paused:
            call(toggle, Trigger.Request())
        expected = SvoStatus.STATUS_PAUSED if paused else SvoStatus.STATUS_PLAYING
        until(lambda: state['status'].status == expected)

    def seek_frame(frame):
        call(seek, SetSvoFrame.Request(frame_id=frame))
        # Paused status is emitted immediately; subsequent frame/clock are
        # checked independently because DDS and NET have different delivery paths.
        until(lambda: state['status'].frame_id >= frame)

    def net_bytes():
        root = Path('/sys/class/net') / args.interface / 'statistics'
        return {key: int((root / (key + '_bytes')).read_text()) for key in ('rx', 'tx')}

    report = {'interface': args.interface, 'checks': {}, 'measurements': {}}
    try:
        required = ['rgb', 'cloud', 'overlay', 'terrain']
        if args.require_prediction:
            required.extend(['pose', 'future_path', 'prediction'])
        print(f'Waiting for {required}...', flush=True)
        until(lambda: all(counts[name] for name in required), 60)
        if node.count_publishers('/clock') != 1:
            raise RuntimeError('Expected exactly one /clock publisher')
        report['checks']['single_clock_publisher'] = True
        counts.clear()
        for values in stamps.values():
            values.clear()
        latencies.clear()
        before = net_bytes()
        started = time.monotonic()
        print(f'Measuring playback for {args.duration:g}s...', flush=True)
        spin(args.duration)
        elapsed = time.monotonic() - started
        after = net_bytes()
        report['measurements']['duration_sec'] = elapsed
        report['measurements']['received_hz'] = {
            name: count / elapsed for name, count in counts.items()
        }
        report['measurements']['ethernet_mbit_sec'] = {
            key: (after[key] - before[key]) * 8 / elapsed / 1e6 for key in before
        }
        report['measurements']['clock_to_return_ms'] = {
            name: {
                'samples': len(values), 'median': statistics.median(values),
                'p95': sorted(values)[min(len(values) - 1, int(len(values) * .95))],
            } for name, values in latencies.items() if values
        }
        report['checks']['rgb_cloud_shared_stamps'] = len(
            set(stamps['rgb']).intersection(stamps['cloud'])
        )
        if not report['checks']['rgb_cloud_shared_stamps']:
            raise RuntimeError('RGB and cloud have no matching source timestamp')

        if args.exercise:
            print(f'Pausing for {args.pause_seconds:g}s...', flush=True)
            set_paused(True)
            spin(1)
            frozen = state['clock']
            spin(args.pause_seconds)
            assert state['clock'] == frozen, 'Clock advanced while paused'
            assert node.count_publishers(camera + '/point_cloud/cloud_registered') == 1
            report['checks']['long_pause_sec'] = args.pause_seconds
            set_paused(False)
            until(lambda: stamps['rgb'][-1] > frozen)
            report['checks']['resume_after_pause'] = True

            set_paused(True)
            for delta in (-150, 300):
                previous = state['clock']
                frame = max(0, state['status'].frame_id + delta)
                seek_frame(frame)
                until(lambda: state['clock'] != previous)
                target = state['clock']
                until(lambda: stamps['rgb'][-1] == target)
                assert state['status'].status == SvoStatus.STATUS_PAUSED
                report['checks']['paused_seek_' + str(delta)] = True
            set_paused(False)
            for delta in (150, -300):
                previous = state['clock']
                seek_frame(max(0, state['status'].frame_id + delta))
                until(lambda: state['clock'] != previous)
                spin(2)
                report['checks']['playing_seek_' + str(delta)] = True

            print('Checking EOF and Replay...', flush=True)
            seek_frame(state['status'].total_frames - 5)
            until(lambda: state['status'].status == SvoStatus.STATUS_END)
            frozen = state['clock']
            spin(2)
            assert state['clock'] == frozen
            report['checks']['eof_clock_frozen'] = True
            seek_frame(0)
            until(lambda: state['status'].status == SvoStatus.STATUS_PLAYING)
            until(lambda: stamps['rgb'][-1] < frozen)
            report['checks']['replay_after_eof'] = True
        report['passed'] = True
    except BaseException as error:
        report['passed'] = False
        report['error'] = str(error)
        raise
    finally:
        Path(args.output).write_text(json.dumps(report, indent=2) + '\n')
        print(json.dumps(report, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
