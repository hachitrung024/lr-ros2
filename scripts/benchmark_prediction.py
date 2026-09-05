#!/usr/bin/env python3
"""Measure the same SVO interval before/after changes, without RViz.

Run after sourcing the workspace. Reports include raw per-second resource
samples and topic counts; this observer does not subscribe to debug images.
"""

import argparse
from collections import Counter
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import numpy as np
import psutil
import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from safety_perception_msgs.msg import PredictionOutput, Trajectory
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection3DArray


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--svo', required=True)
    parser.add_argument('--model', default='models/best.pt')
    parser.add_argument('--baseline-source', help='Original display launch file')
    parser.add_argument('--warmup', type=float, default=10.0)
    parser.add_argument('--duration', type=float, default=60.0)
    args = parser.parse_args()
    destination = Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    command = ['ros2', 'launch', 'lr_display_rviz2', 'display_zed_cam.launch.py']
    environment = os.environ.copy()
    temporary = None
    if args.baseline_source:
        # The old launch has no start_rviz switch. Exclude only its RViz action.
        source = Path(args.baseline_source).read_text()
        needle = 'nodes = [\n        rviz2_node,\n        zed_wrapper_launch\n    ]'
        if needle not in source:
            raise ValueError('Unexpected baseline launch structure')
        temporary = tempfile.TemporaryDirectory(prefix='lr-baseline-')
        launch = Path(temporary.name) / 'baseline.launch.py'
        launch.write_text(source.replace(needle, 'nodes = [zed_wrapper_launch]'))
        command = ['ros2', 'launch', str(launch)]
        # Run the archived LR Python implementation with the same installed
        # ROS messages and ZED component as the after run.
        package_root = Path(args.baseline_source).resolve().parents[2]
        roots = [
            str(path) for path in sorted(package_root.iterdir()) if (path / 'setup.py').is_file()
        ]
        environment['PYTHONPATH'] = os.pathsep.join(roots + [environment.get('PYTHONPATH', '')])
    else:
        command.append('start_rviz:=false')
    command += [
        'camera_model:=zed2i',
        'publish_svo_clock:=true',
        f'segmentation_model_path:={args.model}',
        'mavlink:=true',
        'future_path:=true',
        f'svo_path:={args.svo}',
        'prediction_profile:=dynamic',
    ]
    rclpy.init()
    node = rclpy.create_node('prediction_benchmark')
    counts = Counter()
    received_cycles = set()
    completed_cycles = set()
    trajectory_stamps = {}
    latencies = []
    diagnostics = []
    clock = [None, None]
    observing = [False]
    wall_window = [None, None]

    def on_clock(message):
        stamp = seconds(message.clock)
        if clock[0] is None:
            clock[0] = stamp
        clock[1] = stamp
        observing[0] = args.warmup <= stamp - clock[0] < args.warmup + args.duration
        if observing[0]:
            if wall_window[0] is None:
                wall_window[0] = time.monotonic()
            wall_window[1] = time.monotonic()

    def count(name, message):
        if name == 'trajectory':
            trajectory_stamps[int(message.trajectory_id)] = seconds(message.header.stamp)
        if observing[0]:
            counts[name] += 1
            if name == 'trajectory':
                received_cycles.add(int(message.trajectory_id))
            elif name == 'prediction':
                completed_cycles.add(int(message.source_trajectory_id))
                source = trajectory_stamps.get(int(message.source_trajectory_id))
                if source is not None:
                    latencies.append(seconds(message.header.stamp) - source)
            elif name == 'diagnostics':
                diagnostics.append(
                    [
                        {
                            'name': s.name,
                            'message': s.message,
                            'values': {v.key: v.value for v in s.values},
                        }
                        for s in message.status
                    ]
                )

    node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data)
    for name, topic, kind in (
        ('trajectory', '/trajectory', Trajectory),
        ('prediction', '/predict_output', PredictionOutput),
        ('mask', '/segmentation/instance_mask', Image),
        ('boxes', '/segmentation/boxes_3d', Detection3DArray),
        ('diagnostics', '/diagnostics', DiagnosticArray),
        ('diagnostics', '/prediction/diagnostics', DiagnosticArray),
        ('diagnostics', '/box_estimator_3d/diagnostics', DiagnosticArray),
    ):
        node.create_subscription(
            kind, topic, lambda msg, n=name: count(n, msg), qos_profile_sensor_data
        )
    samples = []
    processes = {}
    log = (destination / 'launch.log').open('w')
    child = subprocess.Popen(
        command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=environment
    )
    start = time.monotonic()
    next_sample = start
    error = None
    try:
        while time.monotonic() - start < 240:
            rclpy.spin_once(node, timeout_sec=0.05)
            if child.poll() is not None:
                error = f'launch exited with {child.returncode}'
                break
            if clock[0] is not None and clock[1] - clock[0] >= args.warmup + args.duration:
                break
            now = time.monotonic()
            if now < next_sample:
                continue
            next_sample = now + 1
            current = psutil.Process(child.pid).children(recursive=True)
            resource = []
            for process in current:
                try:
                    cached = processes.setdefault(process.pid, process)
                    resource.append(
                        {
                            'pid': process.pid,
                            'command': ' '.join(process.cmdline()),
                            'cpu_percent': cached.cpu_percent(),
                            'rss_bytes': process.memory_info().rss,
                        }
                    )
                except psutil.Error:
                    pass
            if observing[0]:
                gpu = subprocess.run(
                    [
                        'nvidia-smi',
                        '--query-gpu=memory.used,utilization.gpu',
                        '--format=csv,noheader,nounits',
                    ],
                    capture_output=True,
                    text=True,
                )
                samples.append(
                    {
                        'wall_sec': now - start,
                        'svo_stamp': clock[1],
                        'processes': resource,
                        'gpu': gpu.stdout.strip(),
                    }
                )
        else:
            error = '240 second wall-time timeout'
    finally:
        child.send_signal(signal.SIGINT) if child.poll() is None else None
        try:
            child.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGTERM)
            child.wait(timeout=10)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
        if temporary:
            temporary.cleanup()
    report = {
        'command': command,
        'error': error,
        'first_svo_stamp': clock[0],
        'last_svo_stamp': clock[1],
        'warmup_svo_sec': args.warmup,
        'duration_svo_sec': args.duration,
        'window_wall_sec': None if wall_window[0] is None else wall_window[1] - wall_window[0],
        'counts': dict(counts),
        'completed_cycle_fraction': len(received_cycles & completed_cycles)
        / max(1, len(received_cycles)),
        'boxes_per_mask': counts['boxes'] / max(1, counts['mask']),
        'prediction_latency_p50_sec': float(np.median(latencies)) if latencies else None,
        'prediction_latency_p95_sec': float(np.percentile(latencies, 95)) if latencies else None,
        'resources': samples,
        'diagnostics': diagnostics,
    }
    (destination / 'report.json').write_text(json.dumps(report, indent=2))
    print(
        json.dumps(
            {k: v for k, v in report.items() if k not in ('resources', 'diagnostics')}, indent=2
        )
    )


if __name__ == '__main__':
    main()
