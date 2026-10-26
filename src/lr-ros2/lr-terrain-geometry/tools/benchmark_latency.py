#!/usr/bin/env python3
"""Benchmark terrain with the headless pipeline, or replay its captured inputs.

Run on the Jetson after sourcing the workspace. Live mode owns and stops its
launch process; do not run it alongside an existing backend. Pickle captures
are local benchmark artifacts and must only be loaded from trusted runs.
"""

import argparse
from collections import Counter
from dataclasses import asdict
import json
import os
from pathlib import Path
import pickle
import signal
import subprocess
import time

import psutil
import yaml


def distribution(values):
    values = list(values)
    if not values:
        return None
    return dict(zip(('p50', 'p95', 'max'), map(float, np.percentile(values, [50, 95, 100]))))


def snapshot(estimator, result, timestamp):
    planes = [
        [*p['column_key'], float(p['grid_center_world'][2]), float(p['slope_deg']),
         float(p['rmse']), float(p['inlier_ratio'])]
        for p in result.accepted_planes
    ]
    return {
        'stamp': timestamp, 'fit_cycle': result.fit_cycle,
        'fitted_cells': estimator.last_fitted_cells,
        'accepted': len(planes), 'rejected': len(result.rejected_planes),
        'pending': len(result.debug_cells), 'cells': result.cell_count,
        'points': result.point_count, 'planes': planes,
        'roi_points': estimator.last_roi_points,
        'corridor_points': estimator.last_corridor_points,
    }


def summarize(rows):
    result = {'samples': len(rows)}
    if not rows:
        return result
    for name in ('processing_ms', 'tf_ms', 'decode_ms', 'extract_ms', 'accumulate_ms',
                 'fit_ms', 'publish_ms', 'sim_age_ms', 'accepted', 'rejected', 'pending',
                 'cells', 'points', 'fitted_cells'):
        result[name] = distribution(r[name] for r in rows if name in r)
    result['fit_callback_ms'] = distribution(
        r['processing_ms'] for r in rows if r['fitted_cells'] and 'processing_ms' in r
    )
    result['nonfit_callback_ms'] = distribution(
        r['processing_ms'] for r in rows if not r['fitted_cells'] and 'processing_ms' in r
    )
    result['svo_span_sec'] = rows[-1]['stamp'] - rows[0]['stamp']
    result['processed_hz_svo'] = (len(rows) - 1) / max(result['svo_span_sec'], 1e-9)
    if 'wall_sec' in rows[0]:
        span = rows[-1]['wall_sec'] - rows[0]['wall_sec']
        result['processed_hz_wall'] = (len(rows) - 1) / max(span, 1e-9)
    result['accepted_rmse_m'] = distribution(p[4] for r in rows for p in r['planes'])
    result['corridor_status_counts'] = dict(Counter(r.get('corridor_status', '') for r in rows))
    return result


def replay(args, parameters):
    capture = pickle.loads(Path(args.replay).read_bytes())
    config = TerrainGeometryConfig(**{
        k.removeprefix('terrain.'): v for k, v in parameters.items() if k.startswith('terrain.')
    })
    estimator = TerrainGeometryEstimator(config)
    rows, first_accepted, latencies = [], set(), []
    for frame in capture['frames']:
        points = frame['points']
        # These points have already passed ROI, corridor and frame-cap filtering.
        # Keep that exact input while comparing fitting and accumulation settings.
        estimator._extract_valid_points = lambda *_args, p=points: p
        corridor = None if frame['path'] is None else TrajectoryCorridor(
            frame['path'], frame['half_width']
        )
        started = time.perf_counter()
        result = estimator.update(
            points, frame['pose'], frame['stamp'], corridor=corridor,
            corridor_required=frame['corridor_required'],
        )
        elapsed = (time.perf_counter() - started) * 1000
        row = snapshot(estimator, result, frame['stamp'])
        row['processing_ms'] = elapsed
        for p in result.accepted_planes:
            key = tuple(p['column_key'])
            if key not in first_accepted:
                first_accepted.add(key)
                latencies.append(frame['stamp'] - estimator._cells[key].first_seen)
        rows.append(row)
    measured = [r for r in rows if r['stamp'] >= capture['window_start']]
    return {
        'mode': 'replay', 'excludes': 'DDS, TF, cloud decoding and ROI filtering',
        'source_capture': str(args.replay), 'config': asdict(config),
        'summary': summarize(measured), 'first_accept_sec': distribution(latencies),
        'rows': rows,
    }


def live(args, parameters):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.parameter import Parameter
    import lr_terrain_geometry.node as terrain_node

    frames, rows, resources = [], [], []
    stage = {}
    first_accepted, latencies = set(), []
    window_start = None

    def timed(original, name):
        def call(*positional, **keywords):
            started = time.perf_counter()
            try:
                return original(*positional, **keywords)
            finally:
                stage[name] = (time.perf_counter() - started) * 1000
        return call

    class MeasuredNode(terrain_node.TerrainGeometryNode):
        def _process_point_cloud(self, message, timestamp, callback_started):
            stage.clear()
            self._bench_stamp = timestamp
            super()._process_point_cloud(message, timestamp, callback_started)

        def _finish_timing(self, callback_started):
            super()._finish_timing(callback_started)
            row = snapshot(self._estimator, self._last_result, self._bench_stamp)
            row.update(stage)
            row['processing_ms'] = self._last_processing_ms
            row['wall_sec'] = time.monotonic()
            clock = self.get_clock().now().nanoseconds / 1e9
            row['sim_age_ms'] = (clock - self._bench_stamp) * 1000
            row['corridor_status'] = self._corridor_status
            rows.append(row)
            for p in self._last_result.accepted_planes:
                key = tuple(p['column_key'])
                if key not in first_accepted:
                    first_accepted.add(key)
                    latencies.append(self._bench_stamp - self._estimator._cells[key].first_seen)

    rclpy.init(args=[])
    node = MeasuredNode(parameter_overrides=[
        Parameter(k, value=v) for k, v in {**parameters, 'use_sim_time': True}.items()
    ])
    estimator = node._estimator
    original_extract = estimator._extract_valid_points
    extracted = [None]

    def extract(*positional):
        result = original_extract(*positional)
        extracted[0] = result
        return result

    estimator._extract_valid_points = timed(extract, 'extract_ms')
    original_update = estimator.update

    def update(points, pose, timestamp, **kwargs):
        extracted[0] = None
        result = original_update(points, pose, timestamp, **kwargs)
        if args.capture:
            corridor = kwargs.get('corridor')
            frames.append({
                'points': np.empty((0, 3)) if extracted[0] is None else extracted[0],
                'pose': pose, 'stamp': timestamp,
                'path': None if corridor is None else corridor._points,
                'half_width': None if corridor is None else corridor.half_width_m,
                'corridor_required': kwargs.get('corridor_required', False),
            })
        return result

    estimator.update = update
    for target, name, metric in (
        (estimator, '_accumulate_world_points', 'accumulate_ms'),
        (estimator, '_fit_eligible_cells', 'fit_ms'),
        (node, '_publish_snapshot', 'publish_ms'),
        (node._tf_buffer, 'lookup_transform', 'tf_ms'),
        (terrain_node, 'point_cloud_to_xyz', 'decode_ms'),
    ):
        setattr(target, name, timed(getattr(target, name), metric))
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    command = [
        'ros2', 'launch', 'lr_display_rviz2', 'headless_zed_cam.launch.py',
        'camera_model:=zed2i', 'stream_svo_mode:=true',
        f'stream_address:={args.stream_address}', f'stream_port:={args.stream_port}',
        f'mavros_fcu_url:={args.mavros_fcu_url}',
        f'mavlink_db_path:={args.mavlink_db_path}', 'future_path:=true',
        f'segmentation_model_path:={args.model}',
        'start_terrain_node:=false', 'start_path_prediction_node:=false',
    ]
    own_process = psutil.Process()
    process_cache = {own_process.pid: own_process}
    own_process.cpu_percent()
    log = (args.output / 'launch.log').open('w')
    child = subprocess.Popen(
        command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
        env=args.pipeline_environment,
    )
    started, next_sample, first_clock = time.monotonic(), 0, None
    error = None
    try:
        while time.monotonic() - started < args.timeout:
            executor.spin_once(timeout_sec=0.1)
            now = time.monotonic()
            clock = node.get_clock().now().nanoseconds / 1e9
            if clock > 0 and first_clock is None:
                first_clock = clock
                window_start = clock + args.warmup
            if first_clock is not None and clock >= window_start + args.duration:
                break
            if child.poll() is not None:
                error = f'launch exited with status {child.returncode}'
                break
            if now < next_sample:
                continue
            next_sample = now + 1
            sample = []
            for process in [own_process, *psutil.Process(child.pid).children(recursive=True)]:
                try:
                    cached = process_cache.setdefault(process.pid, process)
                    sample.append({
                        'pid': process.pid, 'command': ' '.join(cached.cmdline()),
                        'cpu_percent': cached.cpu_percent(), 'rss_bytes': cached.memory_info().rss,
                    })
                except psutil.Error:
                    pass
            resources.append({'stamp': clock, 'wall_sec': now, 'processes': sample})
        else:
            error = 'wall-time timeout'
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGTERM)
                child.wait(timeout=10)
        executor.shutdown()
        counters = {
            'tf_failures': node._tf_failure_count, 'malformed_clouds': node._malformed_cloud_count,
            'marker_subscribers': node._marker_publisher.get_subscription_count(),
            'heatmap_subscribers': node._heatmap_publisher.get_subscription_count(),
        }
        node.destroy_node()
        rclpy.shutdown()
        log.close()
    if args.capture:
        Path(args.capture).write_bytes(pickle.dumps({
            'frames': frames, 'window_start': window_start,
            'parameters': parameters,
        }, protocol=pickle.HIGHEST_PROTOCOL))
    measured = [r for r in rows if window_start is not None and r['stamp'] >= window_start]
    return {
        'mode': 'live', 'command': command, 'parameters': parameters, 'error': error,
        'first_clock': first_clock, 'window_start': window_start,
        'warmup_svo_sec': args.warmup, 'duration_svo_sec': args.duration,
        'summary': summarize(measured), 'first_accept_sec': distribution(latencies),
        'counters': counters, 'resources': resources, 'rows': rows,
    }


def main():
    global np, TerrainGeometryConfig, TerrainGeometryEstimator, TrajectoryCorridor
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--replay', type=Path)
    parser.add_argument('--capture', type=Path)
    parser.add_argument('--stream-address')
    parser.add_argument('--stream-port', type=int)
    parser.add_argument('--mavros-fcu-url')
    parser.add_argument('--mavlink-db-path', default='')
    parser.add_argument('--model', default='/zed/lr-ros2/models/best.pt')
    parser.add_argument('--warmup', type=float, default=5)
    parser.add_argument('--duration', type=float, default=30)
    parser.add_argument('--timeout', type=float, default=180)
    parser.add_argument('--blas-threads', type=int)
    args = parser.parse_args()
    if not args.replay and not all((args.stream_address, args.stream_port, args.mavros_fcu_url)):
        parser.error('NET measurement requires --stream-address, --stream-port '
                     'and --mavros-fcu-url')
    args.pipeline_environment = os.environ.copy()
    if args.blas_threads is not None:
        if args.blas_threads < 1:
            parser.error('--blas-threads must be positive')
        os.environ['OPENBLAS_NUM_THREADS'] = str(args.blas_threads)
    import numpy as np
    from lr_terrain_geometry.estimator import TerrainGeometryConfig, TerrainGeometryEstimator
    from lr_terrain_geometry.trajectory_corridor import TrajectoryCorridor
    args.output.mkdir(parents=True, exist_ok=True)
    parameters = yaml.safe_load(args.config.read_text())['terrain_geometry']['ros__parameters']
    report = replay(args, parameters) if args.replay else live(args, parameters)
    report['blas_threads_requested'] = args.blas_threads
    (args.output / 'report.json').write_text(json.dumps(report, indent=2) + '\n')
    compact = {k: v for k, v in report.items() if k not in ('rows', 'resources')}
    print(json.dumps(compact, indent=2))


if __name__ == '__main__':
    main()
