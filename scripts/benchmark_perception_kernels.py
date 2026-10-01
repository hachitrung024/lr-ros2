#!/usr/bin/env python3
"""Compare terrain/depth grouping with a Git baseline and verify exact samples.

Source the ROS workspace first. This is a synthetic CPU microbenchmark; it does
not measure inference, DDS latency, or whole-pipeline throughput.
"""

import argparse
import ast
import json
from pathlib import Path
import subprocess
import sys
import time
import types

import numpy as np
from sensor_msgs.msg import CameraInfo

from lr_segmentation.depth_sampling import instance_point_clouds
from lr_terrain_geometry.estimator import TerrainGeometryConfig, TerrainGeometryEstimator


ROOT = Path(__file__).resolve().parents[1]


def source_at(revision, path):
    return subprocess.check_output(
        ['git', 'show', f'{revision}:{path}'], cwd=ROOT, text=True
    )


def median_ms(function, repeats):
    function()  # Warm up allocations before recording samples.
    samples = []
    for _ in range(repeats):
        started = time.perf_counter()
        function()
        samples.append((time.perf_counter() - started) * 1000)
    return float(np.median(samples))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-ref', required=True)
    parser.add_argument('--repeats', type=int, default=25)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error('--repeats must be positive')
    revision = subprocess.check_output(
        ['git', 'rev-parse', '--verify', f'{args.baseline_ref}^{{commit}}'], cwd=ROOT, text=True
    ).strip()
    old = types.ModuleType('terrain_before_cleanup')
    sys.modules[old.__name__] = old
    exec(source_at(
        revision, 'src/lr-ros2/lr-terrain-geometry/lr_terrain_geometry/estimator.py'
    ), old.__dict__)
    rng = np.random.default_rng(41)
    points = rng.uniform([-9, -9, -.1], [9, 9, .1], size=(12000, 3))

    def accumulate(estimator_type, config):
        estimator = estimator_type(config)
        estimator._frame_index = 1
        estimator._accumulate_world_points(points, np.zeros(3), 1.)
        return estimator

    def before_terrain():
        return accumulate(old.TerrainGeometryEstimator, old.TerrainGeometryConfig())

    def after_terrain():
        return accumulate(TerrainGeometryEstimator, TerrainGeometryConfig())

    before, after = before_terrain(), after_terrain()
    assert before._cells.keys() == after._cells.keys()
    for key in before._cells:
        np.testing.assert_array_equal(
            before._cell_points(before._cells[key]), after._cell_points(after._cells[key])
        )
    tree = ast.parse(source_at(
        revision, 'src/lr-ros2/lr-segmentation/lr_segmentation/box_estimator_3d.py'
    ))
    functions = [
        node for node in tree.body if isinstance(node, ast.FunctionDef)
        and node.name in ('_instance_points', '_depth_inlier_mask', '_camera_intrinsics')
    ]
    namespace = {'np': np, 'CameraInfo': CameraInfo}
    exec(
        compile(ast.Module(body=functions, type_ignores=[]), '<baseline-depth>', 'exec'),
        namespace,
    )
    height, width = 1080, 1920
    labels = np.zeros((height, width), dtype=np.uint16)
    for index in range(12):
        row, column = divmod(index, 4)
        labels[row * 360 + 20:row * 360 + 250, column * 480 + 20:column * 480 + 350] = index + 1
    depth = rng.uniform(2., 5., size=labels.shape).astype(np.float32)
    depth[::31, ::7] = np.nan
    info = CameraInfo(k=[1000., 0., 960., 0., 1000., 540., 0., 0., 1.])
    kwargs = dict(
        minimum_depth_m=.2, maximum_depth_m=20., sampling_stride=2,
        maximum_points=15000, depth_mad_scale=8., minimum_depth_band_m=.15,
    )

    def before_depth():
        return {
            int(label): namespace['_instance_points'](labels, depth, int(label), info, **kwargs)
            for label in np.unique(labels) if label
        }

    def after_depth():
        return instance_point_clouds(labels, depth, info, **kwargs)

    first, second = before_depth(), after_depth()
    assert first.keys() == second.keys()
    for key in first:
        np.testing.assert_array_equal(first[key], second[key])
    report = {
        'baseline_commit': revision,
        'iterations': args.repeats,
        'terrain': {
            'input_points': len(points), 'cells': len(after._cells),
            'before_ms': median_ms(before_terrain, args.repeats),
            'after_ms': median_ms(after_terrain, args.repeats),
        },
        'depth': {
            'image_shape': [height, width], 'instances': 12,
            'before_ms': median_ms(before_depth, args.repeats),
            'after_ms': median_ms(after_depth, args.repeats),
        },
        'numeric_equivalence': 'exact array equality for per-cell and per-instance samples',
    }
    encoded = json.dumps(report, indent=2)
    print(encoded)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + '\n')


if __name__ == '__main__':
    main()
