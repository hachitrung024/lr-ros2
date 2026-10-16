# Terrain latency tuning on Jetson

The selected profile reduces terrain callback p95 from 697 ms to 236 ms and
terrain CPU consumption from 238% to 71% in the measured headless SVO interval.
The largest improvement comes from limiting OpenBLAS to one thread in the
terrain process. Smaller fits allow a three-cloud cadence with approximately
the same CPU use as the original five-cloud cadence at one BLAS thread.

## Measurement conditions

- Jetson Orin Nano Developer Kit, container `zed-dev`, ROS 2 Humble.
- NumPy 1.26.4 / OpenBLAS; its unconstrained default was six threads.
- ZED SDK 5.4.1, ZED2i, SVO `zed_20260710_092420_0001.svo2`.
- Full headless backend: ZED, MAVLink future path, segmentation with `best.pt`,
  boxes, terrain and static prediction. Domain 42, host networking.
- Every live run starts the same SVO from its beginning, skips five SVO seconds
  for warmup, then measures 30 SVO seconds. These are individual runs rather
  than repeated statistical trials. Live runs can skip different input frames.
- No RViz or marker/heatmap subscribers. Output transport and UI latency are
  outside this measurement. Camera/cloud resolution and cloud rate were not
  changed.
- The existing local `voxel_size_m: 0.1` setting was preserved in every profile;
  it was already different from Git HEAD before this work.

The benchmark uses monotonic time for callback durations. SVO timestamps define
the common scene interval. `sim_age_ms` is relative to the replay clock and
must not be interpreted as sensor-to-UI wall latency.

## Live results

CPU is the mean process CPU sample during the observation window: 100% means
one CPU core. Cloud rates below use SVO timestamps. The benchmark replaces only
the terrain executable with an instrumented instance of the same node.

| Profile | BLAS threads | Samples | Callback p50 / p95, ms | Cloud/s | Terrain CPU | First-accept median, s |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| Original YAML | 6 | 87 | 221.5 / 696.8 | 2.90 | 238.2% | 2.479 |
| Smaller fits + cadence 3 | 6 | 102 | 210.1 / 332.4 | 3.49 | 199.8% | 1.139 |
| Original YAML | 1 | 137 | 107.6 / 237.6 | 4.53 | 70.0% | 1.206 |
| Selected profile | 1 | 137 | 111.0 / 235.9 | 4.60 | 71.1% | 0.670 |

First-accept delay is the time from a cell's first observation to its first
accepted plane, summarized over unique accepted cells across the complete run.
It includes accumulation and confirmation. Sparse cells can have much longer
delays; this is not the age of every plane in every output map.

Baseline stage p50: ROI/corridor extraction 131.8 ms, accumulation 28.9 ms,
fit cycle 416.2 ms, serialization/publication 4.1 ms. Selected profile:
40.4 / 20.6 / 64.9 / 4.2 ms respectively. Fit-cycle statistics include only
cycles that execute that stage. At most eight cells were fitted per cycle, so
lowering `max_cells_fit_per_cycle: 100` would not address this recording.

## Comparisons on identical inputs

The capture stores filtered sensor points, sensor-to-map poses, cloud stamps
and the selected trajectory. Replay reproduces accumulation and fitting while
excluding DDS, TF lookup, cloud decoding and ROI filtering. It does not support
evaluating a different cloud resolution or ingestion cap.

On the baseline capture, replay reproduced every accepted-plane value and
accepted-cell count exactly across all 109 callbacks. Reducing only the fit
budget preserved the accepted-cell sets in every measured snapshot. The p95
absolute differences on common accepted cells were 2.67 mm in elevation and
0.374 degrees in slope.

The denser capture from the selected live run was replayed with both original
and selected settings, each with one BLAS thread:

| Metric | Original | Selected |
| --- | ---: | ---: |
| Measured input clouds | 137 | 137 |
| Estimator update p95, ms | 90.30 | 66.59 |
| First-accept median, s | 2.010 | 0.670 |
| Accepted cells, median | 6 | 7 |
| Accepted-plane RMSE p95, mm | 15.65 | 16.04 |

No baseline-accepted cell was missing from the selected profile in any measured
snapshot. Mean accepted-cell-set Jaccard similarity was 0.991; the selected
profile sometimes accepted additional cells sooner. Common accepted-cell p95
absolute differences were 25.61 mm in elevation and 0.277 degrees in slope.
The cadence changes temporal smoothing, so differences include response timing.
These are comparisons to the original estimator, not accuracy against surveyed
ground truth. Acceptance thresholds were not relaxed.

## Applied changes

| Setting | Before | After |
| --- | ---: | ---: |
| `terrain.fit_every_frames` | 5 | 3 |
| `terrain.max_central_fit_points` | 1200 | 800 |
| `terrain.max_neighbor_fit_points` | 300 | 150 |
| `terrain.ransac_iterations` | 160 | 100 |
| `terrain.robust_refinement_iterations` | 3 | 2 |
| Terrain launch `OPENBLAS_NUM_THREADS` | Inherited, default 6 | 1 |

The thread limit is scoped to the terrain child process. Set
`TERRAIN_OPENBLAS_NUM_THREADS` before launch to override it. Other backend
processes retain their inherited environment. Accumulation, confirmation,
rejection, ROI, corridor, plane acceptance and heatmap settings were preserved.

The host source was previewed with rsync dry-run, synchronized without deletion,
and built on Jetson with:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to lr_display_rviz2
source install/setup.bash
```

Validation: 61 terrain package tests and 17 distributed/display launch tests
passed in the host ROS container. The normal headless launch was then restarted
on Jetson to verify the installed parameters and the terrain-only environment.
An eight-second runtime check received 116 clock and 32 grid-map messages;
all observed terrain diagnostics were OK with an active corridor. The TF
failure counter stayed at five and malformed-cloud count at zero during that
check. The terrain process had `OPENBLAS_NUM_THREADS=1`; segmentation had no
override. The verified parameters matched the selected YAML. The backend was
left running for host RViz inspection.

## Reproducing the benchmark

Use `src/lr-ros2/lr-terrain-geometry/tools/benchmark_latency.py` after sourcing
the workspace on Jetson. Stop the existing backend first: the script owns a
complete headless launch and shuts that launch down when it finishes.

```bash
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=0
python3 src/lr-ros2/lr-terrain-geometry/tools/benchmark_latency.py \
  --config src/lr-ros2/lr-terrain-geometry/config/terrain_geometry.yaml \
  --output /tmp/terrain-live --capture /tmp/terrain-inputs.pkl --blas-threads 1
python3 src/lr-ros2/lr-terrain-geometry/tools/benchmark_latency.py \
  --config src/lr-ros2/lr-terrain-geometry/config/terrain_geometry.yaml \
  --output /tmp/terrain-replay --replay /tmp/terrain-inputs.pkl --blas-threads 1
```

The model/SVO paths default to the files used above. Optional `--warmup`,
`--duration` and `--timeout` control the observation interval. `psutil`, NumPy
and PyYAML are required. Only load pickle captures from trusted local runs.

Raw reports, captured inputs, original/candidate YAML files and runtime logs
are retained on Jetson under `/zed/lr-ros2/log/terrain-tuning-20261016/` (host
mount `/home/hachi/Projects/zed/lr-ros2/log/terrain-tuning-20261016/`). Small
report copies are also available on the development host under
`/tmp/lr-terrain-tuning-20261016/`. Captures and build artifacts were not added
to source control.
