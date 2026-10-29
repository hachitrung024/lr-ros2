# Repository instructions

The root [README.md](README.md) is the single project guide for architecture,
requirements, source synchronization, build commands and runtime CLI examples.
Keep project documentation there and write it in English.

- Preserve existing implementation changes, recordings, models and the pinned
  upstream ZED submodule when performing unrelated cleanup.
- Keep `tools/host_replay/` independently runnable after copying it outside this
  repository. It must not import ROS or workspace source packages.
- Preserve the original ROS sender packages as replay baselines.
- Follow the README source synchronization procedure: preview before rsync,
  transfer only tracked/non-ignored `src/lr-ros2/` files, and do not delete
  remote files or send build products, recordings, models or `tools/`.
- Stop active ROS launches before rebuilding affected packages. Build the host
  through `lr_mavlink_replay` and `lr_display_rviz2`, and Jetson through
  `lr_display_rviz2`; source the resulting workspace before running nodes.
- Archive retired ROS executable/launch entry points and stale installed copies
  in `.legacy-archive/` with `COLCON_IGNORE` before rebuilding.
- Keep network endpoints explicit. The deployment uses ROS domain 42 with
  non-local DDS. Use isolated domains for integration tests.
- In always-on mode, use system time, keep Jetson free of recording DB access,
  and let the MAVROS pose adapter alone own the camera map TF.
