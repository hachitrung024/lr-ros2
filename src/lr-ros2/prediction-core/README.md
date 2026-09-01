# Prediction Core

`prediction_core` is the ROS-independent runtime algorithm package integrated
from the provided `prediction-rover` source. It is now built directly by
`colcon`; the top-level reference repository is not a runtime dependency.

The engine returns safety evidence for each discrete trajectory pose:

- exact 2D oriented-footprint collision candidates and clearance;
- terrain-normal to predicted roll and pitch;
- Static Stability Margin (SSM) and normalized Static SSM;
- edge Stability Moments when rover acceleration is available;
- point-mass ZMP and secondary dynamic evidence.

The output intentionally contains no severity thresholds, risk score, or
Stop/Go decision. Collision is sampled at trajectory poses rather than swept
continuously between poses. Rotational inertia, full rigid-body ZMP, LTR, and
FASM are not implemented.

## Rover parameters

`config/rover.reference.yaml` defines the required schema:

```yaml
rover:
  mass_kg: 100.0
  body_length_m: 1.05
  body_width_m: 0.90
  body_height_m: 0.50
  support_length_m: 0.75
  support_width_m: 0.88
  ground_clearance_m: 0.15
  com_x_m: 0.0
  com_y_m: 0.0
  com_height_m: 0.33
prediction:
  collision_margin_m: 0.20
```

These values are references only. Replace them with measured/CAD values before
using the evidence in a field safety system.

`static` prediction does not need rover state. `dynamic` prediction requires a
valid kinematic acceleration excluding gravity; unavailable acceleration is
represented as unavailable, never as zero. External wrench input is optional.
