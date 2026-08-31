# lr-ros2 Integration Review Requirements

## 1. Purpose

This branch (`integration/real-terrain-segmentation`) is a **reproducible
integration snapshot** used by the Landfill Rover **Prediction** pipeline.

It is **not** intended to replace the team's canonical development branch on the
original upstream repository.

The purpose of this document is to let the original `lr-ros2` developer review
and improve perception locally **without breaking downstream integration**.

Integration with Prediction is **technically working** on this snapshot, but
**terrain normal output may not be physically correct**. Visual inspection has
raised concerns. We are **not** claiming the terrain normal algorithm is
definitely wrong — the owner should inspect, validate, and fix if needed while
preserving the integration contract described below.

---

## 2. Repositories and Baseline

| Item | Value |
|------|-------|
| **Fork (integration snapshot)** | https://github.com/sonsonha/lr-ros2 |
| **Integration branch** | `integration/real-terrain-segmentation` |
| **Validated SHA** | `cb580c0` |
| **Original upstream (reference)** | https://github.com/hachitrung024/lr-ros2 |

The integration branch currently contains these three commits on top of upstream
`main`:

| SHA | Subject |
|-----|---------|
| `79c57fe` | Preserve real terrain integration for prediction |
| `7b3bb31` | Fix segmentation model startup parameters |
| `cb580c0` | Fix object-filter projection frame for map clouds |

**DO NOT develop new features directly on the validated integration branch.**

Treat `integration/real-terrain-segmentation` @ `cb580c0` as a **frozen reference
snapshot** for Prediction integration testing and comparison.

---

## 3. What Changed During Integration

The following changes were made to support Prediction integration. The **core
terrain-fitting algorithm was not redesigned** — changes are integration,
configuration, launch, and coordinate-frame related.

### Terrain

Integration, configuration, and launch changes were made so the real **map-frame
point cloud** and terrain geometry can participate in the Prediction pipeline.

### Segmentation

Model startup and launch parameters were corrected so real segmentation can run
in the current RTX/Docker integration runtime.

### Object projection

A coordinate-frame issue was fixed when using a point cloud **already
transformed into `map`**.

The current real pipeline uses:

```text
/lr/point_cloud/cloud_in_map
```

This point cloud is already in:

```text
frame_id = map
```

**Do not** accidentally treat map-frame points as camera-frame points or apply
transforms twice.

---

## 4. Current Integration Boundary

```text
ZED / point cloud
        |
        v
/lr/point_cloud/cloud_in_map
        |
        v
lr-ros2
  |
  +-- segmentation
  |
  +-- terrain geometry
  |
  +-- object filtering / object_boxes_3d
        |
        v
lr_prediction_bridge
        |
        +-- /geometry
        +-- /tracked_objects
        |
        v
Prediction
```

### Important outputs from lr-ros2 consumed by integration

| Topic | Role |
|-------|------|
| `/segmentation/overlay` | Visual/debug; segmentation health |
| `/terrain_geometry/grid_map` | Terrain layers including normals |
| `/terrain_geometry/object_boxes_3d` | Real 3D object boxes for tracked objects |

### Canonical world frame

```text
map
```

All downstream Prediction inputs use **`map`** as the world frame.

---

## 5. HIGH PRIORITY REVIEW — Terrain Normal Validation

> **Integration works technically, but terrain normals should now be reviewed for
> PHYSICAL CORRECTNESS.**

The owner should inspect and validate the items below **without breaking** the
already-working integration contract (Section 6).

### A. Normal frame

Determine **exactly** which frame the computed normal is expressed in.

Confirm whether it is:

- sensor/camera frame
- base/rover frame
- map frame

Downstream integration expects the terrain representation to be **consistent with
the map-frame trajectory**.

**Do not infer the frame from variable names** — verify from source code and TF
operations.

### B. Normal normalization

Check:

```text
sqrt(nx² + ny² + nz²) ≈ 1
```

for valid normals.

Identify any code path that can output **non-unit** normals.

### C. Normal orientation / sign

Check whether normals can randomly flip between `n` and `-n` for the same terrain
surface.

Verify the **intended convention** from code.

For normal ground, if the intended convention is upward-facing in `map`, verify
that `nz` is positive and usually dominant.

**IMPORTANT:** Confirm the intended convention from code **before** changing it.

### D. Physical sanity

For approximately flat ground:

```text
nx ≈ 0
ny ≈ 0
|nz| ≈ 1
```

(depending on the documented sign convention).

For an inclined plane, the normal direction should change consistently with the
visible slope. Check whether derived slope angles match physical terrain.

### E. Spatial alignment

Verify that the terrain/grid cell used for a trajectory step actually
corresponds to terrain underneath/around that trajectory location.

Check:

- map X/Y alignment
- grid origin
- grid resolution
- indexing
- axis order
- any row/column inversion
- nearest-cell / interpolation behavior

A mathematically correct normal from the **wrong grid cell** is still an
incorrect Prediction input.

### F. Transform correctness

Audit every transform used between:

```text
camera/sensor  →  base  →  map
```

Especially ensure the integration fix for **map-frame point clouds** is not
accidentally undone.

Avoid:

```text
map point → camera transform → incorrectly reused as map
```

and avoid **double transforms**.

### G. Temporal alignment

Check timestamps between:

- point cloud
- terrain / grid map
- trajectory

Determine whether normals can be generated from **stale** cloud data while being
associated with a **newer** trajectory.

### H. Failure behavior

Review what happens when:

- no valid plane can be fitted
- too few points exist
- terrain is highly uneven
- normal is invalid
- grid cell is missing

Invalid terrain must be reported as **invalid/unavailable** rather than silently
producing a plausible-looking normal.

---

## 6. Integration Contracts — DO NOT BREAK

> **WARNING:** Changes inside algorithms are allowed. Do **not** casually change
> integration-facing interfaces.

Do **not** casually change:

- topic names
- message types
- `frame_id` semantics
- timestamp semantics
- coordinate conventions
- QoS
- grid-map layer names used downstream
- object box frame conventions

If any such change is necessary, report it as:

```text
BREAKING INTEGRATION CHANGE
```

with:

| Field | Content |
|-------|---------|
| Old behavior | … |
| New behavior | … |
| Reason | … |
| Downstream migration required | … |

…**before** merging it.

---

## 7. Development Workflow for the Owner

### Clone the integration fork

```bash
git clone https://github.com/sonsonha/lr-ros2.git
cd lr-ros2
```

### Checkout validated integration snapshot

```bash
git checkout integration/real-terrain-segmentation
git pull --ff-only
```

### Create a separate review branch

```bash
git switch -c review/terrain-normal-validation
```

**Do NOT commit directly to:**

```text
integration/real-terrain-segmentation
```

### Optionally add original upstream

```bash
git remote add upstream https://github.com/hachitrung024/lr-ros2.git
git fetch upstream
```

Do **not** merge or rebase `upstream/main` into the validated snapshot just for
this review. Keep the review based on the validated integration state so
results are comparable with Prediction integration tests.

---

## 8. Suggested Review Method

### Level 1 — Source / unit review

Can be done **without** the original SVO dataset.

Review:

- normal computation equations
- coordinate-frame transformations
- normalization
- sign convention
- grid indexing
- validity handling

Add focused unit tests where possible.

**Useful synthetic test cases:**

1. **Horizontal plane** — expected normal direction consistent with documented
   upward convention.
2. **Known X-axis slope** — generate points from a plane with known slope; verify
   recovered normal.
3. **Known Y-axis slope** — same validation.
4. **Rotated/translated cloud** — verify map-frame transformation does not alter
   physical normal incorrectly.
5. **Opposite normal ambiguity** — verify orientation is deterministic.

### Level 2 — Real-data review

If session data is available, inspect real point clouds together with:

- terrain grid
- trajectory
- terrain-normal markers

Select representative locations:

- approximately flat ground
- visible incline
- uneven terrain
- sparse-data region

For every sampled point record:

| Field | Value |
|-------|-------|
| trajectory XY | … |
| grid cell | … |
| normal XYZ | … |
| normal magnitude | … |
| computed slope | … |
| visual terrain expectation | … |
| valid / invalid | … |

Do **not** judge correctness only from marker colors.

---

## 9. Regression Requirements

Before proposing a terrain fix, verify that these integration behaviors **still
work**:

- segmentation starts normally
- `/segmentation/overlay` publishes
- `/lr/point_cloud/cloud_in_map` remains `map`-frame
- terrain grid publishes
- object filtering still produces real 3D boxes
- object boxes remain correctly located in `map`
- Prediction geometry adapter can still consume terrain data

Terrain fixes must **not** regress the already-fixed object projection path.

---

## 10. What NOT to Change

This repository does **not** own:

- Prediction collision physics
- Prediction rollover physics
- PredictionRuntime cycle semantics
- Decision logic
- canonical Prediction messages

Do **not** modify `prediction-rover` while reviewing terrain unless a **confirmed
interface change** requires coordinated downstream work.

---

## 11. Required Review Report

The developer should return a concise report containing:

1. Terrain normal algorithm currently used
2. Frame in which normals are computed/output
3. Whether normals are unit length
4. Orientation/sign convention
5. How slope is derived
6. Whether grid/trajectory spatial alignment is correct
7. Whether timestamps are correctly aligned
8. Bugs found
9. Files changed
10. Tests added/run
11. Before/after examples
12. Any integration-contract changes
13. Whether existing object-box integration remains compatible

If the current output is believed to be **already correct**, provide **evidence**
showing why (synthetic tests, real-data samples, frame audit).

---

## 12. Integration Acceptance Checklist

- [ ] Terrain normal frame documented
- [ ] Terrain normal normalized
- [ ] Sign/orientation deterministic
- [ ] Flat-plane synthetic test passes
- [ ] Known-slope synthetic test passes
- [ ] Grid indexing verified
- [ ] Trajectory/grid spatial alignment verified
- [ ] Timestamp alignment reviewed
- [ ] Invalid terrain behavior verified
- [ ] Map point cloud not double-transformed
- [ ] Object 3D boxes still correct
- [ ] Existing topics/messages/frame conventions preserved
- [ ] Downstream geometry adapter still compatible
- [ ] Real-data visual sanity check completed (if dataset available)

---

## 13. Contact / Handoff Rule

The validated branch must remain **reproducible**.

New fixes should arrive as:

```text
review branch → commit → PR
```

rather than rewriting:

```text
integration/real-terrain-segmentation
```

This allows Prediction integration to test candidate changes before promoting
them into a new integration snapshot.
