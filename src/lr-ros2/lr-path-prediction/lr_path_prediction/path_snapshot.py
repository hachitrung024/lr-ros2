"""Validate the exact relationship between dense source paths and evaluated steps."""

import math


def stamp_ns(stamp):
    """Return integer nanoseconds for matching cycle headers."""
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def validate_snapshot(snapshot, frame, trajectory=None):
    """Reject partial, non-finite, or mismatched source mappings before drawing."""
    if snapshot.header.frame_id != frame or not 1 <= len(snapshot.poses) <= 1000:
        raise ValueError('Invalid path snapshot frame or pose count')
    if len(snapshot.step_ids) != len(snapshot.source_pose_indices):
        raise ValueError('Incomplete step-to-source mapping')
    if len(set(snapshot.step_ids)) != len(snapshot.step_ids):
        raise ValueError('Duplicate mapped step IDs')
    for pose in snapshot.poses:
        p = pose.pose.position
        if pose.header.frame_id not in ('', frame) or not all(
            math.isfinite(v) for v in (p.x, p.y, p.z)
        ):
            raise ValueError('Invalid source pose')
    if any(i >= len(snapshot.poses) for i in snapshot.source_pose_indices):
        raise ValueError('Source pose index outside snapshot')
    mapping = dict(zip(snapshot.step_ids, snapshot.source_pose_indices))
    if trajectory is not None:
        if (snapshot.trajectory_id != trajectory.trajectory_id
                or snapshot.header != trajectory.header
                or set(mapping) != {s.step_id for s in trajectory.steps}):
            raise ValueError('Path snapshot does not match trajectory cycle')
        for step in trajectory.steps:
            p = snapshot.poses[mapping[step.step_id]].pose.position
            if not (math.isclose(p.x, step.x, abs_tol=1e-9, rel_tol=0.)
                    and math.isclose(p.y, step.y, abs_tol=1e-9, rel_tol=0.)):
                raise ValueError('Evaluated XY differs from its source pose')
    return mapping
