"""Check causal pose evidence independently, and measure error against offline ground truth."""

import argparse
import json
from pathlib import Path
import numpy as np
from lr_future_path.mavlink import load_trajectory


def rotate(q, w, dt):
    angle = np.linalg.norm(w) * dt
    if angle < 1e-12:
        return q
    v = w / np.linalg.norm(w) * np.sin(angle / 2)
    s = np.cos(angle / 2)
    xyz = q[3] * v + s * q[:3] + np.cross(q[:3], v)
    out = np.r_[xyz, q[3] * s - np.dot(q[:3], v)]
    return out / np.linalg.norm(out)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--audit", required=True)
    p.add_argument("--db", required=True)
    p.add_argument("--output", default="/tmp/lr-mavros-comparison.json")
    a = p.parse_args()
    trajectory = load_trajectory(a.db)
    exact_position = []
    exact_orientation = []
    gt_position = []
    gt_angle = []
    future = 0
    for line in Path(a.audit).read_text().splitlines():
        r = json.loads(line)
        t = r["stamp_ns"]
        g0, g1 = r["gps_stamps"]
        at = r["attitude_stamp"]
        future += int(max(g0, g1, at) > t)
        p0, p1 = np.array(r["gps_positions"])
        q = np.array(r["attitude"])
        w = np.array(r["angular_velocity"])
        expected = p1 + (p1 - p0) * (t - g1) / (g1 - g0)
        expected_q = rotate(q, w, (t - at) / 1e9)
        actual = np.array(r["position"])
        actual_q = np.array(r["orientation"])
        exact_position.append(float(np.linalg.norm(actual - expected)))
        exact_orientation.append(
            float(
                min(np.linalg.norm(actual_q - expected_q), np.linalg.norm(actual_q + expected_q))
            )
        )
        gt = trajectory.pose_at(t, max_gps_gap_ns=1500000000)
        if gt:
            gt_position.append(float(np.linalg.norm(actual - gt.position)))
            gt_angle.append(
                float(
                    np.degrees(
                        2 * np.arccos(np.clip(abs(np.dot(actual_q, gt.orientation_xyzw)), 0, 1))
                    )
                )
            )

    def stats(values):
        return {
            "samples": len(values),
            "median": float(np.median(values)),
            "p95": float(np.percentile(values, 95)),
            "max": float(max(values)),
        }

    passed = (
        bool(exact_position)
        and future == 0
        and max(exact_position) < 1e-8
        and max(exact_orientation) < 1e-8
    )
    report = {
        "passed": passed,
        "extrinsics": "identity (acceptance run)",
        "future_sample_violations": future,
        "same_received_samples_position_error_m": stats(exact_position),
        "same_received_samples_quaternion_error": stats(exact_orientation),
        "offline_ground_truth_position_error_m": stats(gt_position),
        "offline_ground_truth_orientation_error_deg": stats(gt_angle),
    }
    Path(a.output).write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if not passed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
