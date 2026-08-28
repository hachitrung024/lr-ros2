"""Test helpers for prediction_ros coordinator tests."""

from __future__ import annotations

from prediction_ros.adapters import JsonAdapters
from prediction_ros.cache import PredictionInputCache


def trajectory_payload(
    stamp: float = 1000.0,
    step_ids: list[int] | None = None,
    *,
    trajectory_id: int | None = None,
) -> dict:
    step_ids = step_ids or [0, 1, 2]
    payload = {
        "header": {"stamp": {"sec": int(stamp), "nanosec": 0}, "frame_id": "map"},
        "timestamp": stamp,
        "frame_id": "map",
        "steps": [
            {"step_id": step_id, "x": float(step_id), "y": 0.0, "yaw": 0.0}
            for step_id in step_ids
        ],
    }
    if trajectory_id is not None:
        payload["trajectory_id"] = trajectory_id
    return payload


def objects_payload(
    stamp: float = 1000.0,
    source_trajectory_stamp: float | None = 1000.0,
    objects: list | None = None,
) -> dict:
    payload = {
        "header": {"stamp": {"sec": int(stamp), "nanosec": 0}, "frame_id": "map"},
        "timestamp": stamp,
        "frame_id": "map",
        "objects": objects if objects is not None else [],
    }
    if source_trajectory_stamp is not None:
        payload["source_trajectory_stamp"] = source_trajectory_stamp
    return payload


def geometry_payload(
    stamp: float = 1000.0,
    source_trajectory_stamp: float | None = 1000.0,
    step_ids: list[int] | None = None,
    *,
    source_trajectory_id: int | None = None,
) -> dict:
    step_ids = step_ids or [0, 1, 2]
    payload = {
        "header": {"stamp": {"sec": int(stamp), "nanosec": 0}, "frame_id": "map"},
        "timestamp": stamp,
        "frame_id": "map",
        "geometry": [
            {
                "step_id": step_id,
                "plane_id": f"plane-{step_id}",
                "normal_xyz": [0.0, 0.0, 1.0],
            }
            for step_id in step_ids
        ],
    }
    if source_trajectory_stamp is not None:
        payload["source_trajectory_stamp"] = source_trajectory_stamp
    if source_trajectory_id is not None:
        payload["source_trajectory_id"] = source_trajectory_id
    return payload


def load_trajectory(payload: dict):
    return JsonAdapters.trajectory_from_payload(payload)


def load_objects(payload: dict):
    objects, header, source_stamp = JsonAdapters.objects_from_payload(payload)
    return objects, header.frame_id, source_stamp


def load_geometry(payload: dict):
    geometry, header, source_stamp = JsonAdapters.geometry_from_payload(payload)
    return geometry, header.frame_id, source_stamp


def geometry_source_trajectory_id(payload: dict, *, fallback: int | None = None) -> int | None:
    source_id = payload.get("source_trajectory_id")
    if source_id is not None:
        return int(source_id)
    source_stamp = payload.get("source_trajectory_stamp")
    if source_stamp is not None:
        return int(source_stamp)
    return fallback


def cache_ready_cycle(
    cache: PredictionInputCache,
    *,
    trajectory_stamp: float = 1000.0,
    trajectory_id: int | None = None,
    geometry_stamp: float | None = 1000.0,
    object_stamp: float | None = 1000.0,
    geometry_step_ids: list[int] | None = None,
) -> None:
    cycle_id = trajectory_id if trajectory_id is not None else int(trajectory_stamp)
    trajectory = load_trajectory(trajectory_payload(trajectory_stamp, trajectory_id=cycle_id))
    cache.set_trajectory(trajectory, trajectory_id=cycle_id)
    objects, objects_frame, _ = load_objects(
        objects_payload(object_stamp if object_stamp is not None else trajectory_stamp)
    )
    cache.set_objects(objects, frame_id=objects_frame)
    geometry, geometry_frame, geometry_source = load_geometry(
        geometry_payload(
            geometry_stamp if geometry_stamp is not None else trajectory_stamp,
            source_trajectory_stamp=geometry_stamp if geometry_stamp is not None else trajectory_stamp,
            step_ids=geometry_step_ids,
            source_trajectory_id=cycle_id,
        )
    )
    cache.set_geometry(
        geometry,
        frame_id=geometry_frame,
        source_trajectory_id=cycle_id,
        source_trajectory_stamp=geometry_source,
    )
