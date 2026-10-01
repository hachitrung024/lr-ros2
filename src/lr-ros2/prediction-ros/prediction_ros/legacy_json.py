"""Legacy JSON adapters for development tools; runtime inputs use typed ROS messages."""

from __future__ import annotations

import json
from typing import Any

from prediction_core.models import (
    GeometryStep, PredictionOutput, RoverState, TrackedObject, Trajectory, TrajectoryStep,
)

from .message_types import RosHeader


class JsonAdapters:
    """Parse and serialize the dev JSON-over-String contract."""

    @staticmethod
    def parse_json_string(raw: str) -> dict[str, Any]:
        payload = json.loads(raw)
        if not isinstance(payload, dict):
            raise ValueError("JSON payload must be an object")
        return payload

    @staticmethod
    def trajectory_from_payload(payload: dict[str, Any]) -> Trajectory:
        header = RosHeader.from_payload(payload)
        steps_raw = payload.get("steps")
        if not isinstance(steps_raw, list) or not steps_raw:
            raise ValueError("trajectory requires non-empty steps[]")
        steps = [
            TrajectoryStep(
                step_id=int(step["step_id"]),
                x=float(step["x"]),
                y=float(step["y"]),
                yaw=float(step["yaw"]),
            )
            for step in steps_raw
        ]
        timestamp = float(payload.get("timestamp", header.timestamp))
        frame_id = str(payload.get("frame_id", header.frame_id)).strip()
        return Trajectory(timestamp=timestamp, frame_id=frame_id, steps=steps)

    @staticmethod
    def objects_from_payload(
        payload: dict[str, Any],
    ) -> tuple[list[TrackedObject], RosHeader, float | None]:
        header = RosHeader.from_payload(payload)
        objects_raw = payload.get("objects", [])
        if not isinstance(objects_raw, list):
            raise ValueError("tracked object payload requires objects[] list")
        objects = [
            TrackedObject(
                timestamp=float(obj.get("timestamp", payload.get("timestamp", header.timestamp))),
                track_id=obj["track_id"],
                class_name=str(obj["class_name"]),
                footprint_polygon_xy=[
                    (float(point[0]), float(point[1]))
                    for point in obj["footprint_polygon_xy"]
                ],
                height_m=None if obj.get("height_m") is None else float(obj["height_m"]),
                velocity_xy=(
                    None
                    if obj.get("velocity_xy") is None
                    else (float(obj["velocity_xy"][0]), float(obj["velocity_xy"][1]))
                ),
                confidence=None if obj.get("confidence") is None else float(obj["confidence"]),
            )
            for obj in objects_raw
        ]
        source_stamp = payload.get("source_trajectory_stamp")
        return objects, header, None if source_stamp is None else float(source_stamp)

    @staticmethod
    def geometry_from_payload(
        payload: dict[str, Any],
    ) -> tuple[list[GeometryStep], RosHeader, float | None]:
        header = RosHeader.from_payload(payload)
        geometry_raw = payload.get("geometry", [])
        if not isinstance(geometry_raw, list):
            raise ValueError("geometry payload requires geometry[] list")
        geometry = [
            GeometryStep(
                timestamp=float(item.get("timestamp", payload.get("timestamp", header.timestamp))),
                step_id=int(item["step_id"]),
                plane_id=item["plane_id"],
                normal_xyz=(
                    float(item["normal_xyz"][0]),
                    float(item["normal_xyz"][1]),
                    float(item["normal_xyz"][2]),
                ),
                centroid_xyz=(
                    None
                    if item.get("centroid_xyz") is None
                    else (
                        float(item["centroid_xyz"][0]),
                        float(item["centroid_xyz"][1]),
                        float(item["centroid_xyz"][2]),
                    )
                ),
                confidence=None if item.get("confidence") is None else float(item["confidence"]),
            )
            for item in geometry_raw
        ]
        source_stamp = payload.get("source_trajectory_stamp")
        return geometry, header, None if source_stamp is None else float(source_stamp)

    @staticmethod
    def state_from_payload(payload: dict[str, Any]) -> RoverState:
        header = RosHeader.from_payload(payload)
        optional = payload.get
        acceleration_xyz = optional("acceleration_xyz")
        return RoverState(
            timestamp=float(optional("timestamp", header.timestamp)),
            x=None if optional("x") is None else float(optional("x")),
            y=None if optional("y") is None else float(optional("y")),
            yaw=None if optional("yaw") is None else float(optional("yaw")),
            roll=None if optional("roll") is None else float(optional("roll")),
            pitch=None if optional("pitch") is None else float(optional("pitch")),
            velocity_xy=(
                None
                if optional("velocity_xy") is None
                else (float(optional("velocity_xy")[0]), float(optional("velocity_xy")[1]))
            ),
            acceleration_xy=(
                None
                if optional("acceleration_xy") is None
                else (
                    float(optional("acceleration_xy")[0]),
                    float(optional("acceleration_xy")[1]),
                )
            ),
            angular_velocity_xyz=(
                None
                if optional("angular_velocity_xyz") is None
                else (
                    float(optional("angular_velocity_xyz")[0]),
                    float(optional("angular_velocity_xyz")[1]),
                    float(optional("angular_velocity_xyz")[2]),
                )
            ),
            velocity_xyz=(
                None
                if optional("velocity_xyz") is None
                else (
                    float(optional("velocity_xyz")[0]),
                    float(optional("velocity_xyz")[1]),
                    float(optional("velocity_xyz")[2]),
                )
            ),
            acceleration_xyz=(
                None
                if acceleration_xyz is None
                else (
                    float(acceleration_xyz[0]),
                    float(acceleration_xyz[1]),
                    float(acceleration_xyz[2]),
                )
            ),
            angular_acceleration_xyz=(
                None
                if optional("angular_acceleration_xyz") is None
                else (
                    float(optional("angular_acceleration_xyz")[0]),
                    float(optional("angular_acceleration_xyz")[1]),
                    float(optional("angular_acceleration_xyz")[2]),
                )
            ),
        )

    @staticmethod
    def prediction_to_payload(output: PredictionOutput, frame_id: str) -> dict[str, Any]:
        header = RosHeader(frame_id=frame_id, timestamp=output.timestamp)
        payload = header.to_payload()
        payload.update(
            {
                "timestamp": output.timestamp,
                "source_trajectory_stamp": output.source_trajectory_stamp,
                "collision_steps": [
                    {
                        "step_id": step.step_id,
                        "distance_along_route_m": step.distance_along_route_m,
                        "collision_objects": [
                            {
                                "object_id": candidate.object_id,
                                "object_class": candidate.object_class,
                                "min_distance_m": candidate.min_distance_m,
                                "confidence_or_validity": candidate.confidence_or_validity,
                            }
                            for candidate in step.collision_objects
                        ],
                    }
                    for step in output.collision_steps
                ],
                "rollover_steps": [
                    {
                        "step_id": step.step_id,
                        "predicted_roll_deg": step.predicted_roll_deg,
                        "predicted_pitch_deg": step.predicted_pitch_deg,
                        "static_stability_margin_m": step.static_stability_margin_m,
                        "normalized_static_stability_margin": (
                            step.normalized_static_stability_margin
                        ),
                        "terrain_id": step.terrain_id,
                        "confidence_or_validity": step.confidence_or_validity,
                    }
                    for step in output.rollover_steps
                ],
            }
        )
        return payload

    @staticmethod
    def prediction_to_json(output: PredictionOutput, frame_id: str) -> str:
        return json.dumps(JsonAdapters.prediction_to_payload(output, frame_id), sort_keys=True)
