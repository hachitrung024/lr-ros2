"""Geometry of a supported MAVLink waypoint mission, without invented ETAs."""

import math
import numpy as np

from lr_future_path.mavlink import PoseSample, apply_body_to_camera, geodetic_to_enu
from lr_future_path.path_processing import prepare_path
from lr_future_path.trajectory import quaternion_multiply, rotate_vector


def mission_path(waypoints, current_seq, origin, home_msl, current, mount,
                 radius=15., step=.2):
    if not 0 <= current_seq < len(waypoints):
        raise ValueError('mission_empty_or_finished')
    coordinates = []
    for item in waypoints[current_seq:]:
        if item.command != 16:
            raise ValueError(f'unsupported_mission_command:{item.command}')
        if item.frame not in (0, 3, 5, 6):
            raise ValueError(f'unsupported_mission_frame:{item.frame}')
        lat, lon, alt = item.x_lat, item.y_long, item.z_alt
        if not all(math.isfinite(v) for v in (lat, lon, alt)) or not (
                -90 <= lat <= 90 and -180 <= lon <= 180):
            raise ValueError('invalid_waypoint_coordinates')
        if item.frame in (3, 6):
            if home_msl is None:
                raise ValueError('waiting_home')
            alt += home_msl
        coordinates.append([lat, lon, alt])
    xyz = geodetic_to_enu(np.asarray(coordinates), np.asarray(origin))
    # Pose is camera-referenced. Recover body position before forming mission
    # tangents, then apply the same body-to-camera transform as the pose node.
    q_mount = apply_body_to_camera(PoseSample(
        0, np.zeros(3), np.array([0., 0., 0., 1.])), mount).orientation_xyzw
    q_body = quaternion_multiply(current.orientation_xyzw, q_mount * [-1., -1., -1., 1.])
    body = current.position - rotate_vector(q_body, np.asarray(mount[:3]))
    points, quaternions = [current.position], [current.orientation_xyzw]
    previous = body
    for point in xyz:
        delta = point - previous
        if np.linalg.norm(delta[:2]) < 1e-6:
            continue
        yaw = math.atan2(delta[1], delta[0])
        pose = apply_body_to_camera(PoseSample(
            current.stamp_ns, point, np.array([0., 0., math.sin(yaw/2), math.cos(yaw/2)])), mount)
        points.append(pose.position)
        quaternions.append(pose.orientation_xyzw)
        previous = point
    if len(points) < 2:
        raise ValueError('mission_has_no_forward_segment')
    stamps, positions, _ = prepare_path(
        [current.stamp_ns] * len(points), points, quaternions,
        radius_m=radius, step_m=step, max_points=128,
        smoothing_window_m=0., max_deviation_m=0.)
    # A mission is geometric look-ahead. Interpolating current attitude toward
    # a distant waypoint would retain an unrelated heading throughout the
    # local horizon. Use each sampled segment's tangent, then the camera mount.
    tangents = np.diff(positions[:, :2], axis=0)
    if not len(tangents):
        raise ValueError('mission_has_no_forward_segment')
    tangents = np.vstack((tangents, tangents[-1]))
    headings = np.arctan2(tangents[:, 1], tangents[:, 0])
    orientations = np.asarray([quaternion_multiply(
        np.array([0., 0., math.sin(yaw/2), math.cos(yaw/2)]), q_mount)
        for yaw in headings])
    return stamps, positions, orientations
