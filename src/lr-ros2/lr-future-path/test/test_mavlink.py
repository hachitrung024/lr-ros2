"""Tests for MAVLink database discovery and trajectory conversion."""

import math
from pathlib import Path
import sqlite3

import numpy as np
import pytest

from lr_future_path.mavlink import (
    MavlinkDataError,
    MavlinkTrajectory,
    PoseSample,
    apply_body_to_camera,
    discover_sessions,
    geodetic_to_enu,
    inspect_session,
    mavlink_attitude_to_ros_quaternion,
    select_session,
    slerp,
)
from lr_future_path.trajectory import rotate_vector


SECOND = 1_000_000_000


def _create_database(path: Path, *, start_us=1_000_000) -> None:
    connection = sqlite3.connect(path)
    connection.executescript(
        """
        CREATE TABLE gps (
            t_wall_epoch_us INTEGER NOT NULL,
            lat REAL,
            lon REAL,
            alt REAL,
            fix_type INTEGER
        );
        CREATE TABLE attitude (
            t_wall_epoch_us INTEGER NOT NULL,
            roll REAL,
            pitch REAL,
            yaw REAL
        );
        """
    )
    connection.executemany(
        "INSERT INTO gps VALUES (?, ?, ?, ?, ?)",
        [
            (start_us, 34.0, -118.0, 100.0, 4),
            (start_us + 500_000, 0.0, 0.0, 0.0, 0),
            (start_us + 1_000_000, 34.00001, -118.0, 101.0, 3),
        ],
    )
    connection.executemany(
        "INSERT INTO attitude VALUES (?, ?, ?, ?)",
        [
            (start_us - 100_000, 0.0, 0.0, 0.0),
            (start_us + 1_100_000, 0.0, 0.0, math.pi / 2),
        ],
    )
    connection.commit()
    connection.close()


def _identity_trajectory(gps_stamps, gps_positions):
    return MavlinkTrajectory(
        path=Path("test.db"),
        gps_stamps_ns=np.asarray(gps_stamps, dtype=np.int64),
        gps_positions=np.asarray(gps_positions, dtype=np.float64),
        attitude_stamps_ns=np.asarray(
            [gps_stamps[0], gps_stamps[-1]], dtype=np.int64
        ),
        attitude_orientations_xyzw=np.tile(
            np.array([0.0, 0.0, 0.0, 1.0]), (2, 1)
        ),
    )


def test_session_inspection_discovery_and_immutable_read(tmp_path):
    """Discovery validates nested DBs without creating SQLite sidecars."""
    session = tmp_path / "nested" / "session_20260101_1200_mavlink"
    session.mkdir(parents=True)
    database = session / "session_mavlink.db"
    _create_database(database)

    summary = inspect_session(database)
    discovered = discover_sessions(tmp_path)

    assert summary.path == database.resolve()
    assert discovered == [summary]
    assert summary.start_ns == 900_000 * 1000
    assert summary.gps_start_ns == 1_000_000 * 1000
    assert not database.with_name(database.name + "-wal").exists()
    assert not database.with_name(database.name + "-shm").exists()


def test_session_selection_supports_auto_and_explicit_paths(tmp_path):
    """The SVO clock selects the unique session near its start time."""
    first = tmp_path / "first" / "session_mavlink.db"
    second = tmp_path / "second" / "session_mavlink.db"
    first.parent.mkdir()
    second.parent.mkdir()
    _create_database(first, start_us=1_000_000)
    _create_database(second, start_us=10_000_000)

    selected = select_session(
        1_050_000_000,
        directory=tmp_path,
        tolerance_ns=500_000_000,
    )
    explicit = select_session(
        10_050_000_000,
        explicit_path=second,
        tolerance_ns=500_000_000,
    )

    assert selected.path == first.resolve()
    assert explicit.path == second.resolve()


def test_session_selection_rejects_no_match_and_ambiguity(tmp_path):
    """Missing and overlapping candidate sessions fail clearly."""
    for name in ("one", "two"):
        database = tmp_path / name / "session_mavlink.db"
        database.parent.mkdir()
        _create_database(database, start_us=1_000_000)

    with pytest.raises(MavlinkDataError, match="No MAVLink session"):
        select_session(
            100 * SECOND,
            directory=tmp_path,
            tolerance_ns=SECOND,
        )
    with pytest.raises(MavlinkDataError, match="Multiple MAVLink sessions"):
        select_session(
            SECOND,
            directory=tmp_path,
            tolerance_ns=SECOND,
        )


def test_session_inspection_rejects_rosbag_or_wrong_schema(tmp_path):
    """A generic SQLite database is not accepted as MAVLink telemetry."""
    database = tmp_path / "session_mavlink.db"
    connection = sqlite3.connect(database)
    connection.execute("CREATE TABLE messages (timestamp INTEGER)")
    connection.close()

    with pytest.raises(MavlinkDataError, match="missing columns"):
        inspect_session(database)


def test_geodetic_to_enu_uses_east_north_up_metres():
    """Small longitude, latitude and altitude offsets map to ENU axes."""
    reference = np.array([0.0, 0.0, 10.0])
    values = np.array(
        [
            reference,
            [0.0, 0.00001, 10.0],
            [0.00001, 0.0, 10.0],
            [0.0, 0.0, 11.0],
        ]
    )

    enu = geodetic_to_enu(values, reference)

    assert enu[0] == pytest.approx([0.0, 0.0, 0.0], abs=1e-8)
    assert enu[1] == pytest.approx([1.1132, 0.0, 0.0], abs=1e-3)
    assert enu[2] == pytest.approx([0.0, 1.1057, 0.0], abs=1e-3)
    assert enu[3] == pytest.approx([0.0, 0.0, 1.0], abs=1e-6)


def test_mavlink_attitude_converts_ned_frd_to_enu_flu():
    """North and east MAV headings become ROS ENU yaw axes."""
    north = mavlink_attitude_to_ros_quaternion(0.0, 0.0, 0.0)
    east = mavlink_attitude_to_ros_quaternion(0.0, 0.0, math.pi / 2)

    assert rotate_vector(north, np.array([1.0, 0.0, 0.0])) == pytest.approx(
        [0.0, 1.0, 0.0], abs=1e-7
    )
    assert rotate_vector(east, np.array([1.0, 0.0, 0.0])) == pytest.approx(
        [1.0, 0.0, 0.0], abs=1e-7
    )


def test_slerp_uses_short_arc_across_yaw_wrap():
    """Quaternion interpolation does not rotate through zero at ±180°."""
    left = np.array([0.0, 0.0, math.sin(math.radians(179) / 2),
                     math.cos(math.radians(179) / 2)])
    right = np.array([0.0, 0.0, math.sin(math.radians(-179) / 2),
                      math.cos(math.radians(-179) / 2)])

    middle = slerp(left, right, 0.5)

    assert rotate_vector(middle, np.array([1.0, 0.0, 0.0])) == pytest.approx(
        [-1.0, 0.0, 0.0], abs=1e-6
    )


def test_pose_interpolation_rejects_large_gps_gap_and_supports_seek():
    """Pose lookup is stateless and refuses to bridge a missing-GPS gap."""
    trajectory = _identity_trajectory(
        [0, SECOND, 10 * SECOND],
        [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [10.0, 0.0, 0.0]],
    )

    forward = trajectory.pose_at(
        SECOND // 2, max_gps_gap_ns=2 * SECOND
    )
    missing = trajectory.pose_at(
        5 * SECOND, max_gps_gap_ns=2 * SECOND
    )
    backward = trajectory.pose_at(
        SECOND // 4, max_gps_gap_ns=2 * SECOND
    )

    assert forward.position == pytest.approx([0.5, 0.0, 0.0])
    assert missing is None
    assert backward.position == pytest.approx([0.25, 0.0, 0.0])


def test_body_to_camera_applies_translation_and_rotation():
    """Mounting extrinsics are composed in the MAV body frame."""
    body = PoseSample(
        stamp_ns=1,
        position=np.array([10.0, 20.0, 0.0]),
        orientation_xyzw=np.array(
            [0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)]
        ),
    )

    camera = apply_body_to_camera(
        body, np.array([1.0, 0.0, 0.0, 0.0, 0.0, math.pi / 2])
    )

    assert camera.position == pytest.approx([10.0, 21.0, 0.0])
    assert rotate_vector(
        camera.orientation_xyzw, np.array([1.0, 0.0, 0.0])
    ) == pytest.approx([-1.0, 0.0, 0.0], abs=1e-7)


def test_future_samples_start_now_and_stop_at_gap_radius_and_limit():
    """Future look-ahead sampling follows the existing Path contract."""
    trajectory = _identity_trajectory(
        [0, SECOND, 2 * SECOND, 3 * SECOND, 10 * SECOND],
        [
            [0.0, 0.0, 0.0],
            [0.2, 0.0, 0.0],
            [0.5, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [1.2, 0.0, 0.0],
        ],
    )

    samples = trajectory.future_samples(
        SECOND // 2,
        radius_m=2.0,
        step_m=0.3,
        max_gps_gap_ns=2 * SECOND,
        max_points=3,
    )

    assert [sample.stamp_ns for sample in samples] == [
        SECOND // 2,
        2 * SECOND,
        3 * SECOND,
    ]
    assert samples[0].position == pytest.approx([0.1, 0.0, 0.0])
    assert samples[-1].position == pytest.approx([1.0, 0.0, 0.0])
