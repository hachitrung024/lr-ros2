"""Wire-level timestamp and causal cursor tests."""

import math
import sqlite3

import pytest
from pymavlink.dialects.v20 import common as mavlink
from host_replay.records import Recording, Record, encode


def test_cursor_priming_pause_and_rewind(tmp_path):
    path = tmp_path / "telemetry.db"
    with sqlite3.connect(path) as db:
        for table in ("gps", "imu", "attitude"):
            db.execute(f"CREATE TABLE {table} (id INTEGER, t_wall_epoch_us INTEGER)")
            db.executemany(
                f"INSERT INTO {table} VALUES (?,?)", [(i, i * 100000) for i in range(1, 101)]
            )
    data = Recording(path)
    first = data.advance(5_000_000_000)
    assert len(first) == 6
    assert all(r.stamp_us <= 5_000_000 for r in first)
    assert data.advance(5_000_000_000) == []
    assert len(data.advance(5_200_000_000)) == 6
    assert len(data.advance(2_000_000_000)) == 6
    assert all(r.stamp_us <= 2_000_000 for r in data.advance(2_000_000_000))


@pytest.mark.parametrize("kind", ["gps", "attitude", "imu"])
def test_wire_epoch_round_trip(kind):
    epoch = 1783700660000000
    stamp = epoch + 123456789
    values = dict(
        lat=34.2,
        lon=-118.3,
        alt=506.45,
        fix_type=4,
        satellites_visible=32,
        roll=0.1,
        pitch=0.2,
        yaw=0.3,
        roll_rate=0.01,
        pitch_rate=0.02,
        yaw_rate=0.03,
        acc_x=0.0,
        acc_y=0.0,
        acc_z=-1000.0,
        gyro_x=100.0,
        gyro_y=0.0,
        gyro_z=0.0,
        temperature=2500.0,
    )
    m = mavlink.MAVLink(None, srcSystem=1, srcComponent=1)
    message = encode(Record(kind, stamp, values), epoch, m)
    wire = message.pack(m)
    assert wire[0] == 0xFD
    decoded = mavlink.MAVLink(None).parse_char(wire)
    if kind == "attitude":
        assert 0 <= stamp - (epoch + decoded.time_boot_ms * 1000) < 1000
    else:
        assert epoch + decoded.time_usec == stamp
    if kind == "imu":
        assert decoded.zacc == pytest.approx(-9.80665)
        assert decoded.xgyro == pytest.approx(0.1)
        assert not decoded.fields_updated & (7 << 6)
        assert math.isnan(decoded.xmag)
    if kind == "gps":
        assert decoded.alt == 506450
        assert decoded.cog == 65535


def test_missing_attitude_does_not_invent_orientation():
    assert encode(Record("attitude", 10, {}), 0, mavlink.MAVLink(None)) is None
