"""Read decoded telemetry and reconstruct timestamped MAVLink 2 messages."""

from bisect import bisect_right
from dataclasses import dataclass
import math
from pathlib import Path
import sqlite3
from urllib.parse import quote


@dataclass(frozen=True)
class Record:
    kind: str
    stamp_us: int
    values: dict


class Recording:
    """A read-only recording, with bounded past-only priming on startup/reset."""

    def __init__(self, path):
        path = Path(path).resolve(strict=True)
        self.records = []
        with sqlite3.connect(f"file:{quote(str(path))}?mode=ro", uri=True) as db:
            db.row_factory = sqlite3.Row
            for table in ("gps", "imu", "attitude"):
                previous = -1
                for row in db.execute(f"SELECT * FROM {table} ORDER BY t_wall_epoch_us, id"):
                    values = dict(row)
                    stamp = int(values["t_wall_epoch_us"])
                    if stamp <= previous:
                        continue
                    previous = stamp
                    self.records.append(Record(table, stamp, values))
        if not self.records:
            raise ValueError("Recording has no telemetry")
        # RAW_IMU-derived acceleration reaches MAVROS before attitude at equal stamps.
        self.records.sort(key=lambda r: (r.stamp_us, r.kind == "attitude", r.kind))
        self.stamps = [r.stamp_us for r in self.records]
        self.epoch_us = self.stamps[0] // 1_000_000 * 1_000_000
        self.reset()

    def reset(self):
        """Prime at the next image without replaying samples skipped by a seek."""
        self.cursor = None
        self.last_clock_us = None

    def advance(self, clock_ns):
        now = clock_ns // 1000
        end = bisect_right(self.stamps, now)
        if self.last_clock_us == now:
            return []
        reset = self.cursor is None or now < self.last_clock_us
        if reset:
            past = {}
            # Only retain two recent records per stream; never emit future samples.
            for record in reversed(self.records[:end]):
                if now - record.stamp_us > 1_500_000:
                    break
                bucket = past.setdefault(record.kind, [])
                if len(bucket) < 2:
                    bucket.append(record)
            output = sorted(
                (r for rows in past.values() for r in rows),
                key=lambda r: (r.stamp_us, r.kind == "attitude", r.kind),
            )
        else:
            output = self.records[self.cursor : end]
        self.cursor, self.last_clock_us = end, now
        return output


def finite(value):
    return value is not None and math.isfinite(float(value))


def scaled(value, factor, unknown, low, high):
    if not finite(value):
        return unknown
    number = round(float(value) * factor)
    return number if low <= number <= high else unknown


def encode(record, epoch_us, mav):
    """Preserve measured fields; return None when a required attitude is missing."""
    v = record.values
    boot_us = record.stamp_us - epoch_us
    if boot_us < 0 or boot_us // 1000 > 0xFFFFFFFF:
        raise ValueError("Telemetry exceeds the replay boot-time range")
    if record.kind == "gps":
        valid = all(finite(v.get(k)) for k in ("lat", "lon", "alt"))
        return mav.gps_raw_int_encode(
            boot_us,
            int(v.get("fix_type") or 0) if valid else 0,
            scaled(v.get("lat"), 1e7, 0, -900000000, 900000000),
            scaled(v.get("lon"), 1e7, 0, -1800000000, 1800000000),
            scaled(v.get("alt"), 1000, 0, -2147483648, 2147483647),
            scaled(
                v.get("h_dop") if v.get("h_dop") is not None else v.get("eph"),
                100,
                65535,
                0,
                65534,
            ),
            scaled(
                v.get("v_dop") if v.get("v_dop") is not None else v.get("epv"),
                100,
                65535,
                0,
                65534,
            ),
            scaled(v.get("vel"), 100, 65535, 0, 65534),
            65535,
            scaled(v.get("satellites_visible"), 1, 255, 0, 254),
        )
    if record.kind == "attitude":
        if not all(finite(v.get(k)) for k in ("roll", "pitch", "yaw")):
            return None
        return mav.attitude_encode(
            boot_us // 1000,
            *[
                float(v[k]) if finite(v.get(k)) else math.nan
                for k in ("roll", "pitch", "yaw", "roll_rate", "pitch_rate", "yaw_rate")
            ],
        )
    # The recorder stores ArduPilot RAW_IMU units: milli-g, mrad/s, centidegrees C.
    fields = 0
    values = []
    for i, (name, factor) in enumerate(
        [
            ("acc_x", 9.80665 / 1000),
            ("acc_y", 9.80665 / 1000),
            ("acc_z", 9.80665 / 1000),
            ("gyro_x", 0.001),
            ("gyro_y", 0.001),
            ("gyro_z", 0.001),
            ("mag_x", 0.001),
            ("mag_y", 0.001),
            ("mag_z", 0.001),
            ("abs_pressure", 1),
            ("diff_pressure", 1),
            ("pressure_alt", 1),
            ("temperature", 0.01),
        ]
    ):
        # This DB does not identify a valid magnetometer; zero-filled fields are not evidence.
        valid = finite(v.get(name)) and i not in (6, 7, 8, 9, 10, 11)
        values.append(float(v[name]) * factor if valid else math.nan)
        if valid:
            fields |= 1 << i
    return mav.highres_imu_encode(boot_us, *values, fields)
