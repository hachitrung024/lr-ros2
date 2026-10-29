"""UDP MAVLink transport and explicit recording coverage validation."""

from pathlib import Path
from collections import deque
import socket
import sqlite3
import time
from urllib.parse import quote

from .live import PAYLOAD_TYPE
from .path_protocol import PAYLOAD_TYPE as PATH_PAYLOAD_TYPE
from .records import Record, encode


def validate_recording(path):
    """Check valid GPS/attitude coverage using the baseline's read-only schema."""
    database = Path(path).expanduser().resolve(strict=True)
    required = {
        'gps': {'id', 't_wall_epoch_us', 'lat', 'lon', 'alt', 'fix_type'},
        'attitude': {'id', 't_wall_epoch_us', 'roll', 'pitch', 'yaw'},
        'imu': {'id', 't_wall_epoch_us'},
    }
    with sqlite3.connect(f'file:{quote(str(database))}?mode=ro&immutable=1', uri=True) as db:
        for table, columns in required.items():
            actual = {row[1] for row in db.execute(f'PRAGMA table_info("{table}")')}
            if columns - actual:
                raise ValueError(f'{table}: missing columns {sorted(columns - actual)}')
        gps = db.execute(
            'SELECT MIN(t_wall_epoch_us), MAX(t_wall_epoch_us) FROM gps '
            'WHERE lat IS NOT NULL AND lon IS NOT NULL AND alt IS NOT NULL '
            'AND lat != 0 AND lon != 0 AND fix_type >= 3'
        ).fetchone()
        attitude = db.execute(
            'SELECT MIN(t_wall_epoch_us), MAX(t_wall_epoch_us) FROM attitude '
            'WHERE roll IS NOT NULL AND pitch IS NOT NULL AND yaw IS NOT NULL'
        ).fetchone()
    if None in (*gps, *attitude):
        raise ValueError('Recording has no valid GPS/attitude coverage')
    return min(gps[0], attitude[0]) * 1000, max(gps[1], attitude[1]) * 1000


class TelemetrySender:
    """Retain the baseline replay epoch, units and supported discovery replies."""

    def __init__(self, recording, bind_address, bind_port, destination, port, *, live=None,
                 future_path=None):
        from pymavlink.dialects.v20 import common as mavlink
        self.recording = recording
        self.live = live
        if future_path is not None and live is None:
            raise ValueError('Future path transport requires live calibration')
        self.future_path = future_path
        self.path_pending = deque()
        self.path_packets_sent = 0
        self.next_session = 0.0
        self.protocol = mavlink
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.bind((bind_address, bind_port))
            self.socket.setblocking(False)
            self.destination = (destination, port)
            self.mav = mavlink.MAVLink(self, srcSystem=1, srcComponent=1)
            self.parser = mavlink.MAVLink(None)
            self.parser.robust_parsing = True
            self.sent = 0
            self.next_heartbeat = 0.0
            self.service()
        except BaseException:
            self.socket.close()
            raise

    def write(self, packet):
        return self.socket.sendto(packet, self.destination)

    def service(self):
        """Run heartbeat and bounded reply handling on the same monotonic loop."""
        now = time.monotonic()
        mavlink = self.protocol
        if self.live is not None and now >= self.next_session:
            packet = self.live.packet()
            self.mav.tunnel_send(0, 0, PAYLOAD_TYPE, len(packet), list(packet.ljust(128, b'\0')))
            self.next_session = now + 1.0
        if now >= self.next_heartbeat:
            self.mav.heartbeat_send(
                mavlink.MAV_TYPE_GROUND_ROVER, mavlink.MAV_AUTOPILOT_GENERIC,
                0, 0, mavlink.MAV_STATE_ACTIVE,
            )
            self.next_heartbeat = now + 1.0
        for _ in range(64):
            try:
                packet, peer = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            if peer != self.destination:
                continue
            for request in self.parser.parse_buffer(packet) or []:
                if self.live is not None and request.get_type() == 'TIMESYNC':
                    if request.tc1 == 0:
                        self.mav.timesync_send(self.live.monotonic_ns(), request.ts1)
                    continue
                if request.get_type() != 'COMMAND_LONG':
                    continue
                supported = (
                    request.command == mavlink.MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES
                    or (request.command == mavlink.MAV_CMD_REQUEST_MESSAGE
                        and int(request.param1) == 148)
                )
                if supported:
                    self.mav.autopilot_version_send(
                        mavlink.MAV_PROTOCOL_CAPABILITY_MAVLINK2,
                        0, 0, 0, 0, [0] * 8, [0] * 8, [0] * 8, 0, 0, 0,
                    )
                self.mav.command_ack_send(
                    request.command,
                    mavlink.MAV_RESULT_ACCEPTED if supported else mavlink.MAV_RESULT_UNSUPPORTED,
                    target_system=request.get_srcSystem(),
                    target_component=request.get_srcComponent(),
                )
        # Bounded work per tick: path fragments cannot starve TIMESYNC or heartbeat.
        for _ in range(min(4, len(self.path_pending))):
            packet = self.path_pending.popleft()
            self.mav.tunnel_send(0, 0, PATH_PAYLOAD_TYPE, len(packet),
                                 list(packet.ljust(128, b'\0')))
            self.path_packets_sent += 1

    def send_frame(self, stamp_ns):
        """Emit SYSTEM_TIME and only the bounded past/current sensor batch."""
        boot_ns = (self.live.boot(stamp_ns) if self.live is not None
                   else stamp_ns - self.recording.epoch_us * 1000)
        boot_ms = boot_ns // 1_000_000
        if boot_ms < 0:
            return
        if boot_ms > 0xFFFFFFFF:
            raise ValueError('Telemetry exceeds the replay boot-time range')
        unix_us = (self.live.unix_ns + stamp_ns - self.live.record_ns) // 1000 if (
            self.live is not None) else self.recording.epoch_us + boot_ms * 1000
        self.mav.system_time_send(unix_us, boot_ms)
        for record in self.recording.advance(stamp_ns):
            if self.live is not None:
                # Fresh samples only: an old session must never prime the new live estimator.
                if record.stamp_us * 1000 < self.live.record_ns:
                    continue
                record = Record(record.kind, self.live.boot(record.stamp_us * 1000) // 1000,
                                record.values)
            message = encode(record, 0 if self.live is not None else self.recording.epoch_us,
                             self.mav)
            if message is not None:
                self.mav.send(message)
                self.sent += 1
        if self.future_path is not None:
            packets = self.future_path.prepare(stamp_ns, self.live)
            if packets is not None:
                # Replace unsent work instead of accumulating stale snapshots.
                self.path_pending = deque(packets)

    def close(self):
        self.socket.close()
