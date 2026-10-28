"""Live source calibration, carried inside the existing MAVLink connection."""

import struct
import time
import uuid


PAYLOAD_TYPE = 32768
SESSION = struct.Struct('<4sB3x16sQQQ')


class LiveTime:
    """Map recording timestamps to this host's continuously running boot clock."""

    def __init__(self, record_ns, *, monotonic_ns=time.monotonic_ns, wall_ns=time.time_ns):
        self.record_ns = record_ns
        self.monotonic_ns = monotonic_ns
        self.boot_ns = monotonic_ns()
        self.unix_ns = wall_ns()
        self.session_id = uuid.uuid4()
        self.sequence = 0
        if self.boot_ns // 1_000_000 >= 0xFFFFFFFF:
            raise ValueError('Host uptime exceeds MAVLink 32-bit boot time; reboot before live replay')

    def boot(self, record_ns):
        return self.boot_ns + record_ns - self.record_ns

    def packet(self):
        packet = SESSION.pack(b'LRST', 1, self.session_id.bytes, self.sequence,
                              self.record_ns, self.boot_ns)
        self.sequence += 1
        return packet
