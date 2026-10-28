"""Validate standalone replay clock packets without depending on ROS."""

import json
import uuid


class ReplayClock:
    """Reject stale packets while permitting time rewind only in a new session."""

    def __init__(self):
        self.session_id = None
        self.sequence = -1
        self.stamp_ns = 0
        self.retired_sessions = set()
        self.accepted = 0
        self.rejected = 0

    def accept(self, packet):
        """Return an exact timestamp, or None without changing accepted state."""
        try:
            if len(packet) > 4096:
                raise ValueError('oversized packet')
            value = json.loads(packet)
            if not isinstance(value, dict) or type(value.get('version')) is not int:
                raise ValueError('invalid object/version')
            if value['version'] != 1:
                raise ValueError('unsupported version')
            session = value['session_id']
            if not isinstance(session, str) or str(uuid.UUID(session)) != session:
                raise ValueError('invalid session UUID')
            sequence, stamp = value['sequence'], value['stamp_ns']
            if type(sequence) is not int or not 0 <= sequence < 2**63:
                raise ValueError('invalid sequence')
            if type(stamp) is not int or not 0 < stamp < 2**31 * 1_000_000_000:
                raise ValueError('timestamp outside ROS Time range')
            if session in self.retired_sessions:
                raise ValueError('retired session')
            if session == self.session_id:
                if sequence <= self.sequence or stamp <= self.stamp_ns:
                    raise ValueError('duplicate or out-of-order frame')
        except (ValueError, KeyError, TypeError, UnicodeError, AttributeError, RecursionError):
            self.rejected += 1
            return None
        if self.session_id is not None and session != self.session_id:
            self.retired_sessions.add(self.session_id)
        self.session_id, self.sequence, self.stamp_ns = session, sequence, stamp
        self.accepted += 1
        return stamp
