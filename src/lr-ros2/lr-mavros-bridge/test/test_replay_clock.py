"""Exercise UDP loss, malformed packets, session restarts and delayed packets."""

import json
import uuid

import pytest

from lr_mavros_bridge.replay_clock import ReplayClock


def packet(session, sequence, stamp, **values):
    return json.dumps(dict(version=1, session_id=session, sequence=sequence,
                           stamp_ns=stamp, **values)).encode()


def test_loss_reorder_duplicate_and_session_rewind():
    state = ReplayClock()
    old, new = str(uuid.uuid4()), str(uuid.uuid4())
    start = 1783700660123456789
    assert state.accept(packet(old, 100, start)) == start
    assert state.accept(packet(old, 103, start + 3)) == start + 3
    assert state.accept(packet(old, 102, start + 2)) is None
    assert state.accept(packet(old, 103, start + 3)) is None
    assert state.accept(packet(old, 104, start - 1)) is None
    assert state.accept(packet(new, 0, start - 100)) == start - 100
    assert state.accept(packet(old, 200, start + 100)) is None
    assert state.stamp_ns == start - 100
    assert state.accepted == 3


@pytest.mark.parametrize('payload', [
    b'bad json', b'\xff', b'[]', b'null', b'{}', b' ' * 4097,
    b'{"version":true}', b'{"version":2}',
    json.dumps(dict(version=1, session_id='bad', sequence=0, stamp_ns=1)).encode(),
    json.dumps(dict(version=1, session_id=str(uuid.uuid4()), sequence=True, stamp_ns=1)).encode(),
    json.dumps(dict(version=1, session_id=str(uuid.uuid4()), sequence=0, stamp_ns=1.5)).encode(),
    json.dumps(dict(version=1, session_id=str(uuid.uuid4()), sequence=0, stamp_ns=0)).encode(),
    json.dumps(dict(version=1, session_id=str(uuid.uuid4()), sequence=-1, stamp_ns=1)).encode(),
    json.dumps(dict(version=1, session_id=str(uuid.uuid4()), sequence=0, stamp_ns=2**63)).encode(),
])
def test_bad_packet_does_not_change_clock_or_crash(payload):
    state = ReplayClock()
    assert state.accept(payload) is None
    assert state.session_id is None and state.stamp_ns == 0
    assert state.rejected == 1
