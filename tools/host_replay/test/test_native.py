"""Standard FCU wire contract, independent scheduling, and no private packets."""

import json
import errno
import time
from types import SimpleNamespace

from pymavlink.dialects.v20 import common as ml
import pytest

from host_replay.native import NativeFcu
from host_replay.native_worker import NativeProcess
from host_replay.records import Recording
from .test_runner import database, endpoint


def receive(sink):
    parser, messages = ml.MAVLink(None), []
    sink.settimeout(.03)
    while True:
        try:
            messages.extend(parser.parse_buffer(sink.recv(65535)) or [])
        except TimeoutError:
            return messages


@pytest.fixture
def fcu(tmp_path):
    sink, reserve = endpoint(), endpoint()
    port = reserve.getsockname()[1]
    reserve.close()
    path = tmp_path/'mission.json'
    path.write_text(json.dumps(dict(version=1, home_msl=[34., -118., 500.],
        waypoints=[dict(frame=3, command=16, lat=34.001, lon=-118., alt=5.)])))
    options = dict(mavlink_db_path=str(database(tmp_path/'wire.db')),
                   mavlink_bind_address='127.0.0.1', mavlink_bind_port=port,
                   mavlink_destination='127.0.0.1', mavlink_destination_port=sink.getsockname()[1],
                   mission_path=str(path), system_id=7, component_id=3)
    now = [10_000_000_000]
    server = NativeFcu(Recording(options['mavlink_db_path']), options, monotonic_ns=lambda: now[0])
    class Writer:
        def write(self, packet):
            sink.sendto(packet, ('127.0.0.1', port))
    client = ml.MAVLink(Writer(), srcSystem=255, srcComponent=190)
    try:
        yield server, client, sink, now, options
    finally:
        server.close()
        sink.close()


def test_standard_mission_retries_home_and_timesync_without_tunnel(fcu):
    server, client, sink, now, _ = fcu
    server.reanchor(1_000_000_000, now[0])
    server.service()
    initial = receive(sink)
    assert not any(m.get_type() == 'TUNNEL' for m in initial)
    assert {m.get_type() for m in initial} >= {
        'HEARTBEAT', 'SYSTEM_TIME', 'MISSION_CURRENT', 'GPS_RAW_INT', 'ATTITUDE', 'HIGHRES_IMU'}
    client.mission_request_list_send(7, 3)
    client.timesync_send(0, 123456)
    client.command_long_send(7, 3, 410, 0, 0., 0., 0., 0., 0., 0., 0.)
    client.command_long_send(7, 3, 512, 0, 2., 0., 0., 0., 0., 0., 0.)
    server.service()
    messages = receive(sink)
    assert next(m for m in messages if m.get_type() == 'MISSION_COUNT').count == 1
    sync = next(m for m in messages if m.get_type() == 'TIMESYNC')
    assert (sync.tc1, sync.ts1) == (server.boot(), 123456)
    assert next(m for m in messages if m.get_type() == 'HOME_POSITION').altitude == 500000
    assert any(m.get_type() == 'SYSTEM_TIME' for m in messages)
    for integer in (True, True, False):
        (client.mission_request_int_send if integer else client.mission_request_send)(7, 3, 0)
        server.service()
        item = receive(sink)[0]
        assert item.get_type() == ('MISSION_ITEM_INT' if integer else 'MISSION_ITEM')
        assert item.command == 16 and item.frame == (6 if integer else 3)
        assert item.x == pytest.approx(340010000 if integer else 34.001, abs=1e-5)
        assert item.target_system == 255 and item.target_component == 190
    client.mission_request_list_send(8, 3)
    server.service()
    assert not receive(sink)


def test_causal_telemetry_faults_and_download_snapshot_are_independent(fcu):
    server, client, sink, now, _ = fcu
    server.reanchor(1_000_000_000, now[0])
    server.service()
    receive(sink)
    client.mission_request_list_send(7, 3)
    server.service()
    receive(sink)
    server.faults['mission_empty'] = True
    client.mission_request_int_send(7, 3, 0)
    server.service()
    assert receive(sink)[0].get_type() == 'MISSION_ITEM_INT'  # Complete old transaction.
    client.mission_request_list_send(7, 3)
    server.service()
    assert receive(sink)[0].count == 0
    server.faults['telemetry_enabled'] = False
    now[0] += 100_000_000
    server.service()
    assert not receive(sink)
    server.faults['telemetry_enabled'] = True
    now[0] += 10_000_000
    server.service()
    messages = receive(sink)
    sensor = [m for m in messages if m.get_type() in ('GPS_RAW_INT', 'ATTITUDE', 'HIGHRES_IMU')]
    assert sensor
    assert all(1_100_000 <= (m.time_boot_ms*1000 if m.get_type() == 'ATTITUDE'
                           else m.time_usec) <= 1_110_000 for m in sensor)


def test_worker_replies_while_camera_controller_is_idle(fcu):
    server, _, sink, _, options = fcu
    server.close()
    args = SimpleNamespace(**options)
    process = NativeProcess(args)
    class Writer:
        def write(self, packet):
            sink.sendto(packet, ('127.0.0.1', options['mavlink_bind_port']))
    client = ml.MAVLink(Writer(), srcSystem=255, srcComponent=190)
    try:
        process.send(dict(action='anchor', record_ns=1_000_000_000,
                          monotonic_ns=time.monotonic_ns()))
        # No parent service call: SDK opening/closing cannot starve these replies.
        client.timesync_send(0, 987654)
        client.mission_request_list_send(7, 3)
        time.sleep(.1)
        messages = receive(sink)
        assert any(m.get_type() == 'TIMESYNC' and m.ts1 == 987654 for m in messages)
        assert any(m.get_type() == 'MISSION_COUNT' for m in messages)
        assert not any(m.get_type() == 'TUNNEL' for m in messages)
        pid = process.process.pid
        process.send(dict(action='pause'))
        process.send(dict(action='anchor', record_ns=1_050_000_000,
                          monotonic_ns=time.monotonic_ns()))
        assert process.process.pid == pid
    finally:
        process.close()
    assert process.process.poll() == 0


def test_network_interface_outage_does_not_stop_the_fcu(fcu):
    server, _, sink, now, _ = fcu
    actual = server.socket
    def unavailable(*_args):
        raise OSError(errno.ENETUNREACH, 'Network is unreachable')
    server.socket = SimpleNamespace(sendto=unavailable, recvfrom=actual.recvfrom)
    try:
        server.service()
        assert server.network_errors > 0 and not receive(sink)
    finally:
        server.socket = actual
    now[0] += 1_000_000_000
    server.service()
    assert any(m.get_type() == 'HEARTBEAT' for m in receive(sink))


def test_reboot_never_emits_a_negative_boot_stamp_from_primed_history(fcu):
    server, _, sink, now, _ = fcu
    server.reanchor(1_000_000_000, now[0])
    now[0] += 100_000_000
    server.origin = now[0]-1_000_000
    server.service()
    messages = receive(sink)
    sensors = [m for m in messages if m.get_type() in ('GPS_RAW_INT', 'ATTITUDE', 'HIGHRES_IMU')]
    assert len(sensors) == 3
    assert all((m.time_boot_ms*1000 if m.get_type() == 'ATTITUDE' else m.time_usec) == 1000
               for m in sensors)
