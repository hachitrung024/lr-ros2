"""Independent input lifecycle, native clock gating and replay recovery."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest
import rclpy
from std_msgs.msg import Header, String
from lr_mavros_bridge.live_runtime import LiveRuntime


@pytest.fixture
def runtime():
    rclpy.init()
    node = LiveRuntime()
    node.launch_camera = lambda: setattr(node, 'child', SimpleNamespace(poll=lambda: None))
    node.stop_camera = lambda: setattr(node, 'stopping', True)
    yield node
    node.child = None
    node.destroy_node()
    rclpy.shutdown()


def frame(node, stamp=None):
    header = Header()
    stamp = stamp or time.time_ns()
    header.stamp.sec, header.stamp.nanosec = divmod(stamp, 1_000_000_000)
    node.on_raw(header)


def state(node, ready=True, session='', epoch=1):
    node.on_state(String(data=json.dumps({
        'session_id': session, 'connected': ready, 'sync_ready': ready,
        'offset_ns': 0, 'boot_anchor_ns': time.time_ns(),
        'record_anchor_ns': 1000000000, 'telemetry_epoch': epoch})))


def test_camera_starts_without_mavlink_or_verified_clock(runtime):
    runtime.poll()
    assert runtime.child is not None
    frame(runtime, 123000000000)
    runtime.poll()
    assert runtime.camera_ready and not runtime.ready
    assert runtime.status == 'camera_time_unverified'


def test_native_fusion_requires_clock_and_loss_keeps_camera(runtime):
    runtime.poll()
    frame(runtime)
    state(runtime)
    runtime.resume_at = 0.
    runtime.telemetry = dict.fromkeys(('gps', 'attitude'), (time.time_ns(), time.monotonic()))
    runtime.poll()
    assert not runtime.ready
    runtime.mapping = {'stamp_is_system': True, 'stamp_offset_ns': 0, 'record_anchor_ns': 0}
    runtime.poll()
    assert runtime.ready
    camera_generation, child = runtime.camera_generation, runtime.child
    state(runtime, ready=False)
    runtime.poll()
    assert runtime.camera_ready and not runtime.ready
    assert runtime.child is child and not runtime.stopping
    assert runtime.camera_generation == camera_generation


def test_replay_seek_restarts_only_receiver_and_preserves_session_during_loss(runtime):
    runtime.profile = 'host_replay'
    runtime.poll()
    state(runtime, session='first')
    assert runtime.stopping
    runtime.stopping = False
    state(runtime, ready=False)
    assert runtime.session == 'first' and not runtime.retired
    state(runtime, session='first')
    assert not runtime.stopping
    state(runtime, session='second')
    assert runtime.stopping and runtime.retired == {'first'}
    state(runtime, session='first')
    assert runtime.session == 'second'


def test_camera_initialization_timeout_differs_from_stalled_stream(runtime):
    runtime.poll()
    runtime.started = runtime.last_raw = time.monotonic() - 6.
    runtime.poll()
    assert not runtime.stopping and not runtime.camera_ready
    frame(runtime)
    runtime.poll()
    assert runtime.camera_ready
    runtime.last_raw -= 6.
    runtime.poll()
    assert runtime.stopping and not runtime.camera_ready


def test_clock_jump_retires_inflight_camera_and_fusion(runtime):
    runtime.poll()
    frame(runtime)
    runtime.poll()
    before = runtime.camera_generation, runtime.generation
    frame(runtime, time.time_ns()-10_000_000_000)
    assert runtime.camera_generation > before[0] and runtime.generation > before[1]


def test_camera_child_exits_when_supervisor_is_killed():
    code = '''
import os,subprocess,sys,time
child=subprocess.Popen([sys.executable,'-m','lr_mavros_bridge.camera_child',
                        str(os.getpid()),'/bin/sleep','60'])
print(child.pid,flush=True)
time.sleep(60)
'''
    parent = subprocess.Popen([sys.executable, '-c', code], stdout=subprocess.PIPE, text=True)
    child = int(parent.stdout.readline())
    def running():
        try:
            return Path(f'/proc/{child}/stat').read_text().split()[2] != 'Z'
        except (FileNotFoundError, ProcessLookupError):
            return False
    try:
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            if os.readlink(f'/proc/{child}/exe').endswith('/sleep'):
                break
            time.sleep(.02)
        else:
            raise AssertionError('camera helper did not execute its child')
        parent.kill()
        parent.wait(timeout=3)
        deadline = time.monotonic() + 3
        while running() and time.monotonic() < deadline:
            time.sleep(.02)
        assert not running()
    finally:
        if parent.poll() is None:
            parent.kill()
            parent.wait(timeout=3)
        if running():
            os.kill(child, signal.SIGKILL)


def test_stream_liveness_does_not_reset_map_at_frame_pose_age_limit(runtime):
    runtime.poll()
    frame(runtime)
    state(runtime)
    runtime.resume_at = 0.
    runtime.mapping = {'stamp_is_system': True, 'stamp_offset_ns': 0, 'record_anchor_ns': 0}
    runtime.telemetry = dict.fromkeys(('gps', 'attitude'),
                                      (time.time_ns()-300000000, time.monotonic()-.3))
    runtime.poll()
    assert runtime.ready
    generation = runtime.generation
    runtime.poll()
    assert runtime.generation == generation
    runtime.telemetry['attitude'] = (time.time_ns()-2100000000, time.monotonic()-2.1)
    runtime.poll()
    assert not runtime.ready and runtime.camera_ready
    assert runtime.generation > generation


def test_shared_fcu_utc_requires_explicit_anchor_and_survives_mavlink_loss(runtime):
    runtime.clock_mode = 'fcu_utc'
    runtime.mapping = runtime.native_mapping()
    runtime.poll()
    frame(runtime, time.time_ns()-30_000_000_000)
    state(runtime)
    runtime.poll()
    assert runtime.camera_ready and not runtime.ready
    # Reported UTC->boot plus measured TIMESYNC boot->ROS; zero/arrival-based
    # guesses are never substituted for a missing SYSTEM_TIME relation.
    runtime.on_state(String(data=json.dumps(dict(
        connected=True, sync_ready=True, offset_ns=10_000_000_000,
        utc_ready=True, utc_age_sec=.1, utc_minus_boot_ns=-20_000_000_000,
        session_id='', telemetry_epoch=1))))
    assert runtime.mapping['stamp_offset_ns'] == 30_000_000_000
    assert not runtime.stopping  # Initial calibration does not restart SDK initialization.
    runtime.stopping = False
    frame(runtime, time.time_ns()-30_000_000_000)
    runtime.telemetry = dict.fromkeys(('gps', 'attitude'), (time.time_ns(), time.monotonic()))
    runtime.resume_at = 0.
    runtime.poll()
    assert runtime.ready
    camera_generation = runtime.camera_generation
    state(runtime, ready=False)
    runtime.poll()
    assert runtime.camera_ready and not runtime.ready
    assert runtime.camera_generation == camera_generation
    assert runtime.mapping['stamp_offset_ns'] == 30_000_000_000
