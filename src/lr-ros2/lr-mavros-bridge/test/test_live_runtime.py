"""Recover from MAVROS respawn without retiring the still-running sender."""

import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
from types import SimpleNamespace

from std_msgs.msg import String
from lr_mavros_bridge.live_runtime import LiveRuntime


def test_empty_startup_descriptor_does_not_retire_active_session():
    runtime = SimpleNamespace(session=None, retired=set(), was_synced=False,
                              get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
                                  nanoseconds=100_000_000_000)))
    boundaries = []
    runtime.boundary = lambda reason: boundaries.append(reason)
    def receive(session, ready=True):
        LiveRuntime.on_state(runtime, String(data=json.dumps({
            'session_id': session, 'connected': ready, 'sync_ready': ready,
            'offset_ns': 99_000_000_000 if ready else 0, 'boot_anchor_ns': 1_000_000_000,
        })))
    receive('first')
    receive('', False)
    receive('first')
    assert runtime.session == 'first'
    assert not runtime.retired
    receive('second')
    assert runtime.retired == {'first'}
    receive('first')
    assert runtime.session == 'second'
    assert boundaries.count('new_session') == 2


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
        except FileNotFoundError:
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
