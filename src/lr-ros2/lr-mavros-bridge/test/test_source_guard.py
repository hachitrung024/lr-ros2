"""Generation fencing invalidates work which was already running at reset."""

import json
import time
from types import SimpleNamespace

from std_msgs.msg import String
from lr_mavros_bridge.source_guard import SourceGuard


def test_old_inflight_work_and_stale_source_are_rejected():
    now = time.time_ns()
    node = SimpleNamespace(create_subscription=lambda *_a, **_k: None,
                           get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
                               nanoseconds=now)))
    resets = []
    guard = SourceGuard(node, lambda _: resets.append(1))
    stamp = SimpleNamespace(sec=now // 1_000_000_000, nanosec=now % 1_000_000_000)
    def send(generation, ready, start):
        guard.on_state(String(data=json.dumps({'generation': generation, 'ready': ready,
                                               'start_ns': start})))
    send(1, True, now - 1)
    work = guard.token()
    assert guard.valid(stamp, work)
    send(2, True, now + 1)
    assert not guard.valid(stamp, work)
    assert not guard.valid(stamp, guard.token())
    send(2, False, now)
    assert not guard.valid(stamp, guard.token())
    send(2, True, now)
    guard.seen -= 3
    assert not guard.valid(stamp, guard.token())
    assert len(resets) == 2


def test_resume_rejoins_readers_once_per_readiness_transition():
    node = SimpleNamespace(create_subscription=lambda *_a, **_k: None,
                           create_timer=lambda *_a: None)
    resumed = []
    guard = SourceGuard(node, lambda _: None, lambda: resumed.append(1))
    def send(generation, ready):
        guard.on_state(String(data=json.dumps({'generation': generation, 'ready': ready,
                                               'start_ns': 1})))
        guard.run_resume()
    send(1, False)
    send(1, True)
    send(1, True)
    assert len(resumed) == 1
    send(2, True)
    assert len(resumed) == 2
    send(1, False)  # Late generation cannot undo current readiness.
    assert guard.ready
    send(2, False)
    send(2, True)
    assert len(resumed) == 3


def test_retired_subscription_rejects_already_queued_executor_work():
    from lr_mavros_bridge.source_guard import retire_subscription
    destroyed = []
    node = SimpleNamespace(destroy_subscription=destroyed.append)
    reader = SimpleNamespace(callback_group=None)
    retire_subscription(node, reader)
    assert destroyed == [reader]
    assert not reader.callback_group.can_execute(reader)
    assert not reader.callback_group.beginning_execution(reader)


def test_camera_guard_accepts_unknown_clock_but_fences_old_frames():
    node = SimpleNamespace(create_subscription=lambda *_a, **_k: None,
                           get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
                               nanoseconds=time.time_ns())))
    guard = SourceGuard(node, lambda _: None, topic='/lr/source/camera_generation')
    guard.on_state(String(data=json.dumps({'generation': 1, 'ready': True,
        'start_ns': 1000000000, 'latest_stamp_ns': 4000000000, 'stamp_is_system': False})))
    assert guard.valid(SimpleNamespace(sec=4, nanosec=0), guard.token())
    assert not guard.valid(SimpleNamespace(sec=1, nanosec=0), guard.token())
    old_token = guard.token()
    guard.on_state(String(data=json.dumps({'generation': 2, 'ready': True,
        'start_ns': 1000000000, 'latest_stamp_ns': 4000000000, 'stamp_is_system': False})))
    assert not guard.valid(SimpleNamespace(sec=4, nanosec=0), old_token)
