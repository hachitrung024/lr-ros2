"""Late work and stale paths must not revive cached prediction evidence."""

import json
import time
from types import SimpleNamespace

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String

from lr_mavros_bridge.prediction_guard import PredictionGuard


def test_generation_inflight_fence_path_timeout_and_recovery():
    now = time.time_ns()
    clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=now))
    node = SimpleNamespace(create_subscription=lambda *_a, **_k: None,
                           create_timer=lambda *_a: None,
                           declare_parameter=lambda _name, default: SimpleNamespace(value=default),
                           get_clock=lambda: clock)
    resets = []
    guard = PredictionGuard(node, lambda _: resets.append(1))
    def source(generation, start):
        guard.source.on_state(String(data=json.dumps({
            'runtime_id': 'r', 'generation': generation, 'ready': True, 'start_ns': start})))
    def path(stamp):
        msg = Path(poses=[PoseStamped(), PoseStamped()])
        msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        guard.on_path(msg)
        return msg.header.stamp
    source(1, now - 1)
    stamp = path(now)
    work = guard.enter(stamp)
    assert work is not None and guard.valid(stamp, work)
    # Invalidation is immediate; cache mutation waits for the default callback group.
    source(2, now + 1)
    assert not guard.valid(stamp, work)
    assert len(resets) == 1
    now += 2
    stamp = path(now)
    work = guard.enter(stamp)
    assert guard.valid(stamp, work)
    assert len(resets) == 2
    guard.path_seen -= 1.1
    assert not guard.valid(stamp, work)
    guard.poll()
    assert len(resets) == 3
    now += 1
    stamp = path(now)
    assert guard.enter(stamp) is not None
    empty = Path()
    now += 1
    empty.header.stamp.sec, empty.header.stamp.nanosec = divmod(now, 1_000_000_000)
    guard.on_path(empty)
    assert not guard.valid(stamp, work)
    assert guard.enter(path(now - 1)) is None


def test_same_session_generation_drops_preboundary_dds_samples():
    now = [time.time_ns()]
    node = SimpleNamespace(create_subscription=lambda *_a, **_k: None,
                           create_timer=lambda *_a: None,
                           declare_parameter=lambda _name, default: SimpleNamespace(value=default),
                           get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
                               nanoseconds=now[0])))
    guard = PredictionGuard(node, lambda _: None)
    def generation(value):
        guard.source.on_state(String(data=json.dumps({
            'runtime_id': 'r', 'generation': value, 'ready': True, 'start_ns': 1})))
    def path(stamp):
        message = Path(poses=[PoseStamped(), PoseStamped()])
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
        guard.on_path(message)
        return message.header.stamp
    generation(1)
    old_stamp = path(now[0])
    assert guard.enter(old_stamp) is not None
    now[0] += 100_000_000
    generation(2)  # Same original anchor, e.g. NET receiver restart.
    path(now[0] - 1)
    assert not guard.available()
    fresh = path(now[0])
    assert guard.enter(fresh) is not None
    assert guard.enter(old_stamp) is None


def test_native_processing_latency_keeps_causal_inputs_until_source_deadline():
    now = [time.time_ns()]
    node = SimpleNamespace(create_subscription=lambda *_a, **_k: None,
                           create_timer=lambda *_a: None,
                           declare_parameter=lambda name, default: SimpleNamespace(
                               value=2. if name == 'live_path_max_source_age_sec' else default),
                           get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(
                               nanoseconds=now[0])))
    resets = []
    guard = PredictionGuard(node, lambda _: resets.append(1))
    guard.source.on_state(String(data=json.dumps({
        'runtime_id': 'r', 'generation': 1, 'ready': True, 'start_ns': 1})))
    message = Path(poses=[PoseStamped(), PoseStamped()])
    message.header.stamp.sec, message.header.stamp.nanosec = divmod(now[0], 1_000_000_000)
    guard.on_path(message)
    guard.poll()
    resets.clear()
    # The freshly evaluated route is based on an IMAGE pose. Processing and
    # transport take >1 s, but remain inside the existing 2 s fusion deadline.
    now[0] += 1_200_000_000
    guard.on_path(message)  # Equal stamps do not renew its receipt watchdog.
    guard.poll()
    assert guard.available() and not resets
    guard.path_seen -= 1.1
    guard.poll()
    assert not guard.available() and resets
