"""Optional live-source and path freshness fencing for prediction processes.

Only the SourceGuard state callback runs concurrently with computation. It
invalidates tokens immediately, while cache resets run in the node's default
callback group on the next callback/timer, avoiding concurrent cache mutation.
"""

from functools import wraps
import math
import threading
import time

from nav_msgs.msg import Path
from rclpy.qos import DurabilityPolicy, QoSProfile

from .source_guard import SourceGuard


class PredictionGuard:
    def __init__(self, node, reset):
        self.node, self.reset = node, reset
        self.lock = threading.Lock()
        self.revision, self.dirty = 0, False
        self.path_stamp, self.path_seen, self.floor = None, 0., 0
        self.minimum_ns = 0
        self.was_ready = False
        self.max_source_age_sec = float(node.declare_parameter(
            'live_path_max_source_age_sec', 1.).value)
        if not math.isfinite(self.max_source_age_sec) or not 0 < self.max_source_age_sec <= 2.:
            raise ValueError('live_path_max_source_age_sec must be in (0, 2]')
        self.source = SourceGuard(node, self.invalidate)
        topic = node.declare_parameter('live_path_topic', '/lr/future_path/mavlink').value
        self.subscription = node.create_subscription(
            Path, topic, self.on_path,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.timer = node.create_timer(.1, self.poll)

    def invalidate(self, _reason=None):
        with self.lock:
            self.revision += 1
            self.dirty = True
            self.path_stamp = None
            # NET loss can change generation without changing the sender UUID
            # or its session anchor. Fence DDS messages queued before this edge.
            self.minimum_ns = self.node.get_clock().now().nanoseconds

    def on_path(self, message):
        stamp = message.header.stamp
        ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if not self.source.valid(stamp, self.source.token()) or ns <= self.floor or (
                ns < self.minimum_ns):
            return
        if len(message.poses) < 2:
            self.floor = ns
            self.invalidate()
            return
        with self.lock:
            previous = self.path_stamp
            if previous is not None and (stamp.sec, stamp.nanosec) <= (
                    previous.sec, previous.nanosec):
                return
            self.path_stamp, self.path_seen = stamp, time.monotonic()

    def available(self):
        with self.lock:
            stamp, seen = self.path_stamp, self.path_seen
        if stamp is None:
            return False
        return (time.monotonic() - seen < 1. and self.source.valid(stamp, self.source.token())
                and self.node.get_clock().now().nanoseconds - (
                    stamp.sec * 1_000_000_000 + stamp.nanosec) <= self.max_source_age_sec * 1e9)

    def poll(self):
        ready = self.available()
        if self.was_ready and not ready:
            self.invalidate()
        self.was_ready = ready
        with self.lock:
            dirty, self.dirty = self.dirty, False
        if dirty:
            self.reset(None)

    def enter(self, stamp):
        self.poll()
        token = self.token()
        return token if self.valid(stamp, token) else None

    def token(self):
        with self.lock:
            revision = self.revision
        return self.source.token(), revision

    def valid(self, stamp, token):
        ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        return (token is not None and token == self.token() and ns >= self.minimum_ns
                and self.available()
                and self.source.valid(stamp, token[0]))


def attach_prediction_guard(node, reset):
    if node.declare_parameter('source_guard_enabled', False).value:
        return PredictionGuard(node, reset)
    return None


def guarded(callback):
    """Fence an input callback; geometry must use its trajectory source stamp."""
    @wraps(callback)
    def dispatch(self, message, *args, **kwargs):
        guard = getattr(self, '_source_guard', None)
        if guard is not None:
            stamp = getattr(message, 'source_trajectory_stamp', message.header.stamp)
            token = guard.enter(stamp)
            if token is None:
                return
            self._work_token = token
        return callback(self, message, *args, **kwargs)
    return dispatch
