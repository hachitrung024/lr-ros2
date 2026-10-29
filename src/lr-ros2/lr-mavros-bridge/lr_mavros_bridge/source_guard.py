"""Fence in-flight perception work when the live source changes generation."""

import json
import threading
import time

from rclpy.callback_groups import CallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String


class SourceGuard:
    def __init__(self, node, reset, resume=None):
        self.node, self.reset = node, reset
        self.resume = resume
        self.lock = threading.Lock()
        self.generation, self.ready, self.start_ns, self.seen = None, False, 0, 0.0
        self.runtime_id = ''
        self.session_id = ''
        self.retired = set()
        self.pending_resume = False
        self.subscription = node.create_subscription(
            String, '/lr/source/generation', self.on_state,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
            callback_group=MutuallyExclusiveCallbackGroup())
        if resume is not None:
            # Replacing sensor readers must serialize with their data callbacks.
            self.resume_timer = node.create_timer(.05, self.run_resume)

    def on_state(self, message):
        try:
            data = json.loads(message.data)
            with self.lock:
                runtime = data.get('runtime_id', '')
                incoming = (runtime, data['generation'])
                if runtime in self.retired:
                    return
                if runtime == self.runtime_id and self.generation and incoming < self.generation:
                    return
                if runtime != self.runtime_id:
                    if self.runtime_id:
                        self.retired.add(self.runtime_id)
                    self.runtime_id = runtime
                changed = incoming != self.generation
                resumed = bool(data['ready'] and (changed or not self.ready))
                self.generation = incoming
                self.session_id = data.get('session_id', '')
                self.ready, self.start_ns = data['ready'], int(data['start_ns'])
                self.seen = time.monotonic()
            if changed:
                self.reset(None)
            if resumed and self.resume is not None:
                with self.lock:
                    self.pending_resume = True
        except (KeyError, ValueError, TypeError):
            return

    def run_resume(self):
        with self.lock:
            run = self.pending_resume and self.ready
            self.pending_resume = False
        if run:
            self.resume()

    def token(self):
        with self.lock:
            return self.generation

    def valid(self, stamp, token):
        ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        with self.lock:
            return (self.ready and token == self.generation and ns >= self.start_ns
                    and time.monotonic() - self.seen < 2.0
                    and -500_000_000 <= self.node.get_clock().now().nanoseconds - ns <= 2_000_000_000)


def attach_guard(node, reset):
    if node.declare_parameter('source_guard_enabled', False).value:
        return SourceGuard(node, reset)
    return None


class _RetiredCallbackGroup(CallbackGroup):
    """Keep already-queued executor work from taking a destroyed reader."""

    def can_execute(self, _entity):
        return False

    def beginning_execution(self, _entity):
        return False

    def ending_execution(self, _entity):
        pass


def retire_subscription(node, subscription):
    """Retire a sensor reader from its serialized resume callback."""
    subscription.callback_group = _RetiredCallbackGroup()
    node.destroy_subscription(subscription)
