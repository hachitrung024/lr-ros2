"""Bounded timestamp selection for asynchronous prediction inputs."""

from collections import deque


def stamp_ns(stamp):
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class InputHistory:
    """Select the latest observation at/before a cycle, including empty batches."""

    def __init__(self, max_age_sec, max_samples=128):
        self.max_age_ns = None if max_age_sec is None else round(max_age_sec * 1e9)
        self.messages = deque(maxlen=max_samples)

    def clear(self):
        self.messages.clear()

    def append(self, message):
        self.messages.append(message)

    def select(self, trajectory):
        end = stamp_ns(trajectory.header.stamp)
        candidates = [
            message
            for message in self.messages
            if message.header.frame_id == trajectory.header.frame_id
            and stamp_ns(message.header.stamp) <= end
        ]
        if not candidates:
            return None, 'missing or future input'
        selected = max(candidates, key=lambda m: stamp_ns(m.header.stamp))
        if self.max_age_ns is not None and end - stamp_ns(selected.header.stamp) > self.max_age_ns:
            return None, 'stale input'
        return selected, 'ready'
