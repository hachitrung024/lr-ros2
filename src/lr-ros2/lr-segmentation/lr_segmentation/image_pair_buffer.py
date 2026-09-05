"""Bounded mask/depth matching that tolerates segmentation inference delay."""


def stamp_ns(message):
    """Return the source timestamp as integer nanoseconds."""
    stamp = message.header.stamp
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


class ImagePairBuffer:
    """Match nearest available timestamps, consuming each input at most once."""

    def __init__(self, tolerance_ns, max_samples=30, max_age_ns=1_000_000_000):
        """Configure matching tolerance and hard memory/history limits."""
        if tolerance_ns < 0 or max_samples < 1 or max_age_ns <= 0:
            raise ValueError('invalid synchronization buffer limits')
        self.tolerance_ns = tolerance_ns
        self.max_samples = max_samples
        self.max_age_ns = max_age_ns
        self.reset()

    def reset(self):
        """Discard samples and counters from the old timeline."""
        self.masks = {}
        self.depths = {}
        self.latest_stamp = None
        self.last_mask = -1
        self.last_depth = -1
        self.mask_count = 0
        self.pair_count = 0
        self.dropped_masks = 0

    def add(self, kind, message):
        """Retain a sample, expiring old entries in timestamp order."""
        stamp = stamp_ns(message)
        if kind == 'mask':
            self.mask_count += 1
        if stamp <= (self.last_mask if kind == 'mask' else self.last_depth):
            return
        self.latest_stamp = stamp if self.latest_stamp is None else max(stamp, self.latest_stamp)
        target = self.masks if kind == 'mask' else self.depths
        target[stamp] = message
        for pending in (self.masks, self.depths):
            expired = [s for s in pending if self.latest_stamp - s > self.max_age_ns]
            for old in expired:
                pending.pop(old)
                self.dropped_masks += int(pending is self.masks)
            while len(pending) > self.max_samples:
                pending.pop(min(pending))
                self.dropped_masks += int(pending is self.masks)

    def pop_pair(self):
        """Consume the oldest mask with a depth inside the tolerance."""
        for mask_stamp in sorted(self.masks):
            if not self.depths:
                return None
            depth_stamp = min(self.depths, key=lambda s: (abs(s - mask_stamp), s))
            if abs(depth_stamp - mask_stamp) > self.tolerance_ns:
                continue
            pair = self.masks.pop(mask_stamp), self.depths.pop(depth_stamp)
            self.last_mask, self.last_depth = mask_stamp, depth_stamp
            # Keep tracker input monotonic even if transport reorders samples.
            for pending, used in ((self.masks, mask_stamp), (self.depths, depth_stamp)):
                for old in [s for s in pending if s <= used]:
                    pending.pop(old)
                    self.dropped_masks += int(pending is self.masks)
            self.pair_count += 1
            return pair
        return None
