"""Bounded retention of measured footprints for presentation only."""

from copy import deepcopy
import math

from visualization_msgs.msg import Marker, MarkerArray

from .box_outputs import _marker_array


class BoxDisplayBuffer:
    """Keep last measured map positions briefly across missing detections."""

    def __init__(self, hold_sec):
        """Configure a ROS-time hold; zero disables retention across observations."""
        if not math.isfinite(hold_sec) or hold_sec < 0:
            raise ValueError('visualization.hold_sec must be finite and non-negative')
        self.hold_ns = round(hold_sec * 1e9)
        self.reset()

    def reset(self):
        """Drop all presentation history on a new timeline."""
        self._entries = {}
        self._current_ids = set()
        self._header = None
        self._signature = None

    def update(self, boxes, header, now_ns):
        """Record only measured boxes, keeping their original source headers."""
        if self._header is not None and self._header.frame_id != header.frame_id:
            self.reset()
        self._header = deepcopy(header)
        self._current_ids = {box.track_id for box in boxes}
        for box in boxes:
            self._entries[box.track_id] = (deepcopy(box), deepcopy(header), now_ns)
        if self.hold_ns == 0:
            self._entries = {k: v for k, v in self._entries.items() if k in self._current_ids}

    def markers(self, now_ns, **style):
        """Return a changed full snapshot, expiring held boxes even without new input."""
        if self._header is None:
            return None
        self._entries = {
            k: v for k, v in self._entries.items() if now_ns - v[2] <= self.hold_ns
        }
        signature = tuple((k, v[1].stamp.sec, v[1].stamp.nanosec, v[2], k in self._current_ids)
                          for k, v in self._entries.items())
        if signature == self._signature:
            return None
        self._signature = signature
        result = MarkerArray(markers=[Marker(header=self._header, action=Marker.DELETEALL)])
        for identity, (box, header, _) in self._entries.items():
            markers = _marker_array(header, [box], **style).markers[1:]
            if identity not in self._current_ids:
                for marker in markers:
                    marker.color.a *= .45
                    if marker.type == Marker.TEXT_VIEW_FACING:
                        marker.text += ' (last seen)'
            result.markers.extend(markers)
        return result
