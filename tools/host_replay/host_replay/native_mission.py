"""Explicit read-only mission fixtures for the standard MAVLink FCU emulator."""

from bisect import bisect_right
import json
import math
from pathlib import Path


def distance(a, b):
    lat = math.radians((a[0]+b[0])/2)
    return math.hypot(math.radians(a[0]-b[0])*6371000,
                      math.radians(a[1]-b[1])*6371000*math.cos(lat))


class Mission:
    def __init__(self, recording, filename=None, from_recording=False, spacing=5.):
        self.items, self.times = [], []
        gps = [r for r in recording.records if r.kind == 'gps' and
               (r.values.get('fix_type') or 0) >= 3 and all(
                   r.values.get(k) is not None and math.isfinite(r.values[k])
                   for k in ('lat', 'lon', 'alt'))]
        self.home = ([gps[0].values[k] for k in ('lat', 'lon', 'alt')] if gps else None)
        if filename:
            data = json.loads(Path(filename).expanduser().read_text())
            if data.get('version') != 1 or not isinstance(data.get('waypoints'), list):
                raise ValueError('Mission requires version=1 and a waypoints list')
            self.items = data['waypoints']
            self.home = data.get('home_msl', self.home)
        elif from_recording:
            for record in gps:
                v = record.values
                if not self.items or distance((v['lat'], v['lon']), (
                        self.items[-1]['lat'], self.items[-1]['lon'])) >= spacing:
                    self.items.append(dict(frame=0, command=16, lat=v['lat'],
                                           lon=v['lon'], alt=v['alt']))
                    self.times.append(record.stamp_us*1000)
        if len(self.items) > 65534:
            raise ValueError('Mission exceeds MAVLink item count')
        for item in self.items:
            if item.get('frame', 0) not in (0, 3, 5, 6) or item.get('command', 16) != 16:
                raise ValueError('Native fixture supports global NAV_WAYPOINT only')
            if not all(math.isfinite(item[k]) for k in ('lat', 'lon', 'alt')) or not (
                    -90 <= item['lat'] <= 90 and -180 <= item['lon'] <= 180):
                raise ValueError('Invalid mission coordinates')
            if not all(math.isfinite(item.get('param'+str(i), 0.)) for i in range(1, 5)):
                raise ValueError('Invalid waypoint parameter')
        if self.home is not None and (len(self.home) != 3 or not all(
                math.isfinite(v) for v in self.home)):
            raise ValueError('home_msl must be [latitude, longitude, altitude_msl]')
        self.seq = 0

    def progress(self, record_ns, gps=None):
        if self.times:
            self.seq = bisect_right(self.times, record_ns)
        elif gps is not None:
            while self.seq < len(self.items) and distance(gps, (
                    self.items[self.seq]['lat'], self.items[self.seq]['lon'])) <= 2.:
                self.seq += 1
        return min(self.seq, max(0, len(self.items)-1)), self.seq >= len(self.items)
