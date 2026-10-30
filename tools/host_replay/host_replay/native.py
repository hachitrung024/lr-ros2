"""Standard MAVLink FCU: independent clock, causal sensors and mission download.

No private TUNNEL, ROS, SDK, recording path or control channel crosses to Jetson.
SYSTEM_TIME relates simulated UTC (the SVO clock) to FCU boot time.
"""

import socket
import errno
import time

from .native_mission import Mission
from .records import Record, encode


class NativeFcu:
    def __init__(self, recording, options, *, monotonic_ns=time.monotonic_ns):
        from pymavlink.dialects.v20 import common as ml
        self.ml, self.recording, self.options = ml, recording, options
        self.now = monotonic_ns
        self.origin = self.now()-1_000_000_000
        self.anchor = None
        self.paused = True
        self.mission = Mission(recording, options.get('mission_path'),
                               options.get('mission_from_recording', False))
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            self.socket.bind((options['mavlink_bind_address'], options['mavlink_bind_port']))
            self.socket.setblocking(False)
        except BaseException:
            self.socket.close()
            raise
        self.peer = (socket.gethostbyname(options['mavlink_destination']),
                     options['mavlink_destination_port'])
        self.mav = ml.MAVLink(self, srcSystem=options.get('system_id', 1),
                              srcComponent=options.get('component_id', 1))
        self.parser = ml.MAVLink(None)
        self.parser.robust_parsing = True
        self.due = dict.fromkeys(('heartbeat', 'system', 'progress'), 0)
        self.counts = {}
        self.emitted_types = {}
        self.network_errors = 0
        self.faults = {}
        self.snapshot = None
        self.gps = None
        self.seq, self.complete = 0, False

    def write(self, packet):
        try:
            self.socket.sendto(packet, self.peer)
        except OSError as error:
            if error.errno not in (errno.ENETUNREACH, errno.EHOSTUNREACH, errno.EADDRNOTAVAIL,
                                   errno.ENOBUFS, errno.EAGAIN):
                raise
            self.network_errors += 1
            return 0  # Keep the FCU and camera independent during an interface outage.
        msgid = int.from_bytes(packet[7:10], 'little') if packet[0] == 253 else packet[5]
        name = self.ml.mavlink_map[msgid].msgname
        self.emitted_types[name] = self.emitted_types.get(name, 0)+1
        self.counts['packets'] = self.counts.get('packets', 0)+1
        return len(packet)

    def reanchor(self, record_ns, mono_ns):
        self.origin = self.now()-1_000_000_000  # Seek models a standard FCU reboot.
        self.anchor = (int(record_ns), int(mono_ns))
        self.paused = False
        self.recording.reset()
        self.mission.seq = 0
        self.snapshot = None
        self.gps = None

    def clock(self):
        if self.anchor is None:
            return 0
        return self.anchor[0]+self.now()-self.anchor[1]

    def boot(self):
        return self.now()-self.origin

    def addressed(self, request):
        return (getattr(request, 'target_system', self.mav.srcSystem) in (0, self.mav.srcSystem)
                and getattr(request, 'target_component', self.mav.srcComponent) in (
                    0, self.mav.srcComponent))

    def home(self):
        if self.mission.home is None or not self.faults.get('home_enabled', True):
            return False
        lat, lon, alt = self.mission.home
        self.mav.home_position_send(round(lat*1e7), round(lon*1e7), round(alt*1000),
                                   0., 0., 0., [1., 0., 0., 0.], 0., 0., 0., self.boot()//1000)
        return True

    def handle(self, request):
        ml, kind = self.ml, request.get_type()
        self.counts[kind+'_received'] = self.counts.get(kind+'_received', 0)+1
        if not self.addressed(request):
            return
        if kind == 'TIMESYNC' and request.tc1 == 0:
            if self.faults.get('timesync_enabled', True):
                self.mav.timesync_send(self.boot(), request.ts1)
        elif kind == 'MISSION_REQUEST_LIST' and self.faults.get('mission_enabled', True):
            if request.mission_type != 0:
                self.mav.mission_ack_send(request.get_srcSystem(), request.get_srcComponent(),
                                         ml.MAV_MISSION_UNSUPPORTED, request.mission_type)
                return
            self.snapshot = [] if self.faults.get('mission_empty') else list(self.mission.items)
            self.mav.mission_count_send(request.get_srcSystem(), request.get_srcComponent(),
                                        len(self.snapshot), 0)
        elif kind in ('MISSION_REQUEST_INT', 'MISSION_REQUEST') and (
                self.faults.get('mission_enabled', True) and request.mission_type == 0 and
                self.snapshot is not None and request.seq < len(self.snapshot)):
            item = self.snapshot[request.seq]
            frame = item.get('frame', 0)
            integer = kind.endswith('_INT')
            frame = {0: 5, 3: 6}.get(frame, frame) if integer else {5: 0, 6: 3}.get(frame, frame)
            params = (request.get_srcSystem(), request.get_srcComponent(), request.seq, frame,
                      item.get('command', 16), int(request.seq == self.seq), 1,
                      *[float(item.get('param'+str(i), 0.)) for i in range(1, 5)])
            if integer:
                self.mav.mission_item_int_send(*params, round(item['lat']*1e7),
                                               round(item['lon']*1e7), item['alt'], 0)
            else:
                self.mav.mission_item_send(*params, item['lat'], item['lon'], item['alt'], 0)
        elif kind == 'COMMAND_LONG':
            supported = (request.command == 520 or (
                request.command == 512 and int(request.param1) == 148))
            if supported:
                self.mav.autopilot_version_send(
                    ml.MAV_PROTOCOL_CAPABILITY_MAVLINK2 | ml.MAV_PROTOCOL_CAPABILITY_MISSION_INT,
                    0, 0, 0, 0, [0]*8, [0]*8, [0]*8, 0, 0, 0)
            elif request.command == 512 and int(request.param1) == 2:
                supported = self.faults.get('system_time_enabled', True)
                if supported:
                    self.mav.system_time_send(self.clock()//1000, self.boot()//1_000_000)
            elif request.command == 410 or (request.command == 512 and int(request.param1) == 242):
                supported = self.home()
            self.mav.command_ack_send(request.command, ml.MAV_RESULT_ACCEPTED if supported
                                      else ml.MAV_RESULT_UNSUPPORTED,
                                      target_system=request.get_srcSystem(),
                                      target_component=request.get_srcComponent())
        elif kind in ('MISSION_COUNT', 'MISSION_CLEAR_ALL', 'MISSION_WRITE_PARTIAL_LIST'):
            self.mav.mission_ack_send(request.get_srcSystem(), request.get_srcComponent(),
                                     ml.MAV_MISSION_DENIED, getattr(request, 'mission_type', 0))

    def service(self):
        now, record_ns = self.now(), self.clock()
        enabled = self.faults.get('telemetry_enabled', True)
        if record_ns and not self.paused:
            batch = self.recording.advance(record_ns)
            if enabled:
                for record in batch:
                    if record.stamp_us*1000 < self.anchor[0]:
                        continue
                    if record.kind == 'gps':
                        self.gps = (record.values['lat'], record.values['lon'])
                    boot_us = (self.anchor[1]-self.origin+record.stamp_us*1000-self.anchor[0])//1000
                    if boot_us < 0:
                        continue  # Priming must not emit observations predating this FCU boot.
                    message = encode(Record(record.kind, boot_us, record.values), 0, self.mav)
                    if message is not None:
                        self.mav.send(message)
                        self.counts[message.get_type()] = self.counts.get(message.get_type(), 0)+1
            self.seq, self.complete = self.mission.progress(record_ns, self.gps)
        if enabled:
            if now >= self.due['heartbeat']:
                self.mav.heartbeat_send(self.ml.MAV_TYPE_GROUND_ROVER,
                    self.ml.MAV_AUTOPILOT_GENERIC, 0, 0, self.ml.MAV_STATE_ACTIVE)
                self.due['heartbeat'] = now+500_000_000
            if now >= self.due['system'] and self.faults.get('system_time_enabled', True):
                # The camera uses this explicitly shared UTC clock, not wall arrival.
                self.mav.system_time_send(record_ns//1000, self.boot()//1_000_000)
                self.due['system'] = now+200_000_000
            if now >= self.due['progress'] and self.faults.get('progress_enabled', True):
                empty = self.faults.get('mission_empty') or not self.mission.items
                state = 1 if empty else 5 if self.complete else 3
                self.mav.mission_current_send(self.seq, 65535 if empty else len(self.mission.items),
                                              state, 2 if self.paused else 1)
                self.due['progress'] = now+500_000_000
        for _ in range(64):
            try:
                packet, peer = self.socket.recvfrom(65535)
            except BlockingIOError:
                break
            except OSError as error:
                if error.errno not in (errno.ENETUNREACH, errno.EHOSTUNREACH):
                    raise
                self.network_errors += 1
                break
            if peer == self.peer:
                for request in self.parser.parse_buffer(packet) or []:
                    if enabled:
                        self.handle(request)

    def close(self):
        self.socket.close()
