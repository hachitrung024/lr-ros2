"""Standard MAVLink and an independent camera clock through actual MAVROS.

The camera header source is synthetic here; physical NET decoding is covered
by the separate host/Jetson streaming acceptance run.
"""

import json
import os
from pathlib import Path
import signal
import socket
import sqlite3
import subprocess
import time
from types import SimpleNamespace

from ament_index_python.packages import get_package_share_directory
import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from nav_msgs.msg import Path as RosPath
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

from lr_mavros_bridge.live_runtime import LiveRuntime
from lr_mavros_bridge.mission_path_node import MissionPath
from lr_mavros_bridge.node import MavrosPose


def test_independent_standard_sources_mission_and_recovery(tmp_path, monkeypatch):
    monkeypatch.setattr(sqlite3, 'connect', lambda *_a, **_k: (_ for _ in ()).throw(
        AssertionError('native backend opened a database')))
    ml = pytest.importorskip('pymavlink.dialects.v20.common')
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(('127.0.0.1', 0))
    reserve = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    reserve.bind(('127.0.0.1', 0))
    destination = ('127.0.0.1', reserve.getsockname()[1])
    reserve.close()
    udp.setblocking(False)
    class Writer:
        def write(self, packet):
            udp.sendto(packet, destination)
    mav, parser = ml.MAVLink(Writer(), srcSystem=7, srcComponent=1), ml.MAVLink(None)
    config = Path(get_package_share_directory('lr_mavros_bridge')) / 'config/live.yaml'
    log = (tmp_path / 'mavros.log').open('w')
    process = subprocess.Popen([
        'ros2', 'run', 'mavros', 'mavros_node', '--ros-args', '--params-file', str(config),
        '-p', f'fcu_url:=udp://127.0.0.1:{destination[1]}@127.0.0.1:{udp.getsockname()[1]}',
        '-p', "gcs_url:=''", '-p', 'fcu_protocol:=v2.0', '-p', 'target_system_id:=7',
    ], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init(args=['--ros-args', '-p', 'time_mode:=live'])
    runtime = LiveRuntime(parameter_overrides=[Parameter('camera_time_mode', value='fcu_utc'),
                                               Parameter('target_system_id', value=7)])
    child = SimpleNamespace(poll=lambda: None)
    runtime.launch_camera = lambda: setattr(runtime, 'child', child)
    runtime.stop_camera = lambda: setattr(runtime, 'child', None)
    pose = MavrosPose()
    mission = MissionPath(parameter_overrides=[Parameter('target_system_id', value=7)])
    probe = rclpy.create_node('independent_native_inputs')
    raw = probe.create_publisher(Header, '/lr/net/raw_frame', 10)
    frames = probe.create_publisher(Header, '/lr/net/frame_stamp', 10)
    poses, paths = [], []
    probe.create_subscription(PoseStamped, '/lr/mavlink/pose', poses.append, 10)
    probe.create_subscription(RosPath, '/lr/future_path/mission', paths.append, 10)
    executor = SingleThreadedExecutor()
    for node in (runtime, pose, mission, probe):
        executor.add_node(node)
    due = dict.fromkeys(('heartbeat', 'gps', 'attitude', 'frame', 'mission'), 0.)
    current, count, progress, relative = 0, 2, True, False
    mission_state, mission_mode = 3, 1
    camera_offset_ns = 30_000_000_000
    requests = []

    def spin(seconds, telemetry=True, camera=True, until=None):
        end = time.monotonic()+seconds
        while time.monotonic() < end:
            now = time.monotonic()
            if camera and now >= due['frame']:
                stamp = probe.get_clock().now().nanoseconds-camera_offset_ns
                header = Header()
                header.stamp.sec, header.stamp.nanosec = divmod(stamp, 1_000_000_000)
                raw.publish(header)
                if runtime.ready:
                    output = Header()
                    mapped = stamp+runtime.mapping['stamp_offset_ns']
                    output.stamp.sec, output.stamp.nanosec = divmod(mapped, 1_000_000_000)
                    frames.publish(output)
                due['frame'] = now+1/15
            if telemetry:
                boot_us = time.monotonic_ns()//1000
                if now >= due['heartbeat']:
                    mav.heartbeat_send(ml.MAV_TYPE_GROUND_ROVER, ml.MAV_AUTOPILOT_GENERIC,
                                       0, 0, ml.MAV_STATE_ACTIVE)
                    due['heartbeat'] = now+.5
                if now >= due['gps']:
                    mav.gps_raw_int_send(boot_us, 3, 100000000, 1060000000,
                                         100000, 100, 100, 0, 0, 12)
                    due['gps'] = now+.2
                if now >= due['attitude']:
                    mav.highres_imu_send(boot_us, 0., 0., -9.81, 0., 0., 0.,
                                         0., 0., 0., 1013.25, 0., 100., 25., 0x1fff)
                    mav.attitude_send((boot_us//1000) % 2**32, 0., 0., 0., 0., 0., 0.)
                    due['attitude'] = now+.02
                if progress and now >= due['mission']:
                    mav.mission_current_send(current, count, mission_state, mission_mode)
                    due['mission'] = now+.5
                try:
                    while True:
                        for msg in parser.parse_buffer(udp.recv(65535)) or []:
                            kind = msg.get_type()
                            if kind == 'TIMESYNC' and msg.tc1 == 0:
                                mav.timesync_send(time.monotonic_ns(), msg.ts1)
                            elif kind == 'COMMAND_LONG' and msg.command == 512 and msg.param1 == 2:
                                mav.system_time_send((time.time_ns()-camera_offset_ns)//1000,
                                                     time.monotonic_ns()//1_000_000)
                                mav.command_ack_send(512, 0, target_system=msg.get_srcSystem(),
                                                     target_component=msg.get_srcComponent())
                            elif kind == 'COMMAND_LONG' and msg.command == 410:
                                mav.home_position_send(100000000, 1060000000, 95000,
                                    0., 0., 0., [1., 0., 0., 0.], 0., 0., 0., boot_us)
                                mav.command_ack_send(410, 0)
                            elif kind == 'MISSION_REQUEST_LIST':
                                requests.append(kind)
                                mav.mission_count_send(msg.get_srcSystem(), msg.get_srcComponent(), count)
                            elif kind in ('MISSION_REQUEST_INT', 'MISSION_REQUEST'):
                                args = (msg.get_srcSystem(), msg.get_srcComponent(), msg.seq,
                                        (6 if relative else 5) if kind.endswith('INT') else (3 if relative else 0), 16,
                                        int(msg.seq == current), 1, 0., 0., 0., 0.)
                                lat = 10.0001 + msg.seq * .0001
                                if kind.endswith('INT'):
                                    mav.mission_item_int_send(*args, round(lat*1e7), 1060000000, 5. if relative else 100.)
                                else:
                                    mav.mission_item_send(*args, lat, 106., 5. if relative else 100.)
                except BlockingIOError:
                    pass
            executor.spin_once(timeout_sec=.003)
            if until and until():
                return

    try:
        spin(1., telemetry=False)
        assert runtime.camera_ready and not runtime.ready
        spin(18., until=lambda: len([p for p in paths if len(p.poses)>1]) >= 2)
        assert runtime.ready and mission.published, (runtime.status, mission.status,
                                                     (tmp_path/'mavros.log').read_text()[-2500:])
        assert requests and poses and all(p.header.frame_id == 'map' for p in paths)
        assert runtime.state['session_id'] == ''
        # Switch from a replay-like UTC to current real-world UTC without
        # changing any Jetson parameter, process or mission implementation.
        identity = runtime.runtime_id
        before_clock = runtime.camera_generation
        camera_offset_ns = 0
        spin(8., until=lambda: runtime.ready and mission.published and
             abs(runtime.mapping['stamp_offset_ns']) < 10_000_000)
        assert runtime.ready and mission.published and runtime.runtime_id == identity
        assert runtime.camera_generation > before_clock
        relative = True
        mission.next_pull = 0.
        spin(5., until=lambda: mission.snapshot and mission.snapshot[0].frame in (3, 6)
             and mission.home is not None and mission.published)
        assert mission.home == pytest.approx(95.) and mission.published
        before = len(poses)
        mission_state = 5
        spin(2., until=lambda: mission.status == 'mission_inactive')
        assert not mission.published and runtime.ready and len(poses) > before
        mission_state, mission_mode = 3, 2
        spin(2., until=lambda: mission.status == 'mission_suspended')
        assert not mission.published and runtime.ready
        mission_mode = 1
        spin(2., until=lambda: mission.published)
        assert mission.published
        camera_generation = runtime.camera_generation
        progress = False
        before = len(poses)
        spin(3.6)
        assert not mission.published and len(poses) > before and runtime.ready
        progress, current = True, 1
        spin(2., until=lambda: mission.published and mission.last_seq == 1)
        assert mission.published and mission.last_seq == 1
        count, current = 0, 0
        mission.next_pull = 0.
        spin(3., until=lambda: mission.snapshot == [])
        assert mission.snapshot == [] and not mission.published
        count = 2
        mission.next_pull = 0.
        spin(3., until=lambda: mission.published)
        assert mission.published
        spin(3.5, telemetry=False)
        assert runtime.camera_ready and not runtime.ready and not mission.published
        assert runtime.child is child and runtime.camera_generation == camera_generation
        spin(18., until=lambda: runtime.ready and mission.published)
        assert runtime.ready and mission.published
        before_sync_epoch = runtime.state['telemetry_epoch']
        spin(2.5, camera=False)
        assert not runtime.camera_ready and not runtime.ready
        assert runtime.state['telemetry_epoch'] == before_sync_epoch
        spin(4., until=lambda: runtime.ready and mission.published)
        assert runtime.ready and mission.published
        assert probe.count_publishers('/clock') == 0
        (tmp_path/'native-report.json').write_text(json.dumps({
            'passed': True, 'poses': len(poses), 'paths': len(paths),
            'mission_downloads': len(requests), 'source': runtime.state,
            'camera_generation': runtime.camera_generation,
        }, indent=2))
    finally:
        runtime.child = None
        executor.shutdown()
        for node in (runtime, pose, mission, probe):
            node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=2)
        log.close()
        udp.close()
