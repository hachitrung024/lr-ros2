#!/usr/bin/env python3
"""Smoke-test the display command, pause/resume, and SVO rewind using services."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from safety_perception_msgs.msg import PredictionOutput, RoverState, Trajectory
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from zed_msgs.msg import SvoStatus
from zed_msgs.srv import SetSvoFrame


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--svo', required=True)
    parser.add_argument('--model', default='models/best.pt')
    parser.add_argument('--output', default='log/prediction-replay-smoke')
    parser.add_argument('--headless', action='store_true')
    args = parser.parse_args()
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=True)
    command = [
        'ros2',
        'launch',
        'lr_display_rviz2',
        'display_zed_cam.launch.py',
        'camera_model:=zed2i',
        'publish_svo_clock:=true',
        f'segmentation_model_path:={args.model}',
        'mavlink:=true',
        'future_path:=true',
        f'svo_path:={args.svo}',
        'prediction_profile:=dynamic',
    ]
    if args.headless:
        command.append('start_rviz:=false')
    rclpy.init()
    node = rclpy.create_node('prediction_replay_smoke')
    clocks, trajectories, predictions, states, clears, statuses = [], [], [], [], [], []
    node.create_subscription(
        Clock, '/clock', lambda m: clocks.append(seconds(m.clock)), qos_profile_sensor_data
    )
    node.create_subscription(Trajectory, '/trajectory', trajectories.append, 10)
    node.create_subscription(PredictionOutput, '/predict_output', predictions.append, 10)
    node.create_subscription(RoverState, '/rover/state', states.append, qos_profile_sensor_data)
    node.create_subscription(
        MarkerArray,
        '/lr/path_prediction/markers',
        lambda m: clears.extend(
            seconds(x.header.stamp) for x in m.markers if x.action == Marker.DELETEALL
        ),
        10,
    )
    node.create_subscription(SvoStatus, '/zed/zed_node/status/svo', statuses.append, 10)
    toggle = node.create_client(Trigger, '/zed/zed_node/toggle_svo_pause')
    seek = node.create_client(SetSvoFrame, '/zed/zed_node/set_svo_frame')

    def spin_until(condition, timeout=30):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if condition():
                return
        raise AssertionError('Timed out waiting for replay condition')

    def spin_for(duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)

    def request(client, message):
        spin_until(client.service_is_ready)
        pending = client.call_async(message)
        spin_until(pending.done)
        result = pending.result()
        assert result.success, result.message

    log = (output / 'launch.log').open('w')
    child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    report = {'command': command}
    try:
        spin_until(lambda: len(predictions) >= 5, timeout=60)
        expected = {
            'mavlink_pose',
            'segmentation',
            'box_estimator_3d',
            'terrain_geometry',
            'prediction_bridge_node',
            'prediction_node',
            'canonical_prediction_visualizer',
        }
        names = set(node.get_node_names())
        assert expected <= names, names
        assert not any(name.endswith('_adapter_node') for name in names), names
        if not args.headless:
            assert 'zed2i_rviz2' in names, names
        report['lr_nodes'] = sorted(expected)
        report['rviz_present'] = 'zed2i_rviz2' in names
        request(toggle, Trigger.Request())
        # The SDK can stop emitting SvoStatus while paused; verify the clock itself.
        spin_for(1.0)  # Drain in-flight perception and prediction callbacks.
        paused_clock, paused_count = clocks[-1], len(predictions)
        spin_for(1.5)
        assert clocks[-1] == paused_clock
        assert len(predictions) == paused_count
        report['pause_stops_clock_and_prediction'] = True
        request(toggle, Trigger.Request())
        spin_until(lambda: clocks[-1] > paused_clock + 1 and len(predictions) > paused_count)
        report['resume_recovers_prediction'] = True
        old_clock = clocks[-1]
        old_id = trajectories[-1].trajectory_id
        start_state, start_clear, start_prediction = len(states), len(clears), len(predictions)
        request(seek, SetSvoFrame.Request(frame_id=0))
        spin_until(lambda: clocks[-1] < old_clock - 1)
        spin_until(
            lambda: any(
                seconds(m.header.stamp) < old_clock - 1 and m.trajectory_id > old_id
                for m in trajectories
            )
        )
        first_new_id = min(
            m.trajectory_id
            for m in trajectories
            if seconds(m.header.stamp) < old_clock - 1 and m.trajectory_id > old_id
        )
        # Terrain needs to rebuild after rewind; recovery may be later than
        # the old clock value. Match the new trajectory IDs, not a time cutoff.
        spin_until(
            lambda: any(
                m.source_trajectory_id >= first_new_id for m in predictions[start_prediction:]
            ),
            timeout=30,
        )
        new_states = [m for m in states[start_state:] if seconds(m.header.stamp) < old_clock - 1]
        assert new_states and not new_states[0].acceleration_valid
        assert any(stamp < old_clock - 1 for stamp in clears[start_clear:])
        new_outputs = [
            m for m in predictions[start_prediction:] if m.source_trajectory_id >= first_new_id
        ]
        assert all(m.source_trajectory_id > old_id for m in new_outputs)
        report.update(
            rewind_clock=True,
            monotonic_trajectory_ids=True,
            rewind_clears_acceleration=True,
            rewind_clears_markers=True,
            prediction_recovers_after_rewind=True,
        )
    except Exception as error:
        report['error'] = str(error)
        report['last_clock'] = clocks[-1] if clocks else None
        report['prediction_count'] = len(predictions)
        report['last_prediction_id'] = (
            predictions[-1].source_trajectory_id if predictions else None
        )
        raise
    finally:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
        try:
            child.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGTERM)
            child.wait(timeout=10)
        log.close()
        node.destroy_node()
        rclpy.shutdown()
        (output / 'report.json').write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
