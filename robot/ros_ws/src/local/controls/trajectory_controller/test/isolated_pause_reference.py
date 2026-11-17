#!/usr/bin/env python3
# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""
Actual-controller reference freeze with synthetic inputs on isolated DDS196.

No simulator or aircraft endpoints. Output stability is not physical stop evidence
or exclusive controller authority; valid mode-service writers can still unpause.
"""
import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    """Test PAUSE against late override and invalid-mode inputs."""
    if os.environ.get('ROS_DOMAIN_ID') != '196':
        raise SystemExit('requires isolated ROS_DOMAIN_ID=196')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from airstack_msgs.msg import Odometry as TrackingPoint
    from airstack_msgs.msg import TrajectoryXYZVYaw, WaypointXYZVYaw
    from airstack_msgs.srv import TrajectoryMode
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--executable', type=Path, required=True)
    args = parser.parse_args()
    output = args.output or Path(tempfile.mkdtemp(prefix='pause-reference-'))/'run'
    output.mkdir(parents=True, exist_ok=False)
    executable = args.executable.resolve(strict=True)
    rclpy.init()
    node = Node('isolated_pause_reference')
    events, checks = [], {}
    phase = 'startup'

    def record(kind, **fields):
        events.append(dict(kind=kind, phase=phase, monotonic=time.monotonic(), **fields))

    def sample(msg):
        p = msg.pose.position
        record('reference', position=[p.x, p.y, p.z],
               velocity=[msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z])

    odom = node.create_publisher(Odometry, '/pause_contract/odometry', 10)
    override = node.create_publisher(TrajectoryXYZVYaw, '/pause_contract/trajectory_override', 10)
    subs = [node.create_subscription(TrackingPoint, '/pause_contract/tracking_point', sample, 10),
            node.create_subscription(String, '/pause_contract/consumption_diagnostic',
                                     lambda m: record('diagnostic', value=json.loads(m.data)),
                                     qos_profile_sensor_data)]
    client = node.create_client(TrajectoryMode, '/pause_contract/set_trajectory_mode')
    log = (output/'controller.log').open('w')
    process = subprocess.Popen([
        str(executable), '--ros-args', '-r', '__ns:=/pause_contract',
        '-p', 'target_frame:=map', '-p', 'tracking_point_distance_limit:=0.5',
        '-p', 'min_virtual_tracking_velocity:=0.1'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def spin(duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            msg = Odometry()
            msg.header.stamp = node.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.child_frame_id = 'map'
            msg.pose.pose.position.z = 1.
            msg.pose.pose.orientation.w = 1.
            odom.publish(msg)
            rclpy.spin_once(node, timeout_sec=.01)
            if process.poll() is not None:
                raise RuntimeError('controller exited: '+str(process.returncode))

    def mode(value):
        request = TrajectoryMode.Request()
        request.mode = value
        future = client.call_async(request)
        deadline = time.monotonic()+2.
        while not future.done() and time.monotonic() < deadline:
            spin(.01)
        if not future.done():
            raise RuntimeError('mode response timeout')
        success = future.result().success
        record('mode_result', mode=value, success=success)
        return success

    def trajectory(start, end):
        msg = TrajectoryXYZVYaw()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for i in range(5):
            point = WaypointXYZVYaw()
            point.position.x = start+(end-start)*i/4
            point.position.z = 1.
            point.velocity = .2
            msg.waypoints.append(point)
        override.publish(msg)
        record('override', start_x=start, end_x=end)

    def references(name):
        return [e for e in events if e['phase'] == name and e['kind'] == 'reference']

    result = {}
    try:
        spin(2.)
        if not client.service_is_ready():
            raise RuntimeError('private mode service unavailable')
        checks['track_admitted'] = mode(2)
        phase = 'initial'
        trajectory(0., .2)
        spin(.6)
        checks['initial_reference_observed'] = len(references(phase)) >= 5
        checks['pause_admitted'] = mode(0)
        phase = 'paused'
        spin(.3)
        paused = references(phase)
        if len(paused) < 3:
            raise RuntimeError('paused output coverage missing')
        anchor = paused[-1]['position']
        phase = 'conflicting_override'
        trajectory(-.35, -.15)
        spin(.5)
        samples = references(phase)
        excursion = max((math.dist(e['position'], anchor) for e in samples), default=None)
        checks['paused_override_reference_frozen'] = (
            len(samples) >= 5 and excursion is not None and excursion <= 1e-6)
        checks['paused_output_zero_velocity'] = bool(samples) and all(
            all(abs(v) <= 1e-9 for v in e['velocity']) for e in samples)
        result['paused_reference_excursion_m'] = excursion
        # Invalid values must be rejected without changing the PAUSE state.
        phase = 'invalid_mode'
        checks['invalid_mode_rejected'] = not mode(999)
        spin(.3)
        diagnostics = [e['value'] for e in events if e['phase'] == phase and
                       e['kind'] == 'diagnostic' and e['value'].get('event') == 'sample']
        checks['invalid_mode_preserves_pause'] = bool(diagnostics) and all(
            item['mode'] == 0 for item in diagnostics)
        mode(0)
        checks['explicit_track_admitted'] = mode(2)
        phase = 'explicit_resume_override'
        trajectory(-.35, -.15)
        spin(.5)
        resumed = references(phase)
        checks['normal_override_after_explicit_mode'] = len(resumed) >= 5 and any(
            e['position'][0] < -.1 for e in resumed)
        result['verdict'] = 'PASS' if all(checks.values()) else 'FAIL'
    except Exception as exc:
        result.update(verdict='FAIL', error=repr(exc))
    finally:
        result.update(checks=checks,
                      scope=('synthetic reference stream only; '
                             'no physical stop or exclusive authority'),
                      executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
                      harness_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        log.close()
        (output/'events.json').write_text(json.dumps(events, indent=2)+'\n')
        (output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
        for subscription in subs:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, indent=2), flush=True)
    print('artifacts:', output, flush=True)
    if result['verdict'] != 'PASS':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
