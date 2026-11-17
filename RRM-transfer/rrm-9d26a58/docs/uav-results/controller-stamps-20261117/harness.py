#!/usr/bin/env python3
# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Contain malformed trajectory stamps on isolated DDS190, no simulator endpoints."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import resource
import signal
import subprocess
import tempfile
import time


def main():
    """Test legacy and fenced constructors with independently restarted processes."""
    if os.environ.get('ROS_DOMAIN_ID') != '190':
        raise SystemExit('requires isolated ROS_DOMAIN_ID=190')
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import serialize_message
    from nav_msgs.msg import Odometry
    from airstack_msgs.msg import TrajectoryXYZVYaw, WaypointXYZVYaw
    from airstack_msgs.srv import FencedControl, TrajectoryMode
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--record-baseline', action='store_true')
    args = parser.parse_args()
    executable = args.executable.resolve(strict=True)
    output = args.output or Path(tempfile.mkdtemp(prefix='invalid-stamp-'))/'run'
    output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node('isolated_invalid_stamps')
    events, cases = [], []
    phase = 'startup'
    process = None

    def record(kind, **fields):
        events.append(dict(kind=kind, phase=phase, monotonic=time.monotonic(), **fields))

    def spin(duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            message = Odometry()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = message.child_frame_id = 'map'
            message.pose.pose.orientation.w = 1.
            odometry.publish(message)
            rclpy.spin_once(node, timeout_sec=.01)

    def invoke(endpoint, request):
        discovery_end = time.monotonic()+5.
        while not endpoint.service_is_ready() and time.monotonic() < discovery_end:
            spin(.02)
        if not endpoint.service_is_ready():
            return None
        future = endpoint.call_async(request)
        end = time.monotonic()+2.
        while not future.done() and process.poll() is None and time.monotonic() < end:
            spin(.01)
        return future.result() if future.done() else None

    def path():
        value = TrajectoryXYZVYaw()
        value.header.frame_id = 'map'
        for index in range(5):
            point = WaypointXYZVYaw()
            point.position.x = index*.075
            point.velocity = .3
            value.waypoints.append(point)
        return value

    def samples():
        return [event['value'] for event in events if event['phase'] == phase and
                event['kind'] == 'diagnostic' and event['value'].get('event') == 'sample']

    try:
        for surface in ('legacy_replace', 'legacy_append', 'fenced_replace', 'fenced_prepare'):
            for invalid in ('negative_seconds', 'noncanonical_nanoseconds'):
                name = surface+'_'+invalid
                phase = name+'_startup'
                strict = surface.startswith('fenced')
                lease = surface == 'fenced_prepare'
                prefix = '/stamp_contract/'+name
                subscription = node.create_subscription(
                    String, prefix+'/consumption_diagnostic',
                    lambda message: record('diagnostic', value=json.loads(message.data)),
                    qos_profile_sensor_data)
                odometry = node.create_publisher(Odometry, prefix+'/odometry', 10)
                override = node.create_publisher(
                    TrajectoryXYZVYaw, prefix+'/trajectory_override', 10)
                append = node.create_publisher(
                    TrajectoryXYZVYaw, prefix+'/trajectory_segment_to_add', 10)
                client = node.create_client(FencedControl, prefix+'/fenced_control')
                prepare = node.create_client(FencedControl, prefix+'/prepare_fenced_control')
                legacy = node.create_client(TrajectoryMode, prefix+'/set_trajectory_mode')

                log = (output/(name+'.log')).open('w')
                process = subprocess.Popen([
                    str(executable), '--ros-args', '-r', '__ns:='+prefix,
                    '-p', 'target_frame:=map',
                    '-p', 'require_fenced_control:='+str(strict).lower(),
                    '-p', 'fenced_control_authority:=stamp-authority',
                    '-p', 'require_fenced_command_lease:='+str(lease).lower()],
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
                outcome = {'case': name, 'rejected_without_exit': False,
                           'valid_zero_stamp_accepted': False}
                try:
                    spin(.7)
                    current = invoke(client, FencedControl.Request(operation=0))
                    if current is None:
                        raise RuntimeError('private controller unavailable')
                    sequence = 0

                    def execute(operation, trajectory=None):
                        nonlocal sequence
                        sequence += 1
                        value = FencedControl.Request(
                            operation=operation,
                            controller_instance_id=current.controller_instance_id,
                            authority_id='stamp-authority',
                            stop_generation=current.stop_generation,
                            sequence=sequence, mode=2)
                        if trajectory is not None:
                            value.trajectory = trajectory
                        if lease:
                            response = invoke(prepare, value)
                            if response is None or not response.success:
                                return response
                            value.admission_lease = response.admission_lease
                        return invoke(client, value)

                    if strict:
                        if not execute(2).success or not execute(3).success:
                            raise RuntimeError('private fenced setup failed')
                    else:
                        mode = 3 if surface == 'legacy_append' else 2
                        if not invoke(legacy, TrajectoryMode.Request(mode=mode)).success:
                            raise RuntimeError('private legacy mode failed')
                    value = path()
                    if invalid == 'negative_seconds':
                        value.header.stamp.sec = -1
                    else:
                        value.header.stamp.nanosec = 1_000_000_000
                    wire = serialize_message(value)
                    wire_name = name+'.cdr'
                    (output/wire_name).write_bytes(wire)
                    outcome.update(wire_file=wire_name,
                                   wire_sha256=hashlib.sha256(wire).hexdigest())
                    phase = name+'_invalid'
                    if strict:
                        response = execute(4, value)
                        outcome['invalid_success'] = response.success if response else None
                        outcome['invalid_reason'] = response.reason if response else None
                        outcome['rejected_without_exit'] = (
                            process.poll() is None and response is not None and
                            not response.success)
                    else:
                        endpoint = append if surface == 'legacy_append' else override
                        endpoint.publish(value)
                        spin(.2)
                        received = samples()
                        outcome['rejected_without_exit'] = (
                            process.poll() is None and bool(received) and
                            all(sample['waypoints'] == 0 for sample in received))
                    if process.poll() is None:
                        phase = name+'_valid_zero_stamp'
                        valid = path()
                        if strict:
                            response = execute(4, valid)
                            outcome['valid_zero_stamp_accepted'] = (
                                response is not None and response.success)
                        else:
                            endpoint.publish(valid)
                            spin(.2)
                            outcome['valid_zero_stamp_accepted'] = any(
                                sample['waypoints'] == 5 for sample in samples())
                except Exception as error:
                    outcome['error'] = repr(error)
                finally:
                    outcome['process_exit_before_cleanup'] = process.poll()
                    if process.poll() is None:
                        os.killpg(process.pid, signal.SIGINT)
                        try:
                            process.wait(timeout=5)
                        except subprocess.TimeoutExpired:
                            os.killpg(process.pid, signal.SIGKILL)
                            process.wait()
                    log.close()
                    process = None
                    cases.append(outcome)
                    node.destroy_subscription(subscription)
                    for publisher in (odometry, override, append):
                        node.destroy_publisher(publisher)
                    for endpoint in (client, prepare, legacy):
                        node.destroy_client(endpoint)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    passed = sum(case['rejected_without_exit'] and case['valid_zero_stamp_accepted']
                 for case in cases)
    summary = {
        'verdict': 'PASS' if passed == 8 else 'FAIL', 'passed': passed, 'total': 8,
        'cases': cases, 'recorded_baseline': args.record_baseline,
        'scope': 'actual controller malformed-stamp containment; synthetic odometry',
        'physical_execution': False,
        'executable_sha256': hashlib.sha256(executable.read_bytes()).hexdigest(),
        'harness_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    }
    (output/'events.json').write_text(json.dumps(events, indent=2)+'\n')
    (output/'summary.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary, indent=2), flush=True)
    print('artifacts:', output, flush=True)
    if passed != 8 and not args.record_baseline:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
