#!/usr/bin/env python3
# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Controller-local fenced input contract, isolated DDS194 and synthetic odometry."""
import argparse
import copy
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
    """Exercise process/generation/sequence checks on every controller command path."""
    if os.environ.get('ROS_DOMAIN_ID') != '194':
        raise SystemExit('requires isolated ROS_DOMAIN_ID=194')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from airstack_msgs.msg import Odometry as Reference
    from airstack_msgs.msg import TrajectoryXYZVYaw, WaypointXYZVYaw
    from airstack_msgs.srv import FencedControl, TrajectoryMode
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    from std_msgs.msg import String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    executable = args.executable.resolve(strict=True)
    output = args.output or Path(tempfile.mkdtemp(prefix='fenced-control-'))/'run'
    output.mkdir(parents=True, exist_ok=False)
    events, checks, logs = [], {}, []
    phase = 'startup'
    process = None
    state = None
    sequence = 0
    rclpy.init()
    node = Node('isolated_fenced_control')

    def record(kind, **fields):
        events.append(dict(kind=kind, phase=phase, monotonic=time.monotonic(), **fields))

    def reference(message):
        record('reference', position=[message.pose.position.x, message.pose.position.y,
                                      message.pose.position.z],
               velocity=[message.twist.linear.x, message.twist.linear.y,
                         message.twist.linear.z])

    subscriptions = [node.create_subscription(
        Reference, '/fence_contract/tracking_point', reference, 10),
        node.create_subscription(String, '/fence_contract/consumption_diagnostic',
                                 lambda m: record('diagnostic', value=json.loads(m.data)),
                                 qos_profile_sensor_data)]
    odometry = node.create_publisher(Odometry, '/fence_contract/odometry', 10)
    override = node.create_publisher(TrajectoryXYZVYaw, '/fence_contract/trajectory_override', 10)
    append = node.create_publisher(
        TrajectoryXYZVYaw, '/fence_contract/trajectory_segment_to_add', 10)
    client = node.create_client(FencedControl, '/fence_contract/fenced_control')
    legacy = node.create_client(TrajectoryMode, '/fence_contract/set_trajectory_mode')
    parameters = node.create_client(SetParameters,
                                    '/fence_contract/trajectory_control_node/set_parameters')

    def start():
        log = (output/f'controller-{len(logs)}.log').open('w')
        logs.append(log)
        return subprocess.Popen([
            str(executable), '--ros-args', '-r', '__ns:=/fence_contract',
            '-p', 'target_frame:=map', '-p', 'require_fenced_control:=true',
            '-p', 'fenced_control_authority:=contract-authority',
            '-p', 'tracking_point_distance_limit:=0.5',
            '-p', 'min_virtual_tracking_velocity:=0.1'],
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def stop():
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()

    def spin(duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            message = Odometry()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = message.child_frame_id = 'map'
            message.pose.pose.position.z = 1.
            message.pose.pose.orientation.w = 1.
            odometry.publish(message)
            rclpy.spin_once(node, timeout_sec=.01)
            if process.poll() is not None:
                raise RuntimeError('controller exited: '+str(process.returncode))

    def invoke(endpoint, request):
        future = endpoint.call_async(request)
        end = time.monotonic()+3.
        while not future.done() and time.monotonic() < end:
            spin(.01)
        if not future.done():
            raise RuntimeError('private service deadline')
        return future.result()

    def execute(request):
        nonlocal state
        response = invoke(client, request)
        state = response
        record('response', operation=request.operation, request_sequence=request.sequence,
               request_generation=request.stop_generation,
               success=response.success, duplicate=response.duplicate, reason=response.reason,
               enabled=response.enabled, instance=response.controller_instance_id,
               generation=response.stop_generation, last_sequence=response.last_sequence,
               latched=response.stop_latched, mode=response.mode)
        return response

    def query():
        return execute(FencedControl.Request(operation=FencedControl.Request.QUERY))

    def request(operation, **fields):
        nonlocal sequence
        sequence += 1
        value = FencedControl.Request(
            operation=operation, controller_instance_id=state.controller_instance_id,
            authority_id='contract-authority', stop_generation=state.stop_generation,
            sequence=sequence)
        for key, field in fields.items():
            setattr(value, key, field)
        return value

    def path(start=0., end=.3):
        value = TrajectoryXYZVYaw()
        value.header.frame_id = 'map'
        value.header.stamp = node.get_clock().now().to_msg()
        for index in range(5):
            point = WaypointXYZVYaw()
            point.position.x = start+(end-start)*index/4
            point.position.z = 1.
            point.velocity = .3
            value.waypoints.append(point)
        return value

    def references():
        return [event for event in events
                if event['kind'] == 'reference' and event['phase'] == phase]

    def frozen(anchor):
        values = references()
        return len(values) >= 3 and all(
            math.dist(value['position'], anchor) < 1e-6 and
            all(abs(speed) < 1e-9 for speed in value['velocity']) for value in values)

    result = {}
    try:
        process = start()
        spin(1.5)
        query()
        checks['guarded_start_latched_pause'] = (
            state.enabled and state.stop_latched and state.mode == 0)
        checks['startup_hold_uses_measured_pose'] = frozen([0., 0., 1.])
        initial_instance = state.controller_instance_id
        checks['all_legacy_release_modes_rejected'] = all(
            not invoke(legacy, TrajectoryMode.Request(mode=mode)).success
            for mode in (1, 2, 3, 4, 999))
        phase = 'legacy_inputs_latched'
        for _ in range(4):
            override.publish(path(-.4, -.2))
            append.publish(path(.5, .8))
            spin(.06)
        checks['legacy_topics_ignored_latched'] = frozen([0., 0., 1.])
        checks['mode_blocked_before_reset'] = not execute(request(3, mode=2)).success
        checks['trajectory_blocked_before_reset'] = (
            not execute(request(4, trajectory=path())).success)
        checks['wrong_authority_rejected'] = not execute(request(2, authority_id='other')).success
        checks['wrong_instance_rejected'] = (
            not execute(request(2, controller_instance_id='old')).success)
        checks['wrong_generation_rejected'] = not execute(request(2, stop_generation=999)).success
        checks['zero_sequence_rejected'] = not execute(request(2, sequence=0)).success
        checks['max_sequence_rejected'] = not execute(request(2, sequence=2**64-1)).success
        reset = request(2)
        response = execute(reset)
        checks['explicit_reset_keeps_pause'] = (
            response.success and not response.stop_latched and response.mode == 0)
        response = execute(reset)
        checks['exact_retry_idempotent'] = response.success and response.duplicate
        changed = copy.deepcopy(reset)
        changed.mode = 1
        checks['same_sequence_different_payload_rejected'] = (
            execute(changed).reason == 'sequence_conflict')
        checks['fresh_mode_accepted'] = execute(request(3, mode=2)).success
        checks['fresh_replace_accepted'] = execute(request(4, trajectory=path())).success
        phase = 'guarded_trajectory'
        spin(.4)
        checks['fresh_reference_observed'] = (
            any(value['position'][0] > .02 for value in references()))
        phase = 'legacy_override_unlatched'
        for _ in range(4):
            override.publish(path(-.4, -.2))
            spin(.06)
        checks['legacy_override_ignored_unlatched'] = len(references()) >= 3 and all(
            value['position'][0] >= 0 for value in references())
        phase = 'invalid_fenced_payloads'
        previous_sequence = state.last_sequence
        checks['invalid_fenced_mode_rejected'] = (
            execute(request(3, mode=999)).reason == 'invalid_mode')
        checks['pause_mode_requires_stop_operation'] = (
            execute(request(3, mode=0)).reason == 'invalid_mode')
        checks['invalid_operation_rejected'] = execute(request(255)).reason == 'invalid_operation'
        checks['stale_sequence_rejected'] = (
            execute(request(3, mode=2, sequence=1)).reason == 'stale_sequence')
        for kind in ('frame', 'empty', 'single', 'oversize', 'nan_position', 'inf_velocity',
                     'zero_velocity', 'negative_velocity', 'duplicate', 'nan_yaw', 'nan_accel',
                     'nan_jerk', 'overflow_duration'):
            value = path(-.5, -.2)
            if kind == 'frame':
                value.header.frame_id = 'unbound'
            elif kind == 'empty':
                value.waypoints = []
            elif kind == 'single':
                value.waypoints = value.waypoints[:1]
            elif kind == 'oversize':
                value.waypoints = value.waypoints*205
            elif kind == 'nan_position':
                value.waypoints[2].position.x = float('nan')
            elif kind == 'inf_velocity':
                value.waypoints[2].velocity = float('inf')
            elif kind in ('zero_velocity', 'negative_velocity'):
                value.waypoints[2].velocity = 0. if kind == 'zero_velocity' else -1.
            elif kind == 'duplicate':
                value.waypoints[2] = copy.deepcopy(value.waypoints[1])
            elif kind == 'nan_yaw':
                value.waypoints[2].yaw = float('nan')
            elif kind == 'nan_accel':
                value.waypoints[2].acceleration.x = float('nan')
            elif kind == 'nan_jerk':
                value.waypoints[2].jerk.z = float('nan')
            else:
                for point in value.waypoints:
                    point.velocity = 1e-308
            response = execute(request(4, trajectory=value))
            checks['invalid_trajectory_'+kind] = not response.success
        query()
        checks['rejections_and_query_do_not_consume_sequence'] = (
            state.last_sequence == previous_sequence)
        spin(.2)
        checks['invalid_payloads_leave_existing_reference'] = len(references()) >= 3 and all(
            value['position'][0] >= 0 for value in references())
        checks['append_mode_accepted'] = execute(request(3, mode=3)).success
        checks['fresh_append_accepted'] = execute(request(5, trajectory=path())).success
        phase = 'legacy_append_unlatched'
        for _ in range(4):
            append.publish(path(-.6, -.3))
            spin(.06)
        checks['legacy_append_ignored_unlatched'] = len(references()) >= 3 and all(
            value['position'][0] >= 0 for value in references())

        prepared = [request(operation, sequence=sequence+100, mode=2, trajectory=path(-.5, -.3))
                    for operation in (2, 3, 4, 5)]
        stop_request = request(1)
        old_generation = state.stop_generation
        response = execute(stop_request)
        generation = response.stop_generation
        checks['stop_latches_and_increments_generation'] = (
            response.success and response.stop_latched and response.mode == 0 and
            generation == old_generation+1)
        retry = execute(stop_request)
        checks['stop_retry_does_not_increment_generation'] = (
            retry.success and retry.duplicate and retry.stop_generation == generation)
        phase = 'stopped'
        spin(.2)
        if not references():
            raise RuntimeError('missing stopped reference')
        anchor = references()[-1]['position']
        phase = 'old_commands_after_stop'
        checks['delayed_old_reset_mode_replace_append_rejected'] = all(
            execute(value).reason == 'stop_generation_mismatch' for value in prepared)
        spin(.2)
        checks['stop_reference_frozen_zero_velocity'] = frozen(anchor)
        checks['new_generation_reset_accepted'] = execute(request(2)).success
        phase = 'reset_without_restart'
        spin(.2)
        checks['reset_does_not_restart_buffer'] = frozen(anchor) and state.mode == 0
        checks['old_mode_still_rejected_after_reset'] = (
            execute(prepared[1]).reason == 'stop_generation_mismatch')
        checks['new_generation_mode_accepted'] = execute(request(3, mode=2)).success
        phase = 'mode_without_old_buffer'
        spin(.2)
        checks['old_buffer_was_discarded'] = frozen(anchor)
        fresh = request(4, trajectory=path(.35, .5))
        checks['new_generation_replace_accepted'] = execute(fresh).success
        phase = 'fresh_after_reset'
        spin(.25)
        checks['fresh_command_after_reset_changes_reference'] = any(
            value['position'][0] >= .35 for value in references())
        before = state.stop_generation
        checks['legacy_pause_remains_protective'] = (
            invoke(legacy, TrajectoryMode.Request(mode=0)).success)
        query()
        checks['legacy_pause_invalidates_generation'] = (
            state.stop_latched and state.stop_generation == before+1)
        checks['legacy_pause_invalidates_cached_duplicate'] = not execute(fresh).success
        parameter_request = SetParameters.Request(parameters=[Parameter(
            name='require_fenced_control', value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL, bool_value=False))])
        checks['fence_profile_parameter_read_only'] = (
            not invoke(parameters, parameter_request).results[0].successful)
        stale_instance_request = request(2)
        phase = 'restart'
        stop()
        process = start()
        spin(1.)
        query()
        checks['restart_changes_instance_and_starts_latched'] = (
            state.controller_instance_id != initial_instance and state.stop_latched and
            state.stop_generation == 1 and state.last_sequence == 0)
        checks['pre_restart_request_rejected'] = (
            execute(stale_instance_request).reason == 'controller_instance_mismatch')
        phase = 'restarted_hold'
        spin(.2)
        checks['restart_hold_uses_fresh_measured_pose'] = frozen([0., 0., 1.])
        result['verdict'] = 'PASS' if checks and all(checks.values()) else 'FAIL'
    except Exception as error:
        result.update(verdict='FAIL', error=repr(error))
    finally:
        result.update(checks=checks,
                      scope='controller protocol/reference with synthetic odometry',
                      physical_execution=False, authenticated_authority=False,
                      executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
                      harness_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        stop()
        for log in logs:
            log.close()
        (output/'events.json').write_text(json.dumps(events, indent=2)+'\n')
        (output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, indent=2), flush=True)
    print('artifacts:', output, flush=True)
    if result['verdict'] != 'PASS':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
