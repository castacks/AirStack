#!/usr/bin/env python3
# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Controller-local steady command-lease checks on isolated DDS192."""
import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    """Test exact preparation binding, bounded expiry and already-accepted retry."""
    if os.environ.get('ROS_DOMAIN_ID') != '192':
        raise SystemExit('requires isolated ROS_DOMAIN_ID=192')
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.node import Node
    from rclpy.serialization import serialize_message
    from rclpy.task import Future
    from nav_msgs.msg import Odometry
    from airstack_msgs.msg import TrajectoryXYZVYaw, WaypointXYZVYaw
    from airstack_msgs.srv import FencedControl, TrajectoryMode

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--paused-clock', action='store_true')
    parser.add_argument('--lease-wall-s', type=float, default=.25)
    args = parser.parse_args()
    executable = args.executable.resolve(strict=True)
    output = args.output or Path(tempfile.mkdtemp(prefix='command-lease-'))/'run'
    output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node('isolated_command_lease')
    events, checks, releases = [], {}, []
    phase = 'startup'
    current = None
    sequence = 0

    def record(kind, **fields):
        events.append(dict(kind=kind, phase=phase, monotonic=time.monotonic(), **fields))

    state = node.create_publisher(Odometry, '/lease_contract/odometry', 10)
    client = node.create_client(FencedControl, '/lease_contract/fenced_control')
    prepare = node.create_client(FencedControl, '/lease_contract/prepare_fenced_control')
    forwarded = node.create_client(FencedControl, '/lease_contract/fenced_control',
                                   callback_group=ReentrantCallbackGroup())
    delayed = node.create_client(FencedControl, '/lease_contract/delayed_control')
    legacy = node.create_client(TrajectoryMode, '/lease_contract/set_trajectory_mode')

    async def proxy(request, response):
        wire = serialize_message(request)
        (output/'delayed-command.cdr').write_bytes(wire)
        record('proxy_received', wire_sha256=hashlib.sha256(wire).hexdigest())
        future = Future()
        releases.append(future)
        await future
        record('proxy_forwarding')
        result = await forwarded.call_async(request)
        record('proxy_reply', success=result.success, reason=result.reason)
        return result

    service = node.create_service(FencedControl, '/lease_contract/delayed_control', proxy,
                                  callback_group=ReentrantCallbackGroup())
    log = (output/'controller.log').open('w')
    process = subprocess.Popen([
        str(executable), '--ros-args', '-r', '__ns:=/lease_contract',
        '-p', 'target_frame:=map', '-p', 'require_fenced_control:=true',
        '-p', 'fenced_control_authority:=lease-authority',
        '-p', 'require_fenced_command_lease:=true',
        '-p', 'fenced_command_lease_wall_s:='+str(args.lease_wall_s),
        '-p', 'use_sim_time:='+str(args.paused_clock).lower()],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def spin(duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            message = Odometry()
            message.header.stamp = node.get_clock().now().to_msg()
            message.header.frame_id = message.child_frame_id = 'map'
            message.pose.pose.orientation.w = 1.
            state.publish(message)
            rclpy.spin_once(node, timeout_sec=.01)
            if process.poll() is not None:
                raise RuntimeError('controller exited: '+str(process.returncode))

    def until(predicate):
        end = time.monotonic()+3.
        while not predicate() and time.monotonic() < end:
            spin(.01)
        return bool(predicate())

    def invoke(endpoint, request):
        future = endpoint.call_async(request)
        if not until(future.done):
            raise RuntimeError('private RPC deadline')
        return future.result()

    def call(endpoint, request):
        nonlocal current
        response = invoke(endpoint, request)
        current = response
        record('response', preparing=endpoint is prepare, operation=request.operation,
               request_sequence=request.sequence, success=response.success,
               duplicate=response.duplicate, reason=response.reason,
               token_sha256=hashlib.sha256(response.admission_lease.encode()).hexdigest(),
               lease_remaining_s=response.lease_valid_for_wall_s,
               generation=response.stop_generation, sequence=response.last_sequence,
               latched=response.stop_latched, mode=response.mode)
        return response

    def request(operation, **fields):
        nonlocal sequence
        sequence += 1
        value = FencedControl.Request(
            operation=operation, controller_instance_id=current.controller_instance_id,
            authority_id='lease-authority', stop_generation=current.stop_generation,
            sequence=sequence)
        for name, field in fields.items():
            setattr(value, name, field)
        return value

    def prepared(value):
        response = call(prepare, value)
        if not response.success:
            raise RuntimeError('preparation failed: '+response.reason)
        result = copy.deepcopy(value)
        result.admission_lease = response.admission_lease
        return result

    def path():
        value = TrajectoryXYZVYaw()
        value.header.frame_id = 'map'
        for index in range(5):
            point = WaypointXYZVYaw()
            point.position.x = index*.075
            point.velocity = .3
            value.waypoints.append(point)
        return value

    result = {}
    try:
        spin(1.)
        response = call(client, FencedControl.Request(operation=0))
        checks['lease_profile_advertised'] = response.enabled and response.lease_required
        first = request(2)
        checks['missing_lease_rejected'] = (
            call(client, first).reason == 'lease_missing_or_mismatch')
        initial = call(prepare, first)
        checks['prepare_does_not_reset_or_consume_sequence'] = (
            initial.success and initial.stop_latched and initial.last_sequence == 0)
        spin(.05)
        duplicate = call(prepare, first)
        checks['duplicate_prepare_does_not_extend_deadline'] = (
            duplicate.success and duplicate.duplicate and
            duplicate.admission_lease == initial.admission_lease and
            duplicate.lease_valid_for_wall_s < initial.lease_valid_for_wall_s-.02)
        stale = copy.deepcopy(first)
        stale.admission_lease = initial.admission_lease
        spin(args.lease_wall_s+.05)
        checks['expired_reset_rejected'] = call(client, stale).reason == 'lease_expired'
        checks['expired_prepare_cannot_renew_same_sequence'] = (
            call(prepare, first).reason == 'lease_expired')
        checks['expired_reset_keeps_latch_and_sequence'] = (
            current.stop_latched and current.last_sequence == 0)
        reset = prepared(request(2))
        checks['fresh_prepared_reset_accepted'] = call(client, reset).success
        mode = prepared(request(3, mode=2))
        changed = copy.deepcopy(mode)
        changed.mode = 1
        checks['changed_mode_payload_rejected'] = (
            call(client, changed).reason == 'lease_missing_or_mismatch')
        checks['exact_prepared_mode_accepted'] = call(client, mode).success
        spin(args.lease_wall_s+.05)
        retry = call(client, mode)
        checks['accepted_retry_after_expiry_is_idempotent'] = retry.success and retry.duplicate
        proposed = request(4, trajectory=path())
        bound = prepared(proposed)
        changed = copy.deepcopy(bound)
        changed.trajectory.waypoints[2].position.x += .01
        checks['changed_trajectory_payload_rejected'] = (
            call(client, changed).reason == 'lease_missing_or_mismatch')
        phase = 'delayed_after_expiry'
        pending = delayed.call_async(bound)
        checks['leased_command_actually_submitted_and_held'] = until(lambda: bool(releases))
        spin(args.lease_wall_s+.05)
        record('release_proxy')
        releases[0].set_result(None)
        checks['delayed_reply_received'] = until(pending.done)
        checks['actual_delayed_command_rejected_after_expiry'] = (
            pending.done() and not pending.result().success and
            pending.result().reason == 'lease_expired')
        phase = 'stop_invalidates_preparation'
        candidate = prepared(request(4, trajectory=path()))
        before = current.stop_generation
        checks['legacy_pause_still_available_without_lease'] = invoke(
            legacy, TrajectoryMode.Request(mode=0)).success
        call(client, FencedControl.Request(operation=0))
        checks['stop_advances_generation'] = current.stop_generation == before+1
        checks['stop_invalidates_old_lease'] = (
            call(client, candidate).reason == 'stop_generation_mismatch')
        fresh_reset = prepared(request(2))
        checks['fresh_generation_lease_and_reset_work'] = call(client, fresh_reset).success
        still_valid = prepared(request(3, mode=2))
        checks['invalid_preparation_rejected'] = (
            call(prepare, request(3, mode=999)).reason == 'invalid_mode')
        checks['invalid_preparation_does_not_cancel_valid_token'] = (
            call(client, still_valid).success)
        older_request = request(3, mode=2)
        older = prepared(older_request)
        newer = prepared(request(3, mode=2))
        checks['superseded_preparation_cannot_remint_token'] = (
            call(prepare, older_request).reason == 'stale_or_conflicting_preparation')
        checks['newer_preparation_invalidates_older_token'] = (
            call(client, older).reason == 'lease_missing_or_mismatch')
        checks['newest_preparation_still_accepted'] = call(client, newer).success
        stop_request = request(1)
        checks['fenced_stop_needs_no_lease'] = call(client, stop_request).success
        retry = call(client, stop_request)
        checks['fenced_stop_retry_idempotent'] = retry.success and retry.duplicate
        result['verdict'] = 'PASS' if checks and all(checks.values()) else 'FAIL'
    except Exception as error:
        result.update(verdict='FAIL', error=repr(error))
    finally:
        for future in releases:
            if not future.done():
                future.set_result(None)
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        log.close()
        result.update(checks=checks, scope='actual controller local lease/expiry protocol',
                      paused_simulation_clock=args.paused_clock,
                      lease_budget_wall_s=args.lease_wall_s,
                      physical_execution=False, authenticated_authority=False,
                      executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
                      harness_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        (output/'events.json').write_text(json.dumps(events, indent=2)+'\n')
        (output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
        node.destroy_service(service)
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, indent=2), flush=True)
    print('artifacts:', output, flush=True)
    if result['verdict'] != 'PASS':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
