#!/usr/bin/env python3
# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Actual monitor acceptance escalation on isolated DDS195, fake controller only."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time


def main():
    """Exercise deadline histories independently of state recovery and RPC retry."""
    if os.environ.get('ROS_DOMAIN_ID') != '195':
        raise SystemExit('requires isolated ROS_DOMAIN_ID=195')
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.node import Node
    from rclpy.task import Future
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String
    from airstack_msgs.srv import TrajectoryMode

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--executable', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--expect-unsupported', action='store_true')
    args = parser.parse_args()
    executable = args.executable.resolve(strict=True)
    output = args.output or Path(tempfile.mkdtemp(prefix='pause-escalation-'))/'run'
    output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node('isolated_pause_escalation')
    events, checks, held_pause, held_operator = [], {}, [], []
    phase, behavior = 'unavailable', 'reject'
    latest = {}

    def record(kind, **fields):
        events.append(dict(kind=kind, phase=phase, monotonic=time.monotonic(), **fields))

    def received(kind, message):
        value = json.loads(message.data) if kind == 'status' else message.data
        latest[kind] = value
        record(kind, value=value)

    subscriptions = [node.create_subscription(
        msg_type, '/escalation_contract/'+topic,
        lambda message, name=kind: received(name, message), 10)
        for kind, topic, msg_type in (
            ('status', 'pause_intervention_status', String),
            ('escalation', 'pause_escalation_required', Bool),
            ('ack', 'pause_acknowledged', Bool),
            ('fault', 'state_estimate_timed_out', Bool))]
    state = node.create_publisher(Odometry, '/escalation_contract/state', 10)
    command = node.create_publisher(String, '/escalation_contract/command', 10)
    log = (output/'monitor.log').open('w')
    process = subprocess.Popen([
        str(executable), '--ros-args', '-r', '__ns:=/escalation_contract',
        '-r', 'state_estimate:=/escalation_contract/state',
        '-r', 'command:=/escalation_contract/command',
        '-r', 'set_trajectory_mode:=/escalation_contract/mode',
        '-p', 'health_publish_period_s:=0.05',
        '-p', 'pause_response_timeout_wall_s:=0.15',
        '-p', 'pause_retry_period_wall_s:=0.2',
        '-p', 'pause_escalation_timeout_wall_s:=0.6'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    service = None

    def spin(duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            message = Odometry()
            stamp = node.get_clock().now().nanoseconds
            message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, 10**9)
            message.pose.pose.orientation.w = 1.
            state.publish(message)
            rclpy.spin_once(node, timeout_sec=.01)
            if process.poll() is not None:
                raise RuntimeError('monitor exited: '+str(process.returncode))

    def until(predicate, timeout=2.5):
        end = time.monotonic()+timeout
        while not predicate() and time.monotonic() < end:
            spin(.01)
        return bool(predicate())

    def send(value):
        message = String()
        message.data = value
        command.publish(message)
        record('command', value=value)

    def statuses():
        return [event['value'] for event in events
                if event['kind'] == 'status' and event['phase'] == phase]

    async def mode(request, response):
        record('request', mode=request.mode)
        if request.mode == 0 and behavior == 'hold_pause':
            future = Future()
            held_pause.append(future)
            await future
        elif request.mode == 1 and behavior == 'hold_operator':
            future = Future()
            held_operator.append(future)
            await future
        response.success = not (request.mode == 0 and behavior == 'reject')
        record('applied', mode=request.mode, success=response.success)
        return response

    result = {}
    try:
        if args.expect_unsupported:
            spin(1.8)
            checks['baseline_monitor_publishes_fault'] = latest.get('fault') is True
            checks['baseline_has_no_escalation_telemetry'] = (
                'status' not in latest and 'escalation' not in latest)
        else:
            checks['unavailable_escalates'] = until(
                lambda: latest.get('status', {}).get('escalation_required') is True)
            first = latest['status']
            generation = first['generation']
            checks['unavailable_not_submitted'] = first['pause_submission_attempts'] == 0
            checks['unavailable_result'] = first['last_pause_result'] == 'service_unavailable'
            checks['deadline_not_early'] = first['elapsed_wall_s'] >= .6
            for _ in range(3):
                send('pause')
                spin(.08)
            checks['duplicate_pause_keeps_episode_and_deadline'] = (
                latest['status']['generation'] == generation and
                latest['status']['elapsed_wall_s'] > first['elapsed_wall_s'] and
                latest['status']['escalation_required'] is True)

            phase = 'rejection'
            service = node.create_service(
                TrajectoryMode, '/escalation_contract/mode', mode,
                callback_group=ReentrantCallbackGroup())
            checks['rejection_counted'] = until(
                lambda: latest['status']['pause_rejections'] > 0)
            checks['rejection_keeps_escalation'] = latest['status']['escalation_required']
            phase, behavior = 'response_timeout', 'hold_pause'
            checks['response_timeout_counted'] = until(
                lambda: latest['status']['pause_response_timeouts'] > 0)
            checks['timeout_keeps_same_episode'] = (
                latest['status']['generation'] == generation and
                latest['status']['escalation_required'] is True)

            phase, behavior = 'late_acceptance', 'accept'
            checks['late_acceptance_resolves_current_escalation'] = until(
                lambda: latest['status']['active'] is False)
            accepted = latest['status'].copy()
            for future in held_pause:
                future.set_result(None)
            held_pause.clear()
            spin(.3)
            checks['late_acceptance_retains_deadline_history'] = (
                accepted['deadline_exceeded'] is True and accepted['pause_acknowledged'] is True and
                accepted['escalation_required'] is False and accepted['elapsed_wall_s'] >= .6)
            checks['resolved_elapsed_is_frozen'] = (
                latest['status']['elapsed_wall_s'] == accepted['elapsed_wall_s'])
            checks['readiness_recovers_after_ack'] = until(
                lambda: latest.get('fault') is False and latest.get('ack') is True)

            phase, behavior = 'operator_unresolved', 'hold_operator'
            send('resume')
            checks['operator_received'] = until(lambda: bool(held_operator))
            checks['unresolved_operator_escalates_despite_pause_reply'] = until(
                lambda: latest['status']['escalation_required'] is True and
                latest['status']['operator_outcome_unresolved'] is True)
            operator_generation = latest['status']['generation']
            checks['operator_timeout_starts_new_episode'] = operator_generation == generation+1
            spin(.2)
            samples = statuses()
            checks['unresolved_operator_no_ack'] = (
                bool(samples) and all(not value['pause_acknowledged'] for value in samples))
            checks['unresolved_operator_blocks_readiness'] = (
                latest['fault'] is True and latest['ack'] is False)
            send('pause')
            spin(.15)
            checks['operator_duplicate_pause_retains_deadline'] = (
                latest['status']['generation'] == operator_generation and
                latest['status']['escalation_required'] is True)
            phase = 'operator_reconciled'
            released = time.monotonic()
            record('release_operator')
            for future in held_operator:
                future.set_result(None)
            held_operator.clear()
            checks['operator_reconciles_with_later_pause'] = until(
                lambda: latest['status']['active'] is False and
                latest['status']['pause_acknowledged'] is True)
            applied = [event for event in events if event['kind'] == 'applied' and
                       event['monotonic'] >= released]
            modes = [event['mode'] for event in applied]
            checks['renewed_pause_applied_after_operator'] = (
                1 in modes and 0 in modes[modes.index(1)+1:])
            checks['operator_resolution_retains_episode_history'] = (
                latest['status']['generation'] == operator_generation and
                latest['status']['deadline_exceeded'] is True)

            phase, behavior = 'new_episode', 'accept'
            send('pause')
            checks['new_episode_acknowledged'] = until(
                lambda: latest['status']['generation'] == operator_generation+1 and
                latest['status']['active'] is False)
            checks['new_episode_resets_deadline_history'] = (
                latest['status']['deadline_exceeded'] is False and
                latest['status']['escalation_required'] is False)
            all_statuses = [event['value'] for event in events if event['kind'] == 'status']
            sequences = [value['sequence'] for value in all_statuses]
            identities = {value['monitor_instance_id'] for value in all_statuses}
            checks['instance_identity_and_sequence'] = (
                len(identities) == 1 and bool(re.fullmatch('[0-9a-f]{32}', identities.pop())) and
                all(b > a for a, b in zip(sequences, sequences[1:])))
            checks['never_claims_physical_stop_or_exclusive_authority'] = all(
                value['physical_stop'] == 'UNVERIFIED' and
                value['authority_scope'] == 'monitor_local_requests_only'
                for value in all_statuses)
            checks['only_declared_modes'] = all(
                event['mode'] in (0, 1) for event in events if event['kind'] == 'request')
        result['verdict'] = 'PASS' if checks and all(checks.values()) else 'FAIL'
    except Exception as error:
        result.update(verdict='FAIL', error=repr(error))
    finally:
        result.update(checks=checks, baseline_unsupported=args.expect_unsupported,
                      scope='actual monitor with fake controller; request acceptance only',
                      executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
                      harness_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        for future in held_pause+held_operator:
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
        (output/'events.json').write_text(json.dumps(events, indent=2)+'\n')
        (output/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        if service is not None:
            node.destroy_service(service)
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, indent=2), flush=True)
    if result['verdict'] != 'PASS':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
