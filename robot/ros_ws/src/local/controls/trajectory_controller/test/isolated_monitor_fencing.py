#!/usr/bin/env python3
# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""Actual monitor/controller interaction, isolated DDS193, no simulator endpoints."""
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
    """Hold a real reset RPC across a newer protective PAUSE and restart."""
    if os.environ.get('ROS_DOMAIN_ID') != '193':
        raise SystemExit('requires isolated ROS_DOMAIN_ID=193')
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import serialize_message
    from rclpy.task import Future
    from nav_msgs.msg import Odometry
    from airstack_msgs.msg import Odometry as Reference
    from airstack_msgs.msg import TrajectoryXYZVYaw, WaypointXYZVYaw
    from airstack_msgs.srv import FencedControl
    from std_msgs.msg import Bool, Float32, String

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--controller', type=Path, required=True)
    parser.add_argument('--monitor', type=Path, required=True)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    controller_binary = args.controller.resolve(strict=True)
    monitor_binary = args.monitor.resolve(strict=True)
    output = args.output or Path(tempfile.mkdtemp(prefix='monitor-fencing-'))/'run'
    output.mkdir(parents=True, exist_ok=False)
    events, checks, logs = [], {}, []
    phase, state_kind = 'startup', 'fresh'
    controller = monitor = None
    sequence = 0
    current = None
    latest = {}
    held = []
    rclpy.init()
    node = Node('isolated_monitor_fencing')

    def record(kind, **fields):
        events.append(dict(kind=kind, phase=phase, monotonic=time.monotonic(), **fields))

    def reference(message):
        record('reference', position=[message.pose.position.x, message.pose.position.y,
                                      message.pose.position.z],
               velocity=[message.twist.linear.x, message.twist.linear.y,
                         message.twist.linear.z])

    def received(kind, message):
        value = json.loads(message.data) if kind == 'intervention' else message.data
        latest[kind] = value
        record(kind, value=value)

    subscriptions = [node.create_subscription(
        Reference, '/fence_integration/tracking_point', reference, 10),
        node.create_subscription(Float32, '/fence_integration/trajectory_completion_percentage',
                                 lambda m: record('completion', finite=math.isfinite(m.data),
                                                  value=m.data if math.isfinite(m.data) else None),
                                 10),
        node.create_subscription(String, '/fence_integration/consumption_diagnostic',
                                 lambda m: record('diagnostic', value=json.loads(m.data)),
                                 qos_profile_sensor_data)]
    for kind, topic, msg_type in (
            ('ack', 'pause_acknowledged', Bool), ('fault', 'state_estimate_timed_out', Bool),
            ('reason', 'state_health_reason', String),
            ('escalation', 'pause_escalation_required', Bool),
            ('intervention', 'pause_intervention_status', String)):
        subscriptions.append(node.create_subscription(
            msg_type, '/fence_integration/monitor/'+topic,
            lambda message, key=kind: received(key, message), 10))
    odometry = node.create_publisher(Odometry, '/fence_integration/odometry', 10)
    monitored = node.create_publisher(Odometry, '/fence_integration/state', 10)
    command = node.create_publisher(String, '/fence_integration/monitor/command', 10)
    client = node.create_client(FencedControl, '/fence_integration/fenced_control')
    forwarded = node.create_client(FencedControl, '/fence_integration/fenced_control',
                                   callback_group=ReentrantCallbackGroup())
    delayed = node.create_client(FencedControl, '/fence_integration/delayed_control')

    async def proxy(request, response):
        wire = serialize_message(request)
        name = 'held-request-'+str(len(held))+'.cdr'
        (output/name).write_bytes(wire)
        record('proxy_received', generation=request.stop_generation, sequence=request.sequence,
               operation=request.operation, wire_file=name,
               wire_sha256=hashlib.sha256(wire).hexdigest())
        release = Future()
        held.append(release)
        await release
        record('proxy_forwarding', generation=request.stop_generation,
               sequence=request.sequence)
        reply = await forwarded.call_async(request)
        record('proxy_reply', success=reply.success, reason=reply.reason,
               generation=reply.stop_generation, latched=reply.stop_latched)
        return reply

    service = node.create_service(
        FencedControl, '/fence_integration/delayed_control', proxy,
        callback_group=ReentrantCallbackGroup())

    def launch(binary, name, arguments):
        log = (output/(name+'.log')).open('w')
        logs.append(log)
        return subprocess.Popen([str(binary), '--ros-args']+arguments,
                                stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def start_controller(name):
        return launch(controller_binary, name, [
            '-r', '__ns:=/fence_integration', '-p', 'target_frame:=map',
            '-p', 'require_fenced_control:=true',
            '-p', 'min_virtual_tracking_velocity:=0.1',
            '-p', 'fenced_control_authority:=integration-authority'])

    def stop(process):
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
            if state_kind == 'stale':
                message.header.stamp.sec -= 10
            monitored.publish(message)
            rclpy.spin_once(node, timeout_sec=.01)
            for process in (controller, monitor):
                if process is not None and process.poll() is not None:
                    raise RuntimeError('private child exited: '+str(process.returncode))

    def until(predicate, timeout=3.):
        end = time.monotonic()+timeout
        while not predicate() and time.monotonic() < end:
            spin(.01)
        return bool(predicate())

    def invoke(endpoint, request):
        future = endpoint.call_async(request)
        if not until(future.done):
            raise RuntimeError('private service deadline')
        return future.result()

    def execute(request):
        nonlocal current
        response = invoke(client, request)
        current = response
        record('controller_response', operation=request.operation, success=response.success,
               reason=response.reason, instance=response.controller_instance_id,
               generation=response.stop_generation, sequence=response.last_sequence,
               mode=response.mode, latched=response.stop_latched)
        return response

    def query():
        return execute(FencedControl.Request(operation=0))

    def request(operation, **fields):
        nonlocal sequence
        sequence += 1
        value = FencedControl.Request(
            operation=operation, controller_instance_id=current.controller_instance_id,
            authority_id='integration-authority', stop_generation=current.stop_generation,
            sequence=sequence)
        for key, field in fields.items():
            setattr(value, key, field)
        return value

    def path():
        value = TrajectoryXYZVYaw()
        value.header.frame_id = 'map'
        value.header.stamp = node.get_clock().now().to_msg()
        for index in range(5):
            point = WaypointXYZVYaw()
            point.position.x = index*.075
            point.position.z = 1.
            point.velocity = .3
            value.waypoints.append(point)
        return value

    def references():
        return [event for event in events if event['phase'] == phase and
                event['kind'] == 'reference']

    result = {}
    try:
        controller = start_controller('controller-0')
        monitor = launch(monitor_binary, 'monitor', [
            '-r', '__ns:=/fence_integration/monitor',
            '-r', 'state_estimate:=/fence_integration/state',
            '-r', 'set_trajectory_mode:=/fence_integration/set_trajectory_mode',
            '-p', 'health_publish_period_s:=0.05',
            '-p', 'pause_response_timeout_wall_s:=0.15',
            '-p', 'pause_retry_period_wall_s:=0.2',
            '-p', 'pause_escalation_timeout_wall_s:=0.6'])
        checks['startup_monitor_acknowledged_and_healthy'] = until(
            lambda: latest.get('ack') is True and latest.get('fault') is False)
        query()
        checks['monitor_startup_pause_latches_controller'] = (
            current.stop_latched and current.mode == 0)
        initial_instance = current.controller_instance_id
        initial_generation = current.stop_generation
        pending_reset = delayed.call_async(request(2))
        checks['reset_submitted_and_held'] = until(lambda: bool(held))
        phase, state_kind = 'new_fault_before_reset_delivery', 'stale'
        record('fault_injected', source_age_added_s=10)
        checks['stale_source_detected'] = until(
            lambda: latest.get('reason') == 'stale_source_stamp' and latest.get('fault') is True)
        query()
        end = time.monotonic()+2.
        while current.stop_generation == initial_generation and time.monotonic() < end:
            spin(.03)
            query()
        checks['new_fault_advances_controller_generation'] = (
            current.stop_generation > initial_generation)
        record('release_proxy', generation=current.stop_generation)
        held[0].set_result(None)
        checks['held_reset_reply_arrives'] = until(pending_reset.done)
        if pending_reset.done():
            response = pending_reset.result()
            checks['old_inflight_reset_cannot_release_new_stop'] = (
                not response.success and response.reason == 'stop_generation_mismatch' and
                response.stop_latched and response.mode == 0)
        else:
            checks['old_inflight_reset_cannot_release_new_stop'] = False
        phase, state_kind = 'state_recovered_still_latched', 'fresh'
        checks['fresh_state_recovers_monitor_health'] = until(
            lambda: latest.get('fault') is False and latest.get('ack') is True)
        command.publish(String(data='resume'))
        spin(.3)
        query()
        checks['legacy_monitor_resume_cannot_release_strict_latch'] = (
            current.stop_latched and current.mode == 0)
        checks['fresh_fenced_reset_accepted'] = execute(request(2)).success
        checks['fresh_fenced_mode_accepted'] = execute(request(3, mode=2)).success
        checks['fresh_fenced_path_accepted'] = execute(request(4, trajectory=path())).success
        phase = 'reference_advancing'
        checks['reference_advances_with_fresh_commands'] = until(
            lambda: len(references()) >= 3 and any(
                value['position'][0] > .02 and abs(value['velocity'][0]) > .01
                for value in references()), timeout=2.)
        old_generation = current.stop_generation
        phase, state_kind = 'fault_while_advancing', 'stale'
        record('fault_injected', source_age_added_s=10)
        checks['second_fault_detected'] = (
            until(lambda: latest.get('reason') == 'stale_source_stamp'))
        query()
        end = time.monotonic()+2.
        while current.stop_generation == old_generation and time.monotonic() < end:
            spin(.03)
            query()
        checks['fault_while_advancing_latches_pause'] = (
            current.stop_generation > old_generation and
            current.stop_latched and current.mode == 0)
        spin(.15)
        if not references():
            raise RuntimeError('reference coverage absent')
        anchor = references()[-1]['position']
        phase = 'stopped_reference'
        spin(.25)
        values = references()
        checks['protective_reference_frozen_and_zero_feedforward'] = len(values) >= 3 and all(
            math.dist(value['position'], anchor) <= 1e-6 and
            all(abs(speed) <= 1e-9 for speed in value['velocity']) for value in values)
        state_kind = 'fresh'
        checks['healthy_before_controller_outage'] = until(
            lambda: latest.get('fault') is False and latest.get('ack') is True)
        pre_restart = request(2)
        phase = 'controller_outage'
        record('controller_stopped')
        stop(controller)
        controller = None
        command.publish(String(data='pause'))
        checks['controller_outage_escalates'] = until(
            lambda: latest.get('intervention', {}).get('escalation_required') is True)
        checks['outage_unacknowledged_faulted'] = (
            latest.get('ack') is False and latest.get('fault') is True)
        controller = start_controller('controller-1')
        phase = 'controller_restarted'
        spin(.8)
        query()
        checks['restart_changes_controller_identity'] = (
            current.controller_instance_id != initial_instance)
        checks['pre_restart_reset_rejected'] = (
            execute(pre_restart).reason == 'controller_instance_mismatch')
        checks['monitor_acknowledges_new_controller_pause'] = until(
            lambda: latest.get('ack') is True and latest.get('escalation') is False and
            latest.get('intervention', {}).get('active') is False)
        checks['deadline_history_retained_after_restart_ack'] = (
            latest['intervention']['deadline_exceeded'])
        query()
        checks['restarted_controller_remains_latched'] = current.stop_latched and current.mode == 0
        completion = [event for event in events if event['kind'] == 'completion']
        result['nonfinite_completion_samples'] = sum(not event['finite'] for event in completion)
        checks['empty_buffer_completion_is_finite'] = (
            bool(completion) and all(event['finite'] for event in completion))
        result['verdict'] = 'PASS' if checks and all(checks.values()) else 'FAIL'
    except Exception as error:
        result.update(verdict='FAIL', error=repr(error))
    finally:
        for future in held:
            if not future.done():
                future.set_result(None)
        stop(controller)
        stop(monitor)
        for log in logs:
            log.close()
        result.update(checks=checks, physical_execution=False, authenticated_authority=False,
                      scope='actual monitor/controller; synthetic odometry and delayed RPC proxy',
                      controller_sha256=hashlib.sha256(controller_binary.read_bytes()).hexdigest(),
                      monitor_sha256=hashlib.sha256(monitor_binary.read_bytes()).hexdigest(),
                      harness_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        (output/'events.json').write_text(json.dumps(events, indent=2, allow_nan=False)+'\n')
        (output/'summary.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
        for subscription in subscriptions:
            node.destroy_subscription(subscription)
        node.destroy_service(service)
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, indent=2), flush=True)
    print('artifacts:', output, flush=True)
    if result['verdict'] != 'PASS':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
