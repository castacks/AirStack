#!/usr/bin/env python3
# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""
Actual-monitor PAUSE priority with held operator RPCs, isolated DDS197 only.

No simulator endpoints. A mock controller deliberately applies held operator
requests late; this exercises local reconciliation, not exclusive mode authority.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    """Measure priority and readiness without a live vehicle or controller."""
    if os.environ.get('ROS_DOMAIN_ID') != '197':
        raise SystemExit('requires isolated ROS_DOMAIN_ID=197')
    import rclpy
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.node import Node
    from rclpy.task import Future
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String
    from airstack_msgs.srv import TrajectoryMode

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    parser.add_argument('--executable', type=Path, required=True)
    args = parser.parse_args()
    output = args.output or Path(tempfile.mkdtemp(prefix='operator-preemption-'))/'run'
    output.mkdir(parents=True, exist_ok=False)
    executable = args.executable.resolve(strict=True)
    rclpy.init()
    node = Node('isolated_operator_preemption')
    events, held, checks = [], [], {}
    phase = 'startup'
    hold_operator = False
    state_kind = 'fresh'

    def record(kind, **fields):
        events.append(dict(kind=kind, phase=phase, monotonic=time.monotonic(), **fields))

    async def mode(request, response):
        record('request', mode=request.mode)
        if request.mode in (1, 4) and hold_operator:
            future = Future()
            held.append(future)
            await future
        record('applied', mode=request.mode)
        response.success = True
        return response

    service = node.create_service(TrajectoryMode, '/operator_contract/mode', mode,
                                  callback_group=ReentrantCallbackGroup())
    state = node.create_publisher(Odometry, '/operator_contract/state', 10)
    command = node.create_publisher(String, '/operator_contract/command', 10)
    subs = [node.create_subscription(Bool, '/operator_contract/ack',
                                     lambda m: record('ack', value=m.data), 10),
            node.create_subscription(Bool, '/operator_contract/timeout',
                                     lambda m: record('timeout', value=m.data), 10),
            node.create_subscription(String, '/operator_contract/reason',
                                     lambda m: record('reason', value=m.data), 10)]
    log = (output/'monitor.log').open('w')
    process = subprocess.Popen([
        str(executable), '--ros-args', '-r', '__ns:=/operator_contract',
        '-r', 'state_estimate:=/operator_contract/state',
        '-r', 'command:=/operator_contract/command',
        '-r', 'pause_acknowledged:=/operator_contract/ack',
        '-r', 'state_estimate_timed_out:=/operator_contract/timeout',
        '-r', 'state_health_reason:=/operator_contract/reason',
        '-r', 'set_trajectory_mode:=/operator_contract/mode'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def spin(duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            msg = Odometry()
            stamp = node.get_clock().now().nanoseconds
            if state_kind == 'stale':
                stamp -= 10_000_000_000
            msg.header.stamp.sec, msg.header.stamp.nanosec = divmod(stamp, 1_000_000_000)
            msg.pose.pose.orientation.w = 1.
            state.publish(msg)
            rclpy.spin_once(node, timeout_sec=.01)
            if process.poll() is not None:
                raise RuntimeError('monitor exited: '+str(process.returncode))

    def send(value):
        msg = String()
        msg.data = value
        command.publish(msg)
        record('command', value=value)

    def selected(kind, start, **fields):
        return [event for event in events if event['kind'] == kind and
                event['monotonic'] >= start and
                all(event.get(key) == value for key, value in fields.items())]

    result = {}
    try:
        spin(2.5)
        checks['startup_positive_pause'] = any(e['kind'] == 'ack' and e['value'] for e in events)
        for operator, trigger in [('resume', 'pause'), ('rewind', 'pause'),
                                  ('resume', 'stale'), ('resume', 'timeout')]:
            phase = operator+'_'+trigger
            state_kind = 'fresh'
            hold_operator = False
            spin(1.)
            send('pause')
            spin(.4)
            hold_operator = True
            start = time.monotonic()
            send(operator)
            deadline = start+1.
            while not held and time.monotonic() < deadline:
                spin(.01)
            checks[phase+'_operator_received'] = bool(held)
            if not held:
                continue
            intervention = time.monotonic()
            record('intervention', trigger=trigger)
            if trigger == 'pause':
                send('pause')
            elif trigger == 'stale':
                state_kind = 'stale'
            spin(1.1 if trigger == 'timeout' else .4)
            pauses = selected('request', intervention, mode=0)
            checks[phase+'_pause_before_operator_reply'] = bool(pauses)
            record('pause_priority_measurement',
                   first_pause_delay_s=pauses[0]['monotonic']-intervention if pauses else None)
            # Sample readiness after the observed PAUSE, not before the wall
            # timer has had its allowed opportunity to detect operator timeout.
            spin(.3)
            # Explicit intervention must not claim readiness while its superseded
            # operator request can still apply in the mock server.
            readiness_after = pauses[0]['monotonic']+.05 if pauses else intervention+.3
            readiness = selected('timeout', readiness_after)
            checks[phase+'_unresolved_operator_blocks_readiness'] = (
                bool(readiness) and all(event['value'] for event in readiness))
            record('release_operator')
            hold_operator = False
            for future in held:
                future.set_result(None)
            held.clear()
            spin(.6)
            applied = selected('applied', intervention, mode=1 if operator == 'resume' else 4)
            if applied:
                after = applied[-1]['monotonic']
                checks[phase+'_pause_after_late_operator_apply'] = bool(
                    selected('applied', after, mode=0))
            else:
                checks[phase+'_pause_after_late_operator_apply'] = False
            state_kind = 'fresh'
            spin(1.2)
        result['verdict'] = 'PASS' if all(checks.values()) else 'FAIL'
    except Exception as exc:
        result.update(verdict='FAIL', error=repr(exc))
    finally:
        result.update(checks=checks,
                      scope=('actual binary, mock controller; '
                             'no physical stop or exclusive authority'),
                      executable_sha256=hashlib.sha256(executable.read_bytes()).hexdigest(),
                      harness_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
        for future in held:
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
        for subscription in subs:
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
