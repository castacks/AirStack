#!/usr/bin/env python3
# Copyright (c) 2026 Carnegie Mellon University
# SPDX-License-Identifier: BSD-3-Clause-Clear
"""
Actual monitor contract on DDS197, with controlled clock and mock controller.

No simulator/aircraft endpoints. Logs and summary go to WATCHDOG_TEST_OUTPUT,
which must be new, or to a temporary directory for CTest.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    """Run source-clock and controller-acknowledgement scenarios in isolation."""
    if os.environ.get('ROS_DOMAIN_ID') != '197':
        raise SystemExit('requires isolated ROS_DOMAIN_ID=197')
    import rclpy
    from rclpy.node import Node
    from rclpy.callback_groups import ReentrantCallbackGroup
    from rclpy.task import Future
    from nav_msgs.msg import Odometry
    from rosgraph_msgs.msg import Clock
    from std_msgs.msg import Bool, String
    from airstack_msgs.srv import TrajectoryMode

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', type=Path)
    args = ap.parse_args()
    target = args.output or os.environ.get('WATCHDOG_TEST_OUTPUT')
    if target:
        out = Path(target)
        out.mkdir(parents=True, exist_ok=False)
    else:
        out = Path(tempfile.mkdtemp(prefix='airstack-watchdog-'))
    rclpy.init()
    node = Node('isolated_watchdog_contract')
    events = []
    checks = {}
    phase = 'startup'
    policy = 'accept'
    sim = 100.0
    previous_wall = time.monotonic()
    held = []
    fixed_stamp = None

    def record(kind, **kwargs):
        events.append(dict(phase=phase, monotonic=time.monotonic(), kind=kind, **kwargs))

    async def mode(request, response):
        local_policy = policy
        record('mode', mode=request.mode, policy=local_policy)
        if local_policy == 'hold':
            future = Future()
            held.append((future, time.monotonic()))
            await future
        response.success = local_policy != 'reject'
        record('reply', success=response.success)
        return response

    service_group = ReentrantCallbackGroup()
    service = node.create_service(TrajectoryMode, '/watchdog_contract/mode', mode,
                                  callback_group=service_group)
    subs = [node.create_subscription(Bool, '/watchdog_contract/timeout',
                                     lambda m: record('timeout', value=m.data), 10),
            node.create_subscription(Bool, '/watchdog_contract/ack',
                                     lambda m: record('ack', value=m.data), 10),
            node.create_subscription(String, '/watchdog_contract/reason',
                                     lambda m: record('reason', value=m.data), 10)]
    clock = node.create_publisher(Clock, '/clock', 10)
    state = node.create_publisher(Odometry, '/watchdog_contract/state', 10)
    command = node.create_publisher(String, '/watchdog_contract/command', 10)
    log = (out / 'monitor.log').open('w')
    process = subprocess.Popen([
        'ros2', 'run', 'drone_safety_monitor', 'drone_safety_monitor', '--ros-args',
        '-p', 'use_sim_time:=true', '-r', '__ns:=/watchdog_contract',
        '-r', 'state_estimate:=/watchdog_contract/state',
        '-r', 'command:=/watchdog_contract/command',
        '-r', 'state_estimate_timed_out:=/watchdog_contract/timeout',
        '-r', 'pause_acknowledged:=/watchdog_contract/ack',
        '-r', 'state_health_reason:=/watchdog_contract/reason',
        '-r', 'set_trajectory_mode:=/watchdog_contract/mode'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)

    def select(kind, name=None, after=0):
        return [e for e in events if e['phase'] == (name or phase) and
                e['kind'] == kind and e['monotonic'] >= after]

    def spin(name, seconds, kind='fresh', advance=True, operator=None):
        nonlocal phase, sim, previous_wall, fixed_stamp
        phase = name
        start = time.monotonic()
        record('phase_start')
        sent = False
        if kind == 'backwards':
            fixed_stamp = sim - .2
        if kind == 'duplicate':
            fixed_stamp = sim
        while time.monotonic() - start < seconds:
            now = time.monotonic()
            if advance:
                sim += now - previous_wall
            previous_wall = now
            cm = Clock()
            cm.clock.sec = int(sim)
            cm.clock.nanosec = int((sim % 1) * 1e9)
            clock.publish(cm)
            if kind != 'silent':
                msg = Odometry()
                source = sim
                if kind == 'stale':
                    source -= 10
                if kind == 'future':
                    source += 2
                if kind in ('backwards', 'duplicate'):
                    source = fixed_stamp
                msg.header.stamp.sec = int(source)
                msg.header.stamp.nanosec = int((source % 1) * 1e9)
                msg.pose.pose.orientation.w = 1.0
                if kind == 'nan':
                    msg.twist.twist.linear.x = math.nan
                if kind == 'zero_stamp':
                    msg.header.stamp.sec = 0
                    msg.header.stamp.nanosec = 0
                state.publish(msg)
            if operator and not sent and now-start >= .3:
                msg = String()
                msg.data = operator
                command.publish(msg)
                sent = True
                record('command', value=operator)
            rclpy.spin_once(node, timeout_sec=.02)
            if process.poll() is not None:
                raise RuntimeError('monitor exited: ' + str(process.returncode))
        return start

    def expect(name, condition):
        checks[name] = bool(condition)
        if not condition:
            raise AssertionError(name)

    result = {}
    try:
        spin('startup', 2)
        start = spin('healthy', 1.5)
        expect('fresh_healthy', bool(select('timeout', after=start+.7)) and
               all(not e['value'] for e in select('timeout', after=start+.7)))
        expect('startup_pause_acknowledged', any(e['value'] for e in select('ack')))
        for name, kind, reason in [('stale', 'stale', 'stale_source_stamp'),
                                   ('future', 'future', 'future_source_stamp'),
                                   ('backwards', 'backwards', 'backwards_source_stamp'),
                                   ('nonfinite', 'nan', 'nonfinite_state'),
                                   ('zero_stamp', 'zero_stamp', 'invalid_source_stamp')]:
            spin('recover_'+name, .7)
            start = spin(name, 1, kind, operator='resume')
            expect(name+'_detected', any(e['value'] for e in select('timeout')))
            expect(name+'_reason', any(e['value'] == reason for e in select('reason')))
            expect(name+'_pause', any(e['mode'] == 0 for e in select('mode')))
            expect(name+'_resume_blocked', not any(e['mode'] == 1 for e in select('mode')))
        spin('recover_silence', .7)
        start = spin('silence', 1.8, 'silent')
        expect('silence_detected',
               any(e['value'] for e in select('timeout', after=start+1.3)))
        spin('recover_frozen', .7)
        start = spin('frozen_clock_duplicates', 2.8, 'duplicate', advance=False)
        expect('frozen_clock_detected',
               any(e['value'] for e in select('timeout', after=start+2.2)))
        expect('wall_heartbeat_continues', len(select('timeout', after=start+.3)) >= 8)
        spin('recover_reset', .7)
        sim -= 50
        spin('clock_reset', .6, 'silent')
        expect('clock_reset_detected', any(e['value'] == 'clock_reset' for e in select('reason')))
        start = spin('reset_recovery', 1)
        expect('new_epoch_recovers',
               any(not e['value'] for e in select('timeout', after=start+.4)))
        # Reject responses while supplying fresh state: recovery alone cannot clear PAUSE.
        policy = 'reject'
        spin('rejected', 3.3, operator='pause')
        requests = select('mode')
        expect('rejected_retries', 2 <= len(requests) <= 4)
        expect('retries_bounded', all(
            b['monotonic']-a['monotonic'] >= .9 for a, b in zip(requests, requests[1:])))
        expect('rejection_not_acknowledged', all(not e['value'] for e in select('ack')[-4:]))
        expect('rejection_blocks_legacy_readiness',
               all(e['value'] for e in select('timeout')[-4:]))
        spin('pending_resume', .8, operator='resume')
        expect('pending_resume_blocked', not any(e['mode'] == 1 for e in select('mode')))
        policy = 'accept'
        spin('accepted', 1.6)
        expect('positive_reply_required', any(e['value'] for e in select('ack')))
        spin('manual_resume', .8, operator='resume')
        expect('healthy_explicit_resume', any(e['mode'] == 1 for e in select('mode')))
        spin('manual_rewind', .8, operator='rewind')
        expect('healthy_explicit_rewind', any(e['mode'] == 4 for e in select('mode')))
        # Server callbacks stay pending while the client deadline expires and retries.
        policy = 'hold'
        spin('held_replies', 2.6, operator='pause')
        expect('unanswered_retries', 2 <= len(select('mode')) <= 3)
        expect('unanswered_heartbeat', len(select('timeout')) >= 8)
        expect('unanswered_not_acknowledged', all(not e['value'] for e in select('ack')[-4:]))
        # Old positive replies must not acknowledge the currently pending generation.
        remaining = []
        for future, receipt in held:
            if time.monotonic()-receipt > 1.0:
                future.set_result(None)
            else:
                remaining.append((future, receipt))
        held[:] = remaining
        policy = 'reject'
        spin('late_reply', .4)
        expect('late_reply_not_acknowledged', not any(e['value'] for e in select('ack')))
        node.destroy_service(service)
        service = None
        spin('unavailable', 2.3)
        expect('unavailable_heartbeat', len(select('timeout')) >= 7)
        expect('unavailable_not_acknowledged', not any(e['value'] for e in select('ack')))
        policy = 'accept'
        service = node.create_service(TrajectoryMode, '/watchdog_contract/mode', mode,
                                      callback_group=service_group)
        spin('service_recovered', 2)
        expect('service_recovery_ack', any(e['value'] for e in select('ack')))
        result['verdict'] = 'PASS'
    except Exception as exc:
        result['verdict'] = 'FAIL'
        result['error'] = repr(exc)
    finally:
        result['checks'] = checks
        result['scope'] = ('actual C++ binary, controlled ROS clock, mock controller; '
                           'no physical stop qualification')
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
        log.close()
        (out/'events.json').write_text(json.dumps(events, indent=2)+'\n')
        (out/'summary.json').write_text(json.dumps(result, indent=2)+'\n')
        for subscription in subs:
            node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, indent=2), flush=True)
    print('artifacts:', out, flush=True)
    if result['verdict'] != 'PASS':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
