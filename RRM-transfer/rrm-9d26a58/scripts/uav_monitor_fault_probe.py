#!/usr/bin/env python3
"""Measure actual state-watchdog limits on isolated DDS90; no flight commands."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import time


def main():
    if os.environ.get('ROS_DOMAIN_ID') != '90':
        raise SystemExit('requires isolated ROS_DOMAIN_ID=90')
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool
    from airstack_msgs.srv import TrajectoryMode

    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--output', required=True, type=Path)
    args = ap.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    node = Node('uav_monitor_fault_probe')
    events = []
    phase = 'startup'

    def record(kind, **values):
        events.append(dict(wall_unix=time.time(), monotonic=time.monotonic(),
                           phase=phase, kind=kind, **values))

    def mode(req, res):
        res.success = False
        record('mode_request', mode=req.mode, success=res.success)
        return res

    service = node.create_service(TrajectoryMode, '/fault_probe/mode', mode)
    subscription = node.create_subscription(
        Bool, '/fault_probe/timedout', lambda m: record('timeout', value=m.data), 10)
    publisher = node.create_publisher(Odometry, '/fault_probe/state', 10)
    log = (args.output / 'monitor.log').open('w')
    process = subprocess.Popen([
        'ros2', 'run', 'drone_safety_monitor', 'drone_safety_monitor', '--ros-args',
        '-p', 'use_sim_time:=false', '-p', 'state_estimate_timeout:=1.0',
        '-r', '__ns:=/fault_probe', '-r', 'state_estimate:=/fault_probe/state',
        '-r', 'state_estimate_timed_out:=/fault_probe/timedout',
        '-r', 'set_trajectory_mode:=/fault_probe/mode'], stdout=log, stderr=subprocess.STDOUT)
    result = {}
    try:
        for name, publish, age in [('startup', False, 0), ('fresh', True, 0),
                                   ('stale_source', True, 10), ('silence_rejected', False, 0),
                                   ('fresh_recovery', True, 0), ('second_silence', False, 0)]:
            phase = name
            record('phase_start', publish=publish, source_age_s=age)
            end = time.monotonic() + 3
            while time.monotonic() < end:
                if publish:
                    msg = Odometry()
                    msg.header.stamp = rclpy.time.Time(
                        nanoseconds=node.get_clock().now().nanoseconds - int(age * 1e9)).to_msg()
                    publisher.publish(msg)
                rclpy.spin_once(node, timeout_sec=.03)
            if process.poll() is not None:
                raise RuntimeError('monitor exited: ' + str(process.returncode))

        def tail(name, kind):
            selected = [e for e in events if e['phase'] == name]
            start = selected[0]['monotonic']
            return [e for e in selected if e['kind'] == kind and e['monotonic'] >= start + 1.5]

        checks = dict(
            fresh_healthy=bool(tail('fresh', 'timeout')) and all(not e['value'] for e in tail('fresh', 'timeout')),
            silence_detected=any(e['value'] for e in tail('silence_rejected', 'timeout')),
            pause_requested=any(e['phase'] == 'silence_rejected' and e['kind'] == 'mode_request' and e['mode'] == 0 for e in events),
            resumed_receipts_clear=bool(tail('fresh_recovery', 'timeout')) and all(not e['value'] for e in tail('fresh_recovery', 'timeout')),
            second_silence_rearms=any(e['phase'] == 'second_silence' and e['kind'] == 'mode_request' and e['mode'] == 0 for e in events))
        stale = tail('stale_source', 'timeout')
        rejected = [e for e in events if e['phase'] == 'silence_rejected' and e['kind'] == 'mode_request']
        result = dict(
            verdict='PASS' if all(checks.values()) else 'FAIL',
            scope='receive-silence detection and request only, actual C++ binary with rejecting mock service',
            checks=checks,
            source_freshness=dict(verdict='NOT DEMONSTRATED', injected_age_s=10,
                                  timeout_flags=[e['value'] for e in stale],
                                  detected=any(e['value'] for e in stale)),
            rejected_intervention=dict(verdict='NOT DEMONSTRATED', requests=len(rejected),
                                       accepted=False, physical_recovery_tested=False),
            script_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest())
    finally:
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        log.close()
        (args.output / 'events.json').write_text(json.dumps(events, indent=2) + '\n')
        (args.output / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(result, indent=2))
    if result.get('verdict') != 'PASS':
        raise SystemExit(1)


if __name__ == '__main__':
    main()
