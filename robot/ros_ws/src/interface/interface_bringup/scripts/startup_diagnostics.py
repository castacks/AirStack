#!/usr/bin/env python3
"""Independent read-only wall-time observations; never repairs/restamps telemetry."""
import json
import math
import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


class StartupDiagnostics(Node):
    def __init__(self, robot, output):
        # This independent node must keep reporting when simulation time stops.
        super().__init__('startup_diagnostics', parameter_overrides=[])
        self.output = Path(output).open('a', buffering=1)
        self.observed = {}
        self.start = time.monotonic()
        self.subscriptions_kept = []
        topics = [
            ('clock', Clock, '/clock'),
            ('raw_odometry', Odometry, f'/{robot}/interface/mavros/local_position/odom'),
            ('converted_odometry', Odometry, f'/{robot}/odometry_conversion/odometry'),
            ('fcu_state', State, f'/{robot}/interface/mavros/state'),
            ('state_estimate_timed_out', Bool,
             f'/{robot}/behavior/drone_safety_monitor/state_estimate_timed_out'),
        ]
        for name, cls, topic in topics:
            self.subscriptions_kept.append(self.create_subscription(
                cls, topic, lambda msg, key=name: self.observe(key, msg),
                qos_profile_sensor_data))
        self.topics = {name: topic for name, _, topic in topics}

    def observe(self, key, msg):
        now = time.monotonic()
        previous = self.observed.get(key, {})
        entry = {'count': previous.get('count', 0) + 1, 'received': now}
        if key == 'clock' or key.endswith('odometry'):
            ns = stamp_ns(msg.clock if key == 'clock' else msg.header.stamp)
            entry['stamp_ns'] = ns
            entry['stamp_changed_at'] = (previous['stamp_changed_at']
                if previous.get('stamp_ns') == ns else now)
            if key.endswith('odometry'):
                p = msg.pose.pose.position
                # Preserve non-finite telemetry explicitly without losing the
                # rest of this diagnostic stream to JSON serialization errors.
                entry['xyz'] = [v if math.isfinite(v) else str(v)
                                for v in (p.x, p.y, p.z)]
        elif key == 'fcu_state':
            entry.update(connected=msg.connected, armed=msg.armed, mode=msg.mode)
        else:
            entry['value'] = msg.data
        self.observed[key] = entry

    def snapshot(self):
        now = time.monotonic()
        values = {}
        for key, entry in self.observed.items():
            values[key] = {k: v for k, v in entry.items()
                           if k not in ('received', 'stamp_changed_at')}
            values[key]['arrival_age_s'] = now - entry['received']
            if 'stamp_changed_at' in entry:
                values[key]['stamp_unchanged_wall_s'] = now - entry['stamp_changed_at']
        row = {'wall_time_unix_s': time.time(), 'elapsed_wall_s': now - self.start,
               'topics': self.topics, 'observations': values}
        self.output.write(json.dumps(row, allow_nan=False) + '\n')
        return row

    def destroy_node(self):
        self.output.close()
        return super().destroy_node()


def main():
    rclpy.init()
    robot = os.environ['ROBOT_NAME']
    node = StartupDiagnostics(robot, f'/tmp/raven_startup_{robot}.jsonl')
    next_report = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if time.monotonic() >= next_report:
                node.snapshot()
                next_report = time.monotonic() + 1.0
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
