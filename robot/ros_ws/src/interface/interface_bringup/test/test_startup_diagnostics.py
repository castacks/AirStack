"""Real ROS messages distinguish frozen stamps, stopped arrivals and paused clock."""
import importlib.util
import json
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from mavros_msgs.msg import State
from std_msgs.msg import Bool

spec = importlib.util.spec_from_file_location('startup_diagnostics',
    Path(__file__).parents[1] / 'scripts/startup_diagnostics.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def test_real_ros_stale_stamps_and_arrival_loss(tmp_path):
    rclpy.init()
    output = tmp_path / 'capture.jsonl'
    observer = module.StartupDiagnostics('probe_robot', output)
    source = Node('diagnostic_fixture')
    publishers = {}
    classes = {'clock': Clock, 'raw_odometry': Odometry,
               'converted_odometry': Odometry, 'fcu_state': State,
               'state_estimate_timed_out': Bool}
    for key, topic in observer.topics.items():
        publishers[key] = source.create_publisher(classes[key], topic, 10)
    try:
        deadline = time.monotonic() + 4
        while not all(p.get_subscription_count() for p in publishers.values()):
            assert time.monotonic() < deadline
            rclpy.spin_once(observer, timeout_sec=0.02)
        for i in range(25):
            clock = Clock()
            clock.clock.sec = 200 + i
            publishers['clock'].publish(clock)
            for name in ('raw_odometry', 'converted_odometry'):
                odom = Odometry()
                odom.header.stamp.sec = 112  # Continuing data, frozen source stamp.
                odom.pose.pose.position.x = i / 1000.0
                publishers[name].publish(odom)
            state = State()
            state.connected = True
            state.armed = False
            state.mode = 'MANUAL'
            publishers['fcu_state'].publish(state)
            publishers['state_estimate_timed_out'].publish(Bool(data=True))
            end = time.monotonic() + 0.025
            while time.monotonic() < end:
                rclpy.spin_once(observer, timeout_sec=0.005)
        row = observer.snapshot()['observations']
        assert row['clock']['stamp_ns'] > 200_000_000_000
        for name in ('raw_odometry', 'converted_odometry'):
            assert row[name]['count'] >= 20
            assert row[name]['stamp_ns'] == 112_000_000_000
            assert row[name]['stamp_unchanged_wall_s'] > 0.3
            assert row[name]['arrival_age_s'] < 0.2
            assert row[name]['xyz'][0] > 0.015
        assert row['fcu_state']['connected'] and not row['fcu_state']['armed']
        assert row['state_estimate_timed_out']['value']
        # No arrivals now: independent wall observation must keep advancing.
        time.sleep(0.25)
        paused = observer.snapshot()['observations']
        assert paused['clock']['arrival_age_s'] > 0.2
        assert paused['raw_odometry']['count'] == row['raw_odometry']['count']
        assert paused['raw_odometry']['arrival_age_s'] > 0.2
        # Both reports are flushed and valid JSON while the observer is alive.
        assert len([json.loads(line) for line in output.read_text().splitlines()]) == 2
    finally:
        observer.destroy_node()
        source.destroy_node()
        rclpy.shutdown()
