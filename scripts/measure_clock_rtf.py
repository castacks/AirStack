#!/usr/bin/env python3
"""Measure simulation RTF from paired callback timestamps in one ROS process.

Run inside a sourced ROS container on the domain carrying /clock. Discovery
and process startup are excluded; all elapsed times use the monotonic clock.
"""
import argparse
import json
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=90)
    parser.add_argument("--warmup", type=float, default=10)
    parser.add_argument("--timeout", type=float, default=2400)
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("benchmark_clock_rtf")
    started = time.monotonic()
    first = baseline = previous = None
    done = False

    def callback(msg):
        nonlocal first, baseline, previous, done
        wall = time.monotonic()
        sim = msg.clock.sec + msg.clock.nanosec * 1e-9
        if previous is not None and sim < previous:
            first = baseline = None
        previous = sim
        if first is None:
            first = wall
        if wall - first < args.warmup:
            return
        if baseline is None:
            baseline = (wall, sim)
        elapsed = wall - baseline[0]
        if elapsed >= args.seconds:
            print(json.dumps({"wall_seconds": elapsed,
                              "sim_seconds": sim - baseline[1],
                              "rtf": (sim - baseline[1]) / elapsed,
                              "sim_start": baseline[1], "sim_end": sim,
                              "source": "persistent_clock_callback"}), flush=True)
            done = True

    subscription = node.create_subscription(
        Clock, "/clock", callback, qos_profile_sensor_data)
    try:
        while not done and time.monotonic() - started < args.timeout:
            rclpy.spin_once(node, timeout_sec=0.5)
        if not done:
            raise SystemExit("Clock measurement timed out without a full sample")
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
