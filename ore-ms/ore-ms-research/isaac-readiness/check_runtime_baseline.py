#!/usr/bin/env python3
"""Sample the running ORE-MS R1 ROS graph using subscriptions only."""

import json
import time
from collections import defaultdict

import rclpy
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from tf2_ros import Buffer, TransformListener


def ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def main():
    root = "/robot_1"
    stereo = f"{root}/sensors/front_stereo"
    state_topic = f"{root}/interface/mavros/state"
    odom_topic = f"{root}/interface/mavros/local_position/odom"
    sensors = {
        f"{stereo}/left/image_rect": Image,
        f"{stereo}/right/image_rect": Image,
        f"{stereo}/left/camera_info": CameraInfo,
        f"{stereo}/right/camera_info": CameraInfo,
        f"{stereo}/right/depth_ground_truth": Image,
        f"{root}/sensors/ouster/point_cloud": PointCloud2,
    }
    rclpy.init()
    node = rclpy.create_node("ore_ms_runtime_baseline_probe", enable_rosout=False,
                             start_parameter_services=False)
    tf = Buffer(cache_time=Duration(seconds=30))
    listener = TransformListener(tf, node)
    samples = defaultdict(list)
    clocks = []

    def record(topic, msg):
        samples[topic].append((ns(msg.header.stamp), msg.header.frame_id,
                               clocks[-1] if clocks else None, time.monotonic()))

    try:
        node.create_subscription(Clock, "/clock", lambda msg: clocks.append(ns(msg.clock)),
                                 qos_profile_sensor_data)
        state = []
        node.create_subscription(State, state_topic, state.append, qos_profile_sensor_data)
        for topic, msg_type in {odom_topic: Odometry, **sensors}.items():
            node.create_subscription(msg_type, topic, lambda msg, name=topic: record(name, msg),
                                     qos_profile_sensor_data)
        deadline = time.monotonic() + 12
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        clock_advances = len(clocks) >= 2 and clocks[-1] > clocks[0]
        report = {
            "clock_advances": clock_advances,
            "mavros_state": {"topic": state_topic, "samples": len(state),
                             "connected": bool(state and state[-1].connected),
                             "armed": state[-1].armed if state else None},
            "odometry": {"topic": odom_topic, "samples": len(samples[odom_topic])},
            "latest_tf_from_map": {
                frame: tf.can_transform("map", frame, Time())
                for frame in ("base_link", "camera_left", "camera_right", "ouster")
            },
            "sensors": {},
        }
        passed = (clock_advances and bool(state) and state[-1].connected and
                  len(samples[odom_topic]) >= 2 and all(report["latest_tf_from_map"].values()))
        for topic, msg_type in sensors.items():
            stream = samples[topic]
            result = {"samples": len(stream)}
            report["sensors"][topic] = result
            if len(stream) < 2:
                passed = False
                continue
            first, last = stream[0], stream[-1]
            stamps = [sample[0] for sample in stream]
            valid = stamps[0] > 0 and all(a < b for a, b in zip(stamps, stamps[1:]))
            result.update(frame=last[1], valid_stamps=valid,
                          hz_received=(len(stream) - 1) / (last[3] - first[3]),
                          hz_sim=(len(stream) - 1) * 1e9 / (last[0] - first[0]) if valid else None)
            passed = passed and valid
            if msg_type is CameraInfo:
                continue
            # Use an interior sample so TF can receive transforms on either side of its stamp.
            stamp, frame, clock, _ = stream[len(stream) // 2]
            exact_tf = bool(stamp > 0 and frame and
                            tf.can_transform("map", frame, Time(nanoseconds=stamp)))
            result.update(checked_stamp_ns=stamp, checked_frame=frame, exact_tf=exact_tf,
                          age_ms=(clock - stamp) / 1e6 if clock is not None else None)
            passed = passed and exact_tf and clock is not None

        left = samples[f"{stereo}/left/image_rect"]
        right = samples[f"{stereo}/right/image_rect"]
        paired = {sample[0] for sample in left} & {sample[0] for sample in right}
        report["stereo"] = {
            "exact_pairs": len(paired),
            "latest_received_delta_s": abs(left[-1][0] - right[-1][0]) / 1e9
            if left and right else None,
        }
        report["passed"] = bool(passed and paired)
        print(json.dumps(report, indent=2))
        return 0 if report["passed"] else 1
    finally:
        listener.unregister()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
