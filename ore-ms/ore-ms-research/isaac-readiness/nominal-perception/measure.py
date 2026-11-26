#!/usr/bin/env python3
"""Observe nominal R1 telemetry and emit a single JSON result."""

import argparse
import json
import math
import os
import statistics
import time
from bisect import bisect_left
from collections import defaultdict
from datetime import datetime, timezone

import numpy as np

MAX_MESSAGES = 4096
MAX_PAYLOAD_VALUES = 4096
WARMUP_S = 4
DRAIN_S = 2


def stamp_ns(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def distribution(values):
    if not values:
        return None
    ordered = sorted(values)
    return {
        "min": ordered[0], "median": statistics.median(ordered),
        "p95": ordered[math.ceil(0.95 * len(ordered)) - 1], "max": ordered[-1],
    }


def timing(stamps, arrivals):
    valid = len(stamps) >= 2 and stamps[0] > 0 and all(
        a < b for a, b in zip(stamps, stamps[1:])
    )
    result = {"strict_positive_stamps": valid}
    for key, values in (("wall", arrivals), ("sim", stamps if valid else [])):
        intervals = [(b - a) / 1e6 for a, b in zip(values, values[1:])]
        typical = statistics.median(intervals) if intervals else None
        result[key] = {
            "hz": 1000 * len(intervals) / sum(intervals) if sum(intervals) > 0 else None,
            "interval_ms": distribution(intervals),
            "jitter_ms_population_stddev": statistics.pstdev(intervals) if intervals else None,
            "gaps_over_1_5_median": sum(gap > 1.5 * typical for gap in intervals)
            if typical else None,
        }
    return result


def pairing(left, right):
    left, right = sorted(set(left)), sorted(set(right))
    common = set(left).intersection(right)
    nearest = []
    for stamp in left:
        if right:
            index = bisect_left(right, stamp)
            nearest.append(min(abs(stamp - other) for other in right[max(0, index - 1):index + 1]) / 1e6)
    return {
        "exact_pairs": len(common), "left_unmatched_unique": len(left) - len(common),
        "right_unmatched_unique": len(right) - len(common),
        "left_to_nearest_right_abs_ms": distribution(nearest),
    }


def sampled_grid(msg, dtype, pixel_step, offset=0):
    """Read an evenly strided subset from a row-padded ROS payload."""
    dtype = np.dtype(dtype)
    row_step = msg.row_step if hasattr(msg, "row_step") else msg.step
    if (min(msg.width, msg.height, pixel_step) <= 0 or
            offset < 0 or offset + dtype.itemsize > pixel_step or
            row_step < msg.width * pixel_step or len(msg.data) < row_step * msg.height):
        raise ValueError("invalid payload dimensions or strides")
    grid = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                      offset=offset, strides=(row_step, pixel_step))
    step = math.ceil(grid.size / MAX_PAYLOAD_VALUES)
    indices = np.arange(0, grid.size, step)
    return grid[indices // msg.width, indices % msg.width]


def payload(kind, msg):
    if kind == "cloud":
        fields = {field.name: field for field in msg.fields}
        if not all(axis in fields and fields[axis].count == 1 and
                   fields[axis].datatype in (7, 8) for axis in "xyz"):
            raise ValueError("missing FLOAT32/FLOAT64 XYZ fields")
        endian = ">" if msg.is_bigendian else "<"
        xyz = [sampled_grid(msg, endian + ("f4" if fields[axis].datatype == 7 else "f8"),
                            msg.point_step, fields[axis].offset) for axis in "xyz"]
        valid = np.logical_and.reduce([np.isfinite(axis) for axis in xyz])
        return {"declared_points": msg.width * msg.height, "sampled_values": len(valid),
                "valid_fraction": float(np.mean(valid))}
    disparity = kind == "disparity"
    image = msg.image if disparity else msg
    endian = ">" if image.is_bigendian else "<"
    if image.encoding == "32FC1":
        values = sampled_grid(image, endian + "f4", 4)
    elif kind == "depth" and image.encoding == "16UC1":
        values = sampled_grid(image, endian + "u2", 2)
    else:
        raise ValueError(f"unsupported encoding {image.encoding!r}")
    valid = np.isfinite(values) & (values > 0)
    if disparity:
        if not math.isfinite(msg.min_disparity) or not math.isfinite(msg.max_disparity):
            raise ValueError("nonfinite disparity bounds")
        valid &= (values >= msg.min_disparity) & (values <= msg.max_disparity)
    return {"encoding": image.encoding, "width": image.width, "height": image.height,
            "sampled_values": len(values), "valid_fraction": float(np.mean(valid))}


def observe(args):
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.duration import Duration
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.time import Time
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import CameraInfo, Image, PointCloud2
    from stereo_msgs.msg import DisparityImage
    from tf2_ros import Buffer, TransformListener
    from visualization_msgs.msg import Marker

    root = "/robot_1"
    stereo = root + "/sensors/front_stereo/"
    perception = root + "/perception/stereo_image_proc/"
    required = {
        "left_rgb": (stereo + "left/image_rect", Image, None),
        "right_rgb": (stereo + "right/image_rect", Image, None),
        "left_info": (stereo + "left/camera_info", CameraInfo, None),
        "right_info": (stereo + "right/camera_info", CameraInfo, None),
        "right_depth": (stereo + "right/depth_ground_truth", Image, "depth"),
        "ouster": (root + "/sensors/ouster/point_cloud", PointCloud2, "cloud"),
    }
    optional = {
        "disparity": (perception + "disparity", DisparityImage, "disparity"),
        "stereo_cloud": (perception + "point_cloud", PointCloud2, "cloud"),
        "odometry": (root + "/odometry_conversion/odometry", Odometry, None),
        "vdb_markers": (root + "/vdb_mapping/vdb_map_visualization", Marker, None),
    }
    specs = {**required, **optional}
    tf_streams = ("left_rgb", "right_rgb", "right_depth", "ouster")
    rclpy.init()
    node = rclpy.create_node("ore_ms_nominal_perception_probe", enable_rosout=False,
                             start_parameter_services=False)
    tf = Buffer(cache_time=Duration(seconds=args.seconds + WARMUP_S + DRAIN_S + 5))
    listener = TransformListener(tf, node)
    records, payloads, errors = defaultdict(list), defaultdict(list), defaultdict(list)
    dropped = defaultdict(int)
    clock = None
    clock_samples = []
    collecting = False

    def receive_clock(msg):
        nonlocal clock
        clock = stamp_ns(msg.clock)
        if collecting and len(clock_samples) < MAX_MESSAGES:
            clock_samples.append(clock)
        elif collecting:
            dropped["clock"] += 1

    def receive(key, kind, msg):
        arrival = time.monotonic_ns()
        if not collecting:
            return
        stream = records[key]
        if len(stream) >= MAX_MESSAGES:
            dropped[key] += 1
            return
        header = getattr(msg, "header", None)
        stream.append((stamp_ns(header.stamp) if header else None,
                       header.frame_id if header else None, clock, arrival))
        if kind and (len(stream) - 1) % 10 == 0:
            try:
                payloads[key].append({"stamp_ns": stream[-1][0], **payload(kind, msg)})
            except (ValueError, TypeError, BufferError) as exc:
                errors[key].append(str(exc))

    def spin_for(seconds):
        deadline = time.monotonic() + seconds
        while (remaining := deadline - time.monotonic()) > 0:
            rclpy.spin_once(node, timeout_sec=min(0.1, remaining))

    try:
        node.create_subscription(Clock, "/clock", receive_clock, qos_profile_sensor_data)
        for key, (topic, msg_type, kind) in specs.items():
            node.create_subscription(msg_type, topic, lambda msg, key=key, kind=kind:
                                     receive(key, kind, msg), qos_profile_sensor_data)
        spin_for(WARMUP_S)
        started = datetime.now(timezone.utc).isoformat()
        collecting = True
        spin_for(args.seconds)
        collecting = False
        spin_for(DRAIN_S)

        failures = []
        clock_ok = (len(clock_samples) >= 2 and clock_samples[-1] > clock_samples[0] and
                    all(a <= b for a, b in zip(clock_samples, clock_samples[1:])))
        if not clock_ok:
            failures.append("simulation clock did not advance monotonically")
        streams = {}
        for key, (topic, _, kind) in specs.items():
            stream = records[key]
            stamps = [sample[0] for sample in stream if sample[0] is not None]
            ages = [(clock_at_receive - stamp) / 1e6 for stamp, _, clock_at_receive, _ in stream
                    if stamp is not None and stamp > 0 and clock_at_receive is not None]
            result = {"topic": topic, "required": key in required, "messages": len(stream),
                      "publisher_count_end": node.count_publishers(topic),
                      "received": bool(stream),
                      "frames": sorted({frame for _, frame, _, _ in stream if frame}),
                      "timing": timing(stamps, [sample[3] for sample in stream]),
                      "clock_age_ms": distribution(ages), "clock_age_samples": len(ages),
                      "collector_overflow": dropped[key]}
            streams[key] = result
            if key in required and (not result["timing"]["strict_positive_stamps"] or not ages):
                failures.append(f"{key}: missing stream, invalid stamps or unavailable clock age")
            if kind:
                result["payload_samples"] = payloads[key]
                result["payload_errors"] = errors[key]
                result["valid_fraction"] = distribution(
                    [item["valid_fraction"] for item in payloads[key]])
                if kind == "cloud":
                    result["declared_points"] = distribution(
                        [item["declared_points"] for item in payloads[key]])
                if key in required and (errors[key] or not any(
                        item["valid_fraction"] > 0 for item in payloads[key])):
                    failures.append(f"{key}: unreadable payload or no valid sampled values")
            if key in tf_streams:
                # Exclude window edges to give dynamic TF a chance to bracket each stamp.
                interior = stream[len(stream) // 10:len(stream) - len(stream) // 10]
                interval = max(1, math.ceil(len(interior) / 12))
                checks = [
                    {"stamp_ns": stamp, "frame": frame, "available": bool(
                        stamp and stamp > 0 and frame and
                        tf.can_transform("map", frame, Time(nanoseconds=stamp)))}
                    for stamp, frame, _, _ in interior[::interval]
                ]
                result["map_tf"] = checks
                if not checks or not all(item["available"] for item in checks):
                    failures.append(f"{key}: timestamp-specific map transform unavailable")
        stereo_pairing = pairing(
            *([stamp for stamp, *_ in records[key] if stamp is not None and stamp > 0]
              for key in ("left_rgb", "right_rgb")))
        if stereo_pairing["exact_pairs"] == 0:
            failures.append("no exact left/right RGB stamp pairs")
        overflow = any(dropped.values())
        return {
            "schema_version": 1, "started_utc": started,
            "configuration": {"robot": root, "domain": os.environ.get("ROS_DOMAIN_ID"),
                              "source_commit": args.source_commit, "seconds": args.seconds,
                              "warmup_seconds": WARMUP_S, "drain_seconds": DRAIN_S,
                              "qos": "sensor_data", "max_messages_per_stream": MAX_MESSAGES,
                              "payload_frame_stride": 10, "max_payload_values": MAX_PAYLOAD_VALUES},
            "clock": {"monotonic_advancing": clock_ok, "samples": len(clock_samples),
                      "collector_overflow": dropped["clock"]},
            "streams": streams, "stereo_pairing": stereo_pairing, "failures": failures,
            "status": "inconclusive" if overflow else "fail" if failures else "pass",
        }
    finally:
        listener.unregister()
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=int, choices=range(5, 61), metavar="5..60", default=30)
    parser.add_argument("--source-commit", required=True, help="host checkout revision recorded by operator")
    result = observe(parser.parse_args())
    print(json.dumps(result, indent=2, allow_nan=False))
    return {"pass": 0, "fail": 1, "inconclusive": 2}[result["status"]]


if __name__ == "__main__":
    raise SystemExit(main())
