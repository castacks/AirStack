#!/usr/bin/env python3
"""Bounded subscription-only control evidence capture; never dispatches a command."""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time


def source_stamp_ns(payload):
    stamp = payload.get("header", {}).get("stamp")
    if not isinstance(stamp, dict):
        return None
    if type(stamp.get("sec")) is not int or type(stamp.get("nanosec")) is not int:
        return None
    return stamp["sec"] * 1_000_000_000 + stamp["nanosec"]


class Capture:
    def __init__(self, stream, channels, start, max_events=100_000):
        self.stream, self.channels, self.start = stream, channels, start
        self.max_events = max_events
        self.counts = Counter()
        self.first, self.last, self.gaps = {}, {}, {}
        self.reasons = Counter()
        self.invalid_diagnostics = self.sequence_discontinuities = 0
        self.previous_sequence = None
        self.total = 0

    def record(self, channel, payload, received, ros_now_ns):
        if self.total >= self.max_events:
            return False
        event = {"channel": channel, "receipt_monotonic_s": received,
                 "elapsed_s": received - self.start, "ros_now_ns": ros_now_ns,
                 "source_stamp_ns": source_stamp_ns(payload), "message": payload}
        if channel == "admission":
            try:
                diagnostic = json.loads(payload["data"])
                sequence, mask = diagnostic["sequence"], diagnostic["reason_mask"]
                if (diagnostic.get("schema") != "pid-admission/v1"
                        or type(sequence) is not int or sequence < 0
                        or type(mask) is not int or mask < 0):
                    raise ValueError("invalid diagnostic schema/fields")
                event["diagnostic"] = diagnostic
                self.reasons[str(mask)] += 1
                if self.previous_sequence is not None and sequence != self.previous_sequence + 1:
                    self.sequence_discontinuities += 1
                self.previous_sequence = sequence
            except (KeyError, TypeError, ValueError):
                self.invalid_diagnostics += 1
                event["diagnostic_invalid"] = True
        self.stream.write(json.dumps(event, allow_nan=False) + "\n")
        self.stream.flush()
        self.first.setdefault(channel, received)
        if channel in self.last:
            self.gaps[channel] = max(self.gaps.get(channel, 0), received - self.last[channel])
        self.last[channel] = received
        self.counts[channel] += 1
        self.total += 1
        return True

    def summary(self, ended):
        return {"schema": "rrm-control-capture/v1", "events": self.total,
                "elapsed_wall_s": ended - self.start,
                "event_limit_reached": self.total >= self.max_events,
                "missing_channels": [c for c in self.channels if not self.counts[c]],
                "channels": {c: {"topic": topic, "count": self.counts[c],
                                  "first_receipt_offset_s": (self.first[c] - self.start
                                                              if c in self.first else None),
                                  "final_receipt_age_s": (ended - self.last[c]
                                                          if c in self.last else None),
                                  "max_receipt_gap_s": self.gaps.get(c)}
                             for c, topic in self.channels.items()},
                "admission_reason_mask_counts": dict(self.reasons),
                "invalid_diagnostics": self.invalid_diagnostics,
                "diagnostic_sequence_discontinuities": self.sequence_discontinuities,
                "execution_dispatch": False,
                "limitations": "Receipt coverage is observational; sequence gaps may be "
                "transport loss or process reset. No flight/safety or source/binary equivalence claim."}


def graph_snapshot(node, admission_topic):
    """Graph availability can disappear before Python handles a ROS SIGINT."""
    try:
        return {"available": True, "error": None,
                "publishers": node.count_publishers(admission_topic),
                "capture_publishers": node.get_publisher_names_and_types_by_node(
                    node.get_name(), node.get_namespace()),
                "capture_services": node.get_service_names_and_types_by_node(
                    node.get_name(), node.get_namespace())}
    except Exception as exc:
        # Unknown must not be reported as zero publishers or an empty graph.
        return {"available": False, "error": type(exc).__name__ + ": " + str(exc),
                "publishers": None, "capture_publishers": None, "capture_services": None}


def bounded_duration(value):
    duration = float(value)
    if not math.isfinite(duration) or not 0 < duration <= 300:
        raise argparse.ArgumentTypeError("duration must be finite and in (0, 300] seconds")
    return duration


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--duration", type=bounded_duration, default=15.0)
    parser.add_argument("--robot", default="robot_1")
    parser.add_argument("--source-revision", default="UNKNOWN")
    args = parser.parse_args()
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.parameter import Parameter
    from rosidl_runtime_py.convert import message_to_ordereddict
    from airstack_msgs.msg import Odometry as Tracking
    from nav_msgs.msg import Odometry
    from pid_controller_msgs.msg import PIDInfo
    from mavros_msgs.msg import AttitudeTarget, State, ExtendedState
    from mav_msgs.msg import RollPitchYawrateThrust
    from std_msgs.msg import Bool, String
    from rosgraph_msgs.msg import Clock

    prefix = f"/{args.robot}"
    specs = {"sim_clock": ("/clock", Clock),
             "tracking": (prefix + "/trajectory_controller/tracking_point", Tracking),
             "odom": (prefix + "/odometry_conversion/odometry", Odometry),
             "admission": (prefix + "/control/admission_diagnostic", String),
             "authority_diagnostic": (prefix + "/takeoff_landing_planner/takeoff_landing_task/authority_diagnostic", String),
             "armed": (prefix + "/interface/is_armed", Bool),
             "authority": (prefix + "/interface/has_control", Bool),
             "controller_cmd": (prefix + "/interface/cmd_roll_pitch_yawrate_thrust", RollPitchYawrateThrust),
             "state": (prefix + "/interface/mavros/state", State),
             "landed": (prefix + "/interface/mavros/extended_state", ExtendedState),
             "attitude_cmd": (prefix + "/interface/mavros/setpoint_raw/attitude", AttitudeTarget),
             "attitude_target": (prefix + "/interface/mavros/setpoint_raw/target_attitude", AttitudeTarget)}
    specs.update({axis + "_pid": (prefix + "/control/" + axis + "_pid_info", PIDInfo)
                  for axis in ("x", "y", "z", "vx", "vy", "vz")})
    args.output.parent.mkdir(parents=True, exist_ok=True)
    # Exclusive creation prevents accidental loss of retained evidence.
    with args.output.open("x") as stream:
        rclpy.init()
        node = rclpy.create_node("rrm_control_capture", enable_rosout=False,
                                 start_parameter_services=False,
                                 parameter_overrides=[Parameter("use_sim_time", value=True),
                                     Parameter("start_type_description_service", value=False)])
        start = time.monotonic()
        capture = Capture(stream, {c: spec[0] for c, spec in specs.items()}, start)
        qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
        stop_reason = "duration"
        initial_graph = None
        try:
            for channel, (topic, message_type) in specs.items():
                node.create_subscription(message_type, topic,
                    lambda msg, c=channel: capture.record(
                        c, message_to_ordereddict(msg), time.monotonic(),
                        node.get_clock().now().nanoseconds), qos)
            initial_graph = graph_snapshot(node, prefix + "/control/admission_diagnostic")
            while rclpy.ok() and time.monotonic() - start < args.duration:
                if capture.total >= capture.max_events:
                    stop_reason = "event_limit"
                    break
                rclpy.spin_once(node, timeout_sec=0.05)
            if stop_reason == "duration" and not rclpy.ok():
                stop_reason = "ros_shutdown"
        except KeyboardInterrupt:
            stop_reason = "interrupted"
        except ExternalShutdownException:
            stop_reason = "ros_shutdown"
        except Exception:
            stop_reason = "error"
            raise
        finally:
            summary = capture.summary(time.monotonic())
            summary.update({"stop_reason": stop_reason,
                            "captured_at": datetime.now(timezone.utc).isoformat(),
                            "source_revision_label": args.source_revision,
                            "ros_clock": "simulation (/clock); zero until first clock delivery",
                            "recorder_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                            "graph_at_start": initial_graph,
                            "graph": graph_snapshot(node, prefix + "/control/admission_diagnostic")})
            args.output.with_suffix(args.output.suffix + ".summary.json").write_text(
                json.dumps(summary, indent=2) + "\n")
            node.destroy_node()
            rclpy.try_shutdown()
    print(json.dumps(summary, sort_keys=True))


if __name__ == "__main__":
    main()
