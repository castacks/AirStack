#!/usr/bin/env python3
"""
peer_registry_monitor.py — CLI diagnostic tool for the gossip peer registry.

Run on any robot or from a machine joined to domain 99:
    ROS_DOMAIN_ID=99 python3 peer_registry_monitor.py

Or on a specific robot's domain to see what that robot receives:
    ROS_DOMAIN_ID=1 python3 peer_registry_monitor.py

Options:
  --robot   Only show entries for this robot name (partial match)
  --rate    Refresh rate in Hz (default: 2)
"""

import argparse
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from coordination_msgs.msg import PeerProfile as PeerProfileMsg

GOSSIP_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)

RESET = "\033[0m"
BOLD = "\033[1m"
CYAN = "\033[36m"
YELLOW = "\033[33m"
GREEN = "\033[32m"
DIM = "\033[2m"


def _fmt_gps(gps_fix, heading: float) -> str:
    from sensor_msgs.msg import NavSatStatus
    status = gps_fix.status.status
    status_str = {
        NavSatStatus.STATUS_NO_FIX: "NO_FIX",
        NavSatStatus.STATUS_FIX: "FIX",
        NavSatStatus.STATUS_SBAS_FIX: "SBAS",
        NavSatStatus.STATUS_GBAS_FIX: "GBAS",
    }.get(status, f"?{status}")
    return (
        f"lat={gps_fix.latitude:11.7f}  lon={gps_fix.longitude:11.7f}  "
        f"alt={gps_fix.altitude:7.2f}m  hdg={heading:6.1f}°  [{status_str}]"
    )


def _fmt_waypoint(pose_stamped) -> str:
    s = pose_stamped.header.stamp
    if s.sec == 0 and s.nanosec == 0:
        return f"{DIM}(no plan yet){RESET}"
    p = pose_stamped.pose.position
    o = pose_stamped.pose.orientation
    return f"pos=({p.x:7.2f}, {p.y:7.2f}, {p.z:7.2f})  orient=({o.x:.3f}, {o.y:.3f}, {o.z:.3f}, {o.w:.3f})"


def _fmt_stamp(gps_fix) -> str:
    s = gps_fix.header.stamp
    if s.sec == 0 and s.nanosec == 0:
        return "n/a"
    t = s.sec + s.nanosec * 1e-9
    return time.strftime("%H:%M:%S", time.localtime(t)) + f".{s.nanosec // 1_000_000:03d}"


def _clear():
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()


class RegistryMonitor(Node):

    def __init__(self, filter_name: str = ""):
        super().__init__("peer_registry_monitor")
        self._registry: dict[str, PeerProfileMsg] = {}
        self._recv_times: dict[str, float] = {}
        self._registry_lock = threading.Lock()
        self._filter = filter_name.lower()
        self._inbox: dict[str, PeerProfileMsg] = {}
        self._inbox_lock = threading.Lock()

        self._sub = self.create_subscription(
            PeerProfileMsg, "/gossip/peers", self._on_msg, GOSSIP_QOS,
        )

    def _on_msg(self, msg: PeerProfileMsg) -> None:
        new_t = (msg.gps_fix.header.stamp.sec
                 + msg.gps_fix.header.stamp.nanosec * 1e-9)
        with self._inbox_lock:
            existing = self._inbox.get(msg.robot_name)
            if existing is not None:
                old_t = (existing.gps_fix.header.stamp.sec
                         + existing.gps_fix.header.stamp.nanosec * 1e-9)
                if new_t < old_t:
                    self._recv_times[msg.robot_name] = time.time()
                    return
            self._inbox[msg.robot_name] = msg
            self._recv_times[msg.robot_name] = time.time()

    def _drain_inbox(self) -> None:
        with self._inbox_lock:
            inbox = dict(self._inbox)
            self._inbox.clear()
        for robot_name, msg in inbox.items():
            new_t = (msg.gps_fix.header.stamp.sec
                     + msg.gps_fix.header.stamp.nanosec * 1e-9)
            with self._registry_lock:
                existing = self._registry.get(robot_name)
                if existing is not None:
                    old_t = (existing.gps_fix.header.stamp.sec
                             + existing.gps_fix.header.stamp.nanosec * 1e-9)
                    if new_t < old_t:
                        continue
                self._registry[robot_name] = msg

    def print_registry(self) -> None:
        self._drain_inbox()
        _clear()
        domain = os.environ.get("ROS_DOMAIN_ID", "?")
        now_str = time.strftime("%H:%M:%S")
        print(f"{BOLD}Peer Registry  {DIM}[domain={domain}  {now_str}]{RESET}")
        print("─" * 80)

        with self._registry_lock:
            entries = sorted(self._registry.values(), key=lambda m: m.robot_name)
            recv_times = dict(self._recv_times)
        if self._filter:
            entries = [e for e in entries if self._filter in e.robot_name.lower()]

        if not entries:
            print(f"  {DIM}(no peers seen yet){RESET}")
        else:
            now = time.time()
            for msg in entries:
                src = "direct" if msg.source == 0 else f"relayed({msg.relay_hops}h)"
                payload_summary = (
                    f"{len(msg.payloads)} payload(s): "
                    + ", ".join(p.payload_type for p in msg.payloads)
                    if msg.payloads
                    else "no payloads"
                )
                recv_t = recv_times.get(msg.robot_name)
                age = f"{now - recv_t:.1f}s ago" if recv_t is not None else "?"
                recv_wall = time.strftime("%H:%M:%S", time.localtime(recv_t)) if recv_t else "?"
                stamp_str = f"{recv_wall}  ({age})"
                print(f"  {CYAN}{BOLD}{msg.robot_name}{RESET}  {DIM}[{src}  last_recv={stamp_str}]{RESET}")
                print(f"    {GREEN}gps     {RESET} {_fmt_gps(msg.gps_fix, msg.heading)}")
                print(f"    {YELLOW}waypoint{RESET} {_fmt_waypoint(msg.waypoint)}")
                print(f"    {DIM}payloads{RESET} {payload_summary}")
                print()

        print(f"{DIM}Listening on /gossip/peers — Ctrl+C to quit{RESET}")


def main():
    parser = argparse.ArgumentParser(description="Live peer registry monitor")
    parser.add_argument("--robot", default="", help="Filter by robot name (partial)")
    parser.add_argument("--rate", type=float, default=2.0, help="Refresh rate Hz (default 2)")
    args = parser.parse_args()

    rclpy.init()
    node = RegistryMonitor(filter_name=args.robot)
    interval = 1.0 / max(args.rate, 0.1)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=interval)
            node.print_registry()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
