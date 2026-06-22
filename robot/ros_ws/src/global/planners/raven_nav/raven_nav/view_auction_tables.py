#!/usr/bin/env python3
"""view_auction_tables — live viewer for the raven_nav consensus auction table.

Subscribes to /<robot>/raven_nav/auction_table (std_msgs/String, JSON) and
renders two tables, screen-cleared on every update:

  1. VISITED BBs   — targets already reached (removed from the auction).
  2. AVAILABLE      — the live auction table: ray-leads + BBs, each with its
                      state and the robot it's assigned to.

Run inside the robot container:

    docker exec -it airstack-robot-desktop-1 bash
    sws
    ros2 run raven_nav view_auction_tables                  # uses $ROBOT_NAME
    ros2 run raven_nav view_auction_tables --robot robot_2  # explicit
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

RESET, BOLD, DIM = "\033[0m", "\033[1m", "\033[2m"
STATE_COLOR = {
    'ray': "\033[33m",            # yellow  — ray-lead bearing (range unknown)
    'bb-observing': "\033[36m",   # cyan    — voxel BB
    'bb-visited': "\033[2m",      # dim     — done
}


def _clear():
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()


class AuctionViewer(Node):
    def __init__(self, robot: str):
        super().__init__("view_auction_tables")
        self._robot = robot
        self._data: dict | None = None
        self._recv: float | None = None
        self._lock = threading.Lock()
        self.create_subscription(
            String, f"/{robot}/raven_nav/auction_table", self._on_table, 10)

    def _on_table(self, msg: String) -> None:
        try:
            data = json.loads(msg.data)
        except (ValueError, TypeError):
            return
        with self._lock:
            self._data = data
            self._recv = time.time()

    def render(self) -> None:
        with self._lock:
            data = self._data
            age = (time.time() - self._recv) if self._recv is not None else None
        _clear()
        now = time.strftime("%H:%M:%S")
        age_s = f"{age:.1f}s ago" if age is not None else "never"
        frame = (data or {}).get("frame", "?")
        print(f"{BOLD}raven_nav auction table{RESET}  "
              f"{DIM}[robot={self._robot}  {now}  updated {age_s}  "
              f"coords={frame}]{RESET}")
        print("─" * 78)
        if data is None:
            print("  (waiting for /%s/raven_nav/auction_table ...)" % self._robot)
            print(f"\n{DIM}Ctrl+C to quit{RESET}")
            return

        visited = data.get("visited_bbs", [])
        print(f"{BOLD}[1] VISITED BBs{RESET}  ({len(visited)})  "
              f"{DIM}— reached, removed from the auction{RESET}")
        if not visited:
            print("    (none)")
        else:
            print(f"    {'label':<16} {'x':>8} {'y':>8} {'z':>7}")
            for v in visited:
                print(f"    {str(v.get('label','')):<16} "
                      f"{v.get('x',0):>8.1f} {v.get('y',0):>8.1f} "
                      f"{v.get('z',0):>7.1f}")
        print()

        avail = data.get("available", [])
        my_id = data.get("my_id")
        print(f"{BOLD}[2] AVAILABLE TARGETS{RESET}  ({len(avail)})  "
              f"{DIM}— BBs + ray-leads in the auction{RESET}")
        if not avail:
            print("    (none)")
        else:
            print(f"    {'query':<16} {'state':<10} {'x':>8} {'y':>8} {'assigned':>9}")
            # BBs first, then ray-leads.
            order = {'bb-visited': 0, 'bb-observing': 1, 'ray': 2}
            for t in sorted(avail, key=lambda r: order.get(r.get('status'), 9)):
                st = t.get('status', '?')
                col = STATE_COLOR.get(st, "")
                # rays carry only their query; BBs show observing/visited (no 'bb-').
                state = st[3:] if st.startswith('bb-') else ""
                aid = t.get('assigned')
                mine = (aid is not None and aid == my_id)
                who = (f"r{aid}" if aid is not None else "-")
                who_s = (f"{BOLD}{who}*{RESET}" if mine else who)
                print(f"    {col}{str(t.get('label','')):<16}{RESET} {state:<10} "
                      f"{t.get('x',0):>8.1f} {t.get('y',0):>8.1f} {who_s:>9}")
        print(f"\n{DIM}* = assigned to this robot.  Ctrl+C to quit{RESET}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--robot", default=os.environ.get("ROBOT_NAME", "robot_1"),
                        help="Robot namespace (default: $ROBOT_NAME or robot_1)")
    parser.add_argument("--rate", type=float, default=4.0,
                        help="Render refresh rate Hz (default 4)")
    args = parser.parse_args()

    rclpy.init()
    node = AuctionViewer(robot=args.robot)
    interval = 1.0 / max(args.rate, 0.5)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=interval)
            node.render()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
