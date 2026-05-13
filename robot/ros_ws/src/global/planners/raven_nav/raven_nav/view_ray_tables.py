#!/usr/bin/env python3
"""view_ray_tables — live viewer for raven_nav debug tables.

Subscribes to:
  /<robot>/debug/ray_table     (std_msgs/String)
  /<robot>/debug/groups_table  (std_msgs/String)

Renders both tables in the terminal with a screen-clear on every update so
they don't scroll away. Run inside a tmux pane on the robot container:

    docker exec -it airstack-robot-desktop-1 bash
    sws
    ros2 run raven_nav view_ray_tables                  # uses $ROBOT_NAME
    ros2 run raven_nav view_ray_tables --robot robot_2  # explicit
"""
from __future__ import annotations

import argparse
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


RESET = "\033[0m"
BOLD = "\033[1m"
DIM = "\033[2m"
CYAN = "\033[36m"
YELLOW = "\033[33m"
GREEN = "\033[32m"


def _clear():
    sys.stdout.write("\033[2J\033[H")
    sys.stdout.flush()


class TableViewer(Node):
    def __init__(self, robot: str):
        super().__init__("view_ray_tables")
        self._robot = robot
        self._ray_table = "(waiting...)"
        self._groups_table = "(waiting...)"
        self._bids_table = "(waiting...)"
        self._voxel_table = "(waiting...)"
        self._frontier_table = "(waiting...)"
        self._discoveries_table = "(waiting...)"
        self._ray_recv: float | None = None
        self._groups_recv: float | None = None
        self._bids_recv: float | None = None
        self._voxel_recv: float | None = None
        self._frontier_recv: float | None = None
        self._discoveries_recv: float | None = None
        # Map robot_name -> (nav_mode, recv_time). Includes self.
        self._nav_modes: dict[str, tuple[str, float]] = {}
        self._nav_subs: dict[str, object] = {}
        self._lock = threading.Lock()

        self.create_subscription(
            String, f"/{robot}/debug/ray_table", self._on_ray, 10)
        self.create_subscription(
            String, f"/{robot}/debug/groups_table", self._on_groups, 10)
        self.create_subscription(
            String, f"/{robot}/debug/bids_table", self._on_bids, 10)
        self.create_subscription(
            String, f"/{robot}/debug/voxel_table", self._on_voxel, 10)
        self.create_subscription(
            String, f"/{robot}/debug/frontier_table", self._on_frontier, 10)
        self.create_subscription(
            String, f"/{robot}/debug/discoveries_table",
            self._on_discoveries, 10)

        # Discover navigation_mode topics for self + peers and subscribe to
        # each one. Re-scan periodically so peers that come up after we start
        # are picked up.
        self._discover_nav_modes()
        self.create_timer(2.0, self._discover_nav_modes)

    def _discover_nav_modes(self) -> None:
        for name, types in self.get_topic_names_and_types():
            if not name.endswith("/navigation_mode"):
                continue
            if "std_msgs/msg/String" not in types:
                continue
            # /<robot>/navigation_mode -> robot
            parts = name.strip("/").split("/")
            if len(parts) != 2:
                continue
            robot = parts[0]
            if robot in self._nav_subs:
                continue
            self._nav_subs[robot] = self.create_subscription(
                String, name,
                lambda msg, r=robot: self._on_nav_mode(r, msg), 10)

    def _on_nav_mode(self, robot: str, msg: String) -> None:
        with self._lock:
            self._nav_modes[robot] = (msg.data, time.time())

    def _on_ray(self, msg: String) -> None:
        with self._lock:
            self._ray_table = msg.data
            self._ray_recv = time.time()

    def _on_groups(self, msg: String) -> None:
        with self._lock:
            self._groups_table = msg.data
            self._groups_recv = time.time()

    def _on_bids(self, msg: String) -> None:
        with self._lock:
            self._bids_table = msg.data
            self._bids_recv = time.time()

    def _on_voxel(self, msg: String) -> None:
        with self._lock:
            self._voxel_table = msg.data
            self._voxel_recv = time.time()

    def _on_frontier(self, msg: String) -> None:
        with self._lock:
            self._frontier_table = msg.data
            self._frontier_recv = time.time()

    def _on_discoveries(self, msg: String) -> None:
        with self._lock:
            self._discoveries_table = msg.data
            self._discoveries_recv = time.time()

    def render(self) -> None:
        with self._lock:
            ray = self._ray_table
            groups = self._groups_table
            bids = self._bids_table
            voxels = self._voxel_table
            frontier = self._frontier_table
            discoveries = self._discoveries_table
            ray_age = (time.time() - self._ray_recv
                       if self._ray_recv is not None else None)
            grp_age = (time.time() - self._groups_recv
                       if self._groups_recv is not None else None)
            bid_age = (time.time() - self._bids_recv
                       if self._bids_recv is not None else None)
            vox_age = (time.time() - self._voxel_recv
                       if self._voxel_recv is not None else None)
            front_age = (time.time() - self._frontier_recv
                         if self._frontier_recv is not None else None)
            disc_age = (time.time() - self._discoveries_recv
                        if self._discoveries_recv is not None else None)
            nav_modes_snapshot = dict(self._nav_modes)
        _clear()
        now = time.strftime("%H:%M:%S")
        ray_age_s = f"{ray_age:.1f}s ago" if ray_age is not None else "never"
        grp_age_s = f"{grp_age:.1f}s ago" if grp_age is not None else "never"
        bid_age_s = f"{bid_age:.1f}s ago" if bid_age is not None else "never"
        vox_age_s = f"{vox_age:.1f}s ago" if vox_age is not None else "never"
        front_age_s = (f"{front_age:.1f}s ago"
                       if front_age is not None else "never")
        disc_age_s = (f"{disc_age:.1f}s ago"
                      if disc_age is not None else "never")
        print(f"{BOLD}raven_nav debug tables{RESET}  "
              f"{DIM}[robot={self._robot}  {now}]{RESET}")
        print("─" * 100)
        print(f"\033[34m{BOLD}[nav modes]{RESET}")
        if not nav_modes_snapshot:
            print("  (no navigation_mode topics discovered yet)")
        else:
            now_ts = time.time()
            for r in sorted(nav_modes_snapshot):
                mode, recv = nav_modes_snapshot[r]
                age = now_ts - recv
                tag = f"{BOLD}(self){RESET}" if r == self._robot else "(peer)"
                print(f"  {r:<16} {tag}  mode={mode:<16} "
                      f"{DIM}({age:.1f}s ago){RESET}")
        print()
        print(f"{CYAN}{BOLD}[ray_table]{RESET} "
              f"{DIM}(updated {ray_age_s}){RESET}")
        print(ray)
        print()
        print(f"{YELLOW}{BOLD}[groups]{RESET} "
              f"{DIM}(updated {grp_age_s}){RESET}")
        print(groups)
        print()
        print(f"{GREEN}{BOLD}[bids]{RESET} "
              f"{DIM}(updated {bid_age_s}){RESET}")
        print(bids)
        print()
        print(f"\033[35m{BOLD}[voxels]{RESET} "
              f"{DIM}(updated {vox_age_s}){RESET}")
        print(voxels)
        print()
        print(f"\033[36m{BOLD}[frontiers]{RESET} "
              f"{DIM}(updated {front_age_s}){RESET}")
        print(frontier)
        print()
        print(f"\033[31m{BOLD}[discoveries]{RESET} "
              f"{DIM}(updated {disc_age_s}){RESET}")
        print(discoveries)
        print()
        print(f"{DIM}Ctrl+C to quit{RESET}")


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--robot", default=os.environ.get("ROBOT_NAME", "robot_1"),
                        help="Robot namespace (default: $ROBOT_NAME or robot_1)")
    parser.add_argument("--rate", type=float, default=4.0,
                        help="Render refresh rate Hz (default 4)")
    args = parser.parse_args()

    rclpy.init()
    node = TableViewer(robot=args.robot)
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
