#!/usr/bin/env python3
"""
wait_for_px4.py - Wait until PX4 MAVLink backend is actually sending heartbeats.
Uses pymavlink to bind to the correct local UDP port and block until heartbeat.
"""

import sys
import re
import argparse
from pymavlink import mavutil
import time

def get_robot_ports(robot_name: str) -> tuple[int, int]:
    """
    Return (local_port, remote_port) for MAVLink UDP connection.
    Example: robot_1 -> local=14540, remote=14580
    """
    match = re.match(r'robot_(\d+)', robot_name)
    if match:
        robot_num = int(match.group(1))
        local_port = 14540 + (robot_num - 1)  # MAVROS local port
        remote_port = 14580 + (robot_num - 1)  # PX4 remote port
        return local_port, remote_port
    return 14540, 14580

def wait_for_px4(robot_name: str, timeout: int = 120) -> bool:
    """Wait for a MAVLink heartbeat from PX4 on the real MAVROS port."""
    local_port, remote_port = get_robot_ports(robot_name)
    bind_addr = f"udp:0.0.0.0:{local_port}"  # bind to MAVROS port

    print(f"[INFO] Waiting for PX4 heartbeat for {robot_name}")
    print(f"[INFO] Listening on local port {local_port}, sending heartbeat to PX4 remote port {remote_port}")
    print(f"[INFO] Timeout: {timeout} seconds")

    try:
        mav = mavutil.mavlink_connection(bind_addr, dialect='common')
    except Exception as e:
        print(f"[ERROR] Could not bind to {bind_addr}: {e}")
        return False

    # Send a heartbeat from the local port to PX4 to trigger responses
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_GCS,
        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
        0, 0, 0
    )

    start_time = time.time()
    while True:
        elapsed = int(time.time() - start_time)
        if elapsed >= timeout:
            print(f"[ERROR] Timeout: No heartbeat received from PX4 after {timeout} seconds.")
            return False

        try:
            hb = mav.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
            if hb:
                print(f"[OK] Heartbeat received from PX4! "
                      f"System ID: {hb.get_srcSystem()}, Component ID: {hb.get_srcComponent()}")
                mav.close()  # Release the port for MAVROS
                return True
        except Exception:
            pass

        print(f"[WAIT] No heartbeat yet... ({elapsed}/{timeout} seconds)")



def main():
    parser = argparse.ArgumentParser(description="Wait for PX4 MAVLink heartbeat")
    parser.add_argument("robot_name", nargs="?", default="robot_1",
                        help="Robot name (e.g., robot_1, robot_2)")
    parser.add_argument("timeout", nargs="?", type=int, default=120,
                        help="Timeout in seconds")

    args = parser.parse_args()

    success = wait_for_px4(args.robot_name, args.timeout)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
