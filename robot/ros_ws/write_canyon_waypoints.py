#!/usr/bin/env python3
"""
Write GPS-degradation canyon test waypoints to the waypoints_1 ROS2 bag.

Run inside the robot container:
    docker exec airstack-robot-desktop-1 bash -lc \
        'source /root/AirStack/robot/ros_ws/install/setup.bash && \
         python3 /root/AirStack/robot/ros_ws/write_canyon_waypoints.py'
"""
import sqlite3
from rclpy.serialization import serialize_message
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time

# ── CCity scene geometry after launch transforms ────────────────────────────
# The city USD is Y-up and authored in centimetres. The Isaac launch applies
# scale=0.01 and rotate_x=90°, so ROS/map coordinates are ENU:
#   x ≈ original X * 0.01, y ≈ -original Z * 0.01, z ≈ original Y * 0.01
#
# Measured transformed scene bounds:
#   overall:        x=[-36.35, 16.20], y=[-0.89, 21.14], z=[0.00, 3.00]
#   building blocks: mostly x=[-35, 6], y=[7.3, 19.5], z=[0.02, 3.00]
#
# Route design:
#   Fly the street gaps at y=6.6, 13.5, 16.8 and vertical corridors centered
#   near x=-6.7, -13.5, -21.8, -27.0, -33.8. These points avoid measured
#   building bounding boxes but keep the drone low enough for realistic
#   satellite line-of-sight blockage.

WAYPOINTS = [
    # Phase 1: enter the city from open south road.
    (  4.0,  6.6, 2.2),
    ( -6.7,  6.6, 2.2),

    # Phase 2: first narrow vertical canyon.
    ( -6.7, 13.5, 2.2),
    (-13.5, 13.5, 2.2),
    (-13.5,  6.6, 2.2),

    # Phase 3: deeper west-side canyons between dense building rows.
    (-21.8,  6.6, 2.2),
    (-21.8, 13.5, 2.2),
    (-27.0, 13.5, 2.2),
    (-27.0,  6.6, 2.2),

    # Phase 4: far-west block pass, then return through the northern street.
    (-33.8,  6.6, 2.2),
    (-33.8, 13.5, 2.2),
    (-27.0, 16.8, 2.2),
    (-21.8, 16.8, 2.2),
    (-13.5, 16.8, 2.2),
    ( -6.7, 16.8, 2.2),
    ( -0.5, 13.5, 2.2),
    (  4.0, 13.5, 2.2),
]

DB = "/root/AirStack/robot/ros_ws/waypoints_1/waypoints_1_new_0.db3"


def build_path():
    path = Path()
    path.header.frame_id = "map"
    path.header.stamp = Time(sec=0, nanosec=0)
    for x, y, z in WAYPOINTS:
        ps = PoseStamped()
        ps.header = path.header
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
    return path


def main():
    path = build_path()
    data = serialize_message(path)

    conn = sqlite3.connect(DB)
    cur = conn.cursor()
    cur.execute("DELETE FROM messages")
    cur.execute(
        "INSERT INTO messages (topic_id, timestamp, data) VALUES (1, 1933129956791, ?)",
        (data,),
    )
    conn.commit()
    conn.close()

    print(f"Written {len(WAYPOINTS)} waypoints to {DB}")
    for i, (x, y, z) in enumerate(WAYPOINTS):
        tag = ""
        print(f"  WP{i+1:02d}: x={x:6.1f}  y={y:5.1f}  z={z:.1f}m")


if __name__ == "__main__":
    main()
