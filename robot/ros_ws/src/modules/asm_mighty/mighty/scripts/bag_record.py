# /* ----------------------------------------------------------------------------
#  * Copyright 2025, Kota Kondo, Aerospace Controls Laboratory
#  * Massachusetts Institute of Technology
#  * All Rights Reserved
#  * Authors: Kota Kondo, et al.
#  * See LICENSE file for the license information
#  * -------------------------------------------------------------------------- */

import os
import subprocess
import argparse
import datetime

# Topics that exist under /<agent>/... — every entry below is recorded for each
# agent in the --agents list. The set mirrors mighty.rviz so any topic that is
# currently visualized in RViz is captured.
PER_AGENT_TOPICS = [
    # ---- 2D occupancy / ESDF / persistent maps ----
    "/occ_2d_topic",
    "/occ_2d_topic_updates",
    "/esdf_2d_topic",
    "/esdf_2d_topic_updates",
    "/exploration/visited_map",
    "/exploration/visited_map_updates",
    "/ground_2d_heat",
    "/ground_2d_occupied",
    # ---- 3D occupancy / heat / dynamic / free / unknown clouds ----
    "/occupancy_grid",
    "/heat_cloud",
    "/dynamic_grid",
    "/free_grid",
    "/unknown_grid",
    # ---- Trajectories ----
    "/traj_committed_colored",
    "/traj_subopt_colored",
    "/actual_traj",
    # ---- Global / hierarchical path ----
    "/hgp_path_marker",
    "/original_hgp_path_marker",
    # ---- Goal / waypoint markers ----
    "/point_A",
    "/point_E",
    "/point_G",
    "/point_G_term",
    "/term_goal",
    # ---- Frontier exploration ----
    "/exploration/frontiers",
    # ---- Obstacle tracking ----
    "/tracked_obstacles",
    "/cluster_bounding_boxes",
    "/uncertainty_spheres",
    # NOTE: /poly_whole uses decomp_ros_msgs/PolyhedronArray, whose
    # libdecomp_ros_msgs__rosidl_typesupport_fastrtps_cpp.so isn't installed.
    # Excluded so the recorder doesn't bail. Re-enable after installing
    # ros-humble-decomp-ros-msgs (or building decomp_ros_msgs from source).
    # ---- Sensors actually shown in RViz ----
    "/mid360_PointCloud2",
    "/d435/depth/color/points",
    "/d435/color/image_raw",
    # ---- Robot / FOV / overlays ----
    "/drone_marker",
    "/fov",
    "/vel_text",
    "/hover_avoidance_viz",
]

# Topics that are not under /<agent>/... — recorded once regardless of how many
# agents are in --agents. Includes essentials (tf, rosout, clock).
STATIC_TOPICS = [
    # ---- ROS essentials ----
    "/tf",
    "/tf_static",
    "/clock",
    "/rosout",
    "/parameter_events",
    # ---- RViz interaction ----
    "/clicked_point",
    "/initialpose",
    # ---- Sim / world geometry ----
    "/map_generator/global_cloud",
    "/shapes_dynamic_mesh",
    "/tf_axis",
    "/local_traj_overlay",
    # ---- Mighty global debug overlays ----
    "/mighty/poly_time_layer",
    "/mighty/temporal_test_markers",
]


def record_ros2_bag(bag_name, bag_path, agents, topics=None, storage="mcap"):
    # Build per-agent topic list
    all_topics = []
    for agent in agents:
        for topic in PER_AGENT_TOPICS:
            all_topics.append(f"/{agent}{topic}")
    all_topics.extend(STATIC_TOPICS)

    if topics is not None:
        all_topics = topics

    # Make sure the destination directory exists
    os.makedirs(bag_path, exist_ok=True)

    out_path = os.path.join(bag_path, bag_name)
    command = [
        "ros2", "bag", "record",
        "-s", storage,                       # mcap is faster + smaller than sqlite3
        "-o", out_path,
    ] + all_topics

    print(f"Recording {len(all_topics)} topics → {out_path}")
    print("Press Ctrl-C to stop recording.")
    try:
        subprocess.run(command, check=True)
        print(f"\nBag saved to: {out_path}")
    except KeyboardInterrupt:
        print(f"\nRecording stopped. Bag at: {out_path}")
    except subprocess.CalledProcessError as e:
        print(f"\nFailed to start recording: {e}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Record a ROS 2 bag of MIGHTY topics")
    parser.add_argument(
        "--bag_name",
        type=str,
        default=None,
        help="Bag name (default: timestamped 'mighty_<YYYYMMDD_HHMMSS>')",
    )
    parser.add_argument(
        "--bag_path",
        type=str,
        default="/home/kkondo/data/mighty_exploration",
        help="Directory to write the bag into",
    )
    parser.add_argument(
        "--agents",
        nargs="+",
        default=["NX01"],
        help="Agent namespaces to record (e.g. NX01 NX02). Defaults to NX01.",
    )
    parser.add_argument(
        "--storage",
        type=str,
        default="mcap",
        choices=["mcap", "sqlite3"],
        help="Storage backend (mcap recommended)",
    )
    args = parser.parse_args()

    if args.bag_name is None:
        args.bag_name = "mighty_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S")

    print("Bag name :", args.bag_name)
    print("Bag path :", args.bag_path)
    print("Agents   :", args.agents)
    print("Storage  :", args.storage)

    record_ros2_bag(args.bag_name, args.bag_path, args.agents, storage=args.storage)
