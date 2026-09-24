"""Include rayfronts.launch.xml only on the robots named in RAYFRONTS_ROBOTS.

Robot containers are replicas of one compose service and share one environment,
so a per-robot on/off cannot come from the env alone; this compares the
container's ROBOT_NAME against a comma-separated allowlist (default: every
robot). One RADIO mapper costs ~4 GB of GPU, so a 16 GB card shared with Isaac
Sim fits one: RAYFRONTS_ROBOTS=robot_1.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def _wanted() -> bool:
    robots = [r.strip() for r in os.environ.get("RAYFRONTS_ROBOTS", "").split(",") if r.strip()]
    if os.environ.get("RAYFRONTS_PER_ROBOT", "true").lower() not in ("1", "true", "yes", "on"):
        return False
    return not robots or os.environ.get("ROBOT_NAME", "") in robots


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("emb_enabled", default_value=os.environ.get("RAYFRONTS_EMB", "false")),
        DeclareLaunchArgument("config_name", default_value="low_memory"),
        DeclareLaunchArgument("query_period", default_value="10"),
    ]
    if not _wanted():
        return LaunchDescription(args)
    include = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([FindPackageShare("perception_bringup"), "/launch/rayfronts.launch.xml"]),
        launch_arguments={
            "emb_enabled": LaunchConfiguration("emb_enabled"),
            "config_name": LaunchConfiguration("config_name"),
            "query_period": LaunchConfiguration("query_period"),
        }.items(),
    )
    return LaunchDescription(args + [include])
