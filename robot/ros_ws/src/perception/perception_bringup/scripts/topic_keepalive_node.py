#!/usr/bin/env python3
"""Keep Isaac Sim's on-demand publishers alive by subscribing silently.

Isaac Sim's Replicator/SDG ROS 2 publisher only emits a message when at
least one rclcpp subscriber exists on the robot domain. ddsrouter's bare
DDS reader does not count, so without a local subscriber the topic stops
publishing and downstream GCS / Foxglove sees nothing. The list mirrors
every Topic.Value entry in desktop_bringup/rviz/robot.rviz so disabling
rviz no longer drops topics off the GCS side.
"""

import os

import rclpy
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile,
                       ReliabilityPolicy)
from sensor_msgs.msg import CameraInfo, Image, PointCloud, PointCloud2
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker, MarkerArray
from tf2_msgs.msg import TFMessage


SENSOR_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
)
DEFAULT_QOS = 10  # rclpy default: RELIABLE / VOLATILE / KEEP_LAST(10)


# (topic, msg_type, absolute, qos)
TOPICS = [
    ('sensors/front_stereo/left/image_rect',          Image,        False, SENSOR_QOS),
    ('sensors/front_stereo/right/image_rect',         Image,        False, SENSOR_QOS),
    ('sensors/front_stereo/left/depth_ground_truth',  Image,        False, SENSOR_QOS),
    ('sensors/front_stereo/right/depth_ground_truth', Image,        False, SENSOR_QOS),
    ('perception/macvo/disparity',                    Image,        False, SENSOR_QOS),
    ('sensors/front_stereo/left/camera_info',         CameraInfo,   False, DEFAULT_QOS),
    ('sensors/front_stereo/right/camera_info',        CameraInfo,   False, DEFAULT_QOS),
    ('sensors/lidar/point_cloud',                     PointCloud2,  False, SENSOR_QOS),
    ('perception/stereo_image_proc/point_cloud',      PointCloud2,  False, SENSOR_QOS),
    ('droan/expansion_cloud',                         PointCloud2,  False, SENSOR_QOS),
    ('droan/fg_bg_cloud',                             PointCloud2,  False, SENSOR_QOS),
    ('perception/macvo/point_cloud',                  PointCloud,   False, SENSOR_QOS),
    ('odometry_conversion/odometry',                  Odometry,     False, DEFAULT_QOS),
    ('macvo/odometry',                                Odometry,     False, DEFAULT_QOS),
    ('global_plan',                                   Path,         False, DEFAULT_QOS),
    ('vdb_mapping/vdb_map_visualization',             Marker,       False, DEFAULT_QOS),
    ('droan/frustum',                                 Marker,       False, DEFAULT_QOS),
    ('droan/local_planner_global_plan_vis',           MarkerArray,  False, DEFAULT_QOS),
    ('droan/expansion_poly',                          MarkerArray,  False, DEFAULT_QOS),
    ('droan/disparity_map_debug',                     MarkerArray,  False, DEFAULT_QOS),
    ('droan/trajectory_library_vis',                  MarkerArray,  False, DEFAULT_QOS),
    ('droan/disparity_graph',                         MarkerArray,  False, DEFAULT_QOS),
    ('droan/traj_debug',                              MarkerArray,  False, DEFAULT_QOS),
    ('droan/graph_vis',                               MarkerArray,  False, DEFAULT_QOS),
    ('droan/virtual_obstacles',                       MarkerArray,  False, DEFAULT_QOS),
    ('droan/rewind_info',                             MarkerArray,  False, DEFAULT_QOS),
    ('trajectory_controller/trajectory_controller_debug_markers', MarkerArray, False, DEFAULT_QOS),
    ('trajectory_controller/trajectory_vis',          MarkerArray,  False, DEFAULT_QOS),
    ('/tf',                                           TFMessage,    True,  DEFAULT_QOS),
    ('/tf_static',                                    TFMessage,    True,  DEFAULT_QOS),
]


class TopicKeepalive(Node):
    def __init__(self):
        super().__init__('topic_keepalive')
        robot = os.getenv('ROBOT_NAME', 'robot_1')
        prefix = f'/{robot}/'
        self._subs = []
        for topic, msg_type, absolute, qos in TOPICS:
            full = topic if absolute else f'{prefix}{topic}'
            self._subs.append(
                self.create_subscription(msg_type, full, lambda _msg: None, qos))
        self.get_logger().info(
            f'topic_keepalive subscribed to {len(self._subs)} topics '
            f'(prefix={prefix})')


def main(args=None):
    rclpy.init(args=args)
    node = TopicKeepalive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
