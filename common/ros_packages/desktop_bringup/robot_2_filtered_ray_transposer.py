#!/usr/bin/env python3
import copy

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray


class Robot2FilteredRayTransposer(Node):
    def __init__(self):
        super().__init__('robot_2_filtered_ray_transposer')

        # Taken from simulation/isaac-sim/launch_scripts/two_drone_fire_academy.py
        #old fire academy init:
        # robot_1_init = (20.0, -7.0, 0.15)
        # robot_2_init = (17.0, 1.5, 0.15)
        #RetroNeighborhood init:
        # robot_1_init = (35.0, -19.0, 0.15)
        # robot_2_init = (30.0, -19.0, 0.15)

        robot_1_init = (25.0, 7.6, 0.15)
        robot_2_init = (23.4, 9.8, 0.15)

        dx = robot_2_init[0] - robot_1_init[0]
        dy = robot_2_init[1] - robot_1_init[1]
        dz = robot_2_init[2] - robot_1_init[2]

        self.declare_parameter('translation_x', dx)
        self.declare_parameter('translation_y', dy)
        self.declare_parameter('translation_z', dz)
        self.declare_parameter('color_r', 0.0)
        self.declare_parameter('color_g', 0.6)
        self.declare_parameter('color_b', 1.0)
        self.declare_parameter('color_a', 1.0)

        self.translation_x = float(self.get_parameter('translation_x').value)
        self.translation_y = float(self.get_parameter('translation_y').value)
        self.translation_z = float(self.get_parameter('translation_z').value)
        self.color_r = float(self.get_parameter('color_r').value)
        self.color_g = float(self.get_parameter('color_g').value)
        self.color_b = float(self.get_parameter('color_b').value)
        self.color_a = float(self.get_parameter('color_a').value)

        self.subscription = self.create_subscription(
            MarkerArray,
            '/robot_2/filtered_rays',
            self.callback,
            10,
        )
        self.publisher = self.create_publisher(
            MarkerArray,
            '/robot_2/filtered_rays/transposed',
            10,
        )

        self.get_logger().info(
            f'Publishing robot_2 transposed rays with translation '
            f'({self.translation_x:.3f}, {self.translation_y:.3f}, {self.translation_z:.3f})'
        )

    def callback(self, msg: MarkerArray):
        out = copy.deepcopy(msg)

        for marker in out.markers:
            # Opposite direction from robot_1_semantic_ray_transposer.py:
            # apply +translation instead of -translation.
            marker.pose.position.x += self.translation_x
            marker.pose.position.y += self.translation_y
            marker.pose.position.z += self.translation_z
            marker.color.r = self.color_r
            marker.color.g = self.color_g
            marker.color.b = self.color_b
            marker.color.a = self.color_a

            for point in marker.points:
                point.x += self.translation_x
                point.y += self.translation_y
                point.z += self.translation_z

            for color in marker.colors:
                color.r = self.color_r
                color.g = self.color_g
                color.b = self.color_b
                color.a = self.color_a

        self.publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = Robot2FilteredRayTransposer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
