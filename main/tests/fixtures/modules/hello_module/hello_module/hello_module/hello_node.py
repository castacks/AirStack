#!/usr/bin/env python3
"""Tiny rclpy node: logs one greeting, then exits."""

import rclpy
from rclpy.node import Node


class HelloNode(Node):
    def __init__(self):
        super().__init__("hello_node")
        self.get_logger().info("hello_module says hello")


def main(args=None):
    rclpy.init(args=args)
    node = HelloNode()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
