#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
import cv2

from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from std_msgs.msg import Header
from sensor_msgs.msg import RegionOfInterest
from cv_bridge import CvBridge

class DisparityPublisher(Node):
    def __init__(self):
        super().__init__('disparity_publisher')

        self.set_parameters([rclpy.parameter.Parameter(
            'use_sim_time',
            rclpy.parameter.Parameter.Type.BOOL,
            True
        )])

        # Publisher for disparity image
        self.pub = self.create_publisher(DisparityImage, '/fake_disparity', 10)

        # Timer to publish at 10 Hz
        self.timer = self.create_timer(1.0, self.publish_disparity)

        self.bridge = CvBridge()
        self.frame_id = "front_stereo_left_camera_optical_frame"

        # Parameters you can change
        self.width = 480
        self.height = 270
        self.disparity_value = -2.#float('nan')   # Example constant disparity everywhere
        self.f = 233.8268585205078                # Focal length in pixels
        self.t = 0.12                  # Baseline in meters

        self.count = 0

    def publish_disparity(self):
        # Create a numpy array filled with the disparity value
        disparity_array = np.full((self.height, self.width), self.disparity_value, dtype=np.float32)

        gap = 50
        #disparity_array[gap:(self.height-gap), int(self.width/2)] = 1.
        disparity_array[int(self.height/2), int(self.width/2)] = 1.

        # Convert to ROS Image
        disparity_img = self.bridge.cv2_to_imgmsg(disparity_array, encoding="32FC1")

        # Fill header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id

        # Create DisparityImage message
        disp_msg = DisparityImage()
        disp_msg.header = header
        disp_msg.image = disparity_img
        disp_msg.f = self.f
        disp_msg.t = self.t
        disp_msg.min_disparity = 0.0
        disp_msg.max_disparity = 63.0
        disp_msg.delta_d = 1.0 / 16.0
        disp_msg.valid_window.x_offset = 70
        disp_msg.valid_window.y_offset = 7
        disp_msg.valid_window.height = 4294967281
        disp_msg.valid_window.width = 4294967281

        # Publish
        self.pub.publish(disp_msg)
        self.get_logger().info("Published disparity image with value %.2f" % self.disparity_value)
        
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = DisparityPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
