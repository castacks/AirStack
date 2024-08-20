import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from stereo_msgs.msg import DisparityImage
from cv_bridge import CvBridge
import cv2
import numpy as np


def visualize_disparity_image(disparity_image):
    if disparity_image.size == 0:
        raise ValueError("Disparity image is empty.")
    cv2.imshow("Disparity Image", disparity_image)
    cv2.waitKey(1)


def visualize_depth_image(depth_image):
    if depth_image.size == 0:
        raise ValueError("Depth image is empty.")
    depth_image_normalized = cv2.normalize(depth_image, None, 0, 255, cv2.NORM_MINMAX)
    depth_image_uint8 = np.uint8(depth_image_normalized)
    cv2.imshow("Depth Image", depth_image_uint8)
    cv2.waitKey(1)


class DepthToDisparityNode(Node):
    def __init__(self):
        super().__init__("depth_to_disparity_node")

        # Declare and get parameters
        self.declare_parameter("baseline", 0.25)  # Baseline in meters
        self.declare_parameter("focal_length", 731.4286)

        self.baseline = (
            self.get_parameter("baseline").get_parameter_value().double_value
        )
        self.focal_length = (
            self.get_parameter("focal_length").get_parameter_value().double_value
        )

        self.get_logger().info(f"Baseline: {self.baseline} meters")
        self.get_logger().info(f"Focal length (fx): {self.focal_length} pixels")

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image, "/depth", self.listener_callback, 10
        )
        self.publisher = self.create_publisher(
            DisparityImage, "disparity/image_raw", 10
        )

    def listener_callback(self, msg):
        try:
            # Convert the ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        if cv_image.size == 0:
            self.get_logger().error("Received empty depth image. Raising an error.")
            raise ValueError("Depth image is empty.")

        disparity_image = self.depth_to_disparity(cv_image)

        disparity_msg = DisparityImage()
        disparity_msg.header = msg.header
        disparity_msg.image = self.bridge.cv2_to_imgmsg(
            disparity_image, encoding="32FC1"
        )
        disparity_msg.f = self.focal_length  # Focal length

        visualize_disparity_image(disparity_image)

        self.get_logger().info(f"Disparity image min: {np.min(disparity_image)}")
        # Publish the disparity image
        self.publisher.publish(disparity_msg)

    def depth_to_disparity(self, depth_image):
        if depth_image.size == 0:
            self.get_logger().error(
                "Depth image is empty in depth_to_disparity. Raising an error."
            )
            raise ValueError("Depth image is empty in depth_to_disparity.")

        depth_image = np.array(depth_image, dtype=np.float32)

        with np.errstate(divide="ignore", invalid="ignore"):
            # DISPARITY FORMULA
            disparity_image = (self.baseline * self.focal_length) / depth_image

            # Handle infinite and NaN values in the disparity image
            disparity_image[np.isinf(disparity_image)] = 0
            disparity_image[np.isnan(disparity_image)] = 0

        if disparity_image.size == 0:
            self.get_logger().error(
                "Disparity image is empty after processing. Raising an error."
            )
            raise ValueError("Disparity image is empty after processing.")

        return disparity_image


def main(args=None):
    rclpy.init(args=args)
    node = DepthToDisparityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
