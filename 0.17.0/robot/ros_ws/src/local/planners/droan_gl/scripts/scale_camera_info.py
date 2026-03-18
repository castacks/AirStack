#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import copy


class CameraInfoScaler(Node):
    def __init__(self):
        super().__init__("camera_info_scaler")

        # Parameters
        self.declare_parameter("input_topic", "/camera_info")
        self.declare_parameter("output_topic", "/camera_info_scaled")
        self.declare_parameter("scale_factor", 1.0)

        input_topic = (
            self.get_parameter("input_topic").get_parameter_value().string_value
        )
        output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.scale_factor = (
            self.get_parameter("scale_factor").get_parameter_value().double_value
        )

        # Publisher and Subscriber
        self.pub_ = self.create_publisher(CameraInfo, output_topic, 10)
        self.sub_ = self.create_subscription(CameraInfo, input_topic, self.callback, 10)

        self.get_logger().info(
            f"Scaling fx, fy by {self.scale_factor} from '{input_topic}' → '{output_topic}'"
        )

    def callback(self, msg: CameraInfo):
        scaled = copy.deepcopy(msg)  # CameraInfo()
        # scaled.header = msg.header
        # scaled.height = msg.height
        # scaled.width = msg.width
        # scaled.distortion_model = msg.distortion_model
        # scaled.d = list(msg.d)

        # Copy and modify intrinsic matrices
        scaled.k = list(msg.k)
        scaled.k[0] *= self.scale_factor  # fx
        scaled.k[4] *= self.scale_factor  # fy

        scaled.r = list(msg.r)
        scaled.p = list(msg.p)
        scaled.p[0] *= self.scale_factor  # fx
        scaled.p[5] *= self.scale_factor  # fy

        self.pub_.publish(scaled)


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoScaler()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
