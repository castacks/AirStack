import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class LocalPositionToOdom(Node):
    def __init__(self):
        super().__init__("local_position_to_odom")

        self.input_topic = self.declare_parameter(
            "input_topic", "/fmu/out/vehicle_local_position"
        ).value
        self.output_topic = self.declare_parameter(
            "output_topic", "/fmu/out/vehicle_local_position_odom"
        ).value
        self.frame_id = self.declare_parameter("frame_id", "px4_local_ned").value
        self.child_frame_id = self.declare_parameter("child_frame_id", "base_link_frd").value
        self.valid_covariance = float(
            self.declare_parameter("valid_covariance", 1.0).value
        )
        self.invalid_covariance = float(
            self.declare_parameter("invalid_covariance", 0.0).value
        )

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(
            VehicleLocalPosition,
            self.input_topic,
            self.callback,
            qos,
        )
        self.pub = self.create_publisher(Odometry, self.output_topic, qos)
        self.msg_count = 0
        self.wait_timer = self.create_timer(5.0, self.report_waiting)

        self.get_logger().info(
            "Publishing {} from {} (frame_id={}, child_frame_id={})".format(
                self.output_topic,
                self.input_topic,
                self.frame_id,
                self.child_frame_id,
            )
        )

    def fill_covariance(self, valid):
        value = self.valid_covariance if valid else self.invalid_covariance
        return [value] * 36

    def report_waiting(self):
        if self.msg_count == 0:
            self.get_logger().info(
                "Still waiting for messages on {} ...".format(self.input_topic)
            )

    def callback(self, msg):
        self.msg_count += 1

        if self.msg_count == 1:
            self.wait_timer.cancel()
            self.get_logger().info(
                "Received first VehicleLocalPosition message: "
                "x={:.3f}, y={:.3f}, z={:.3f}, vx={:.3f}, vy={:.3f}, vz={:.3f}".format(
                    msg.x, msg.y, msg.z, msg.vx, msg.vy, msg.vz
                )
            )

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = float(msg.x)
        odom.pose.pose.position.y = float(msg.y)
        odom.pose.pose.position.z = float(msg.z)

        # This bridge is for inspecting xyz / vxyz only, so keep orientation neutral.
        odom.pose.pose.orientation.w = 1.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0

        odom.twist.twist.linear.x = float(msg.vx)
        odom.twist.twist.linear.y = float(msg.vy)
        odom.twist.twist.linear.z = float(msg.vz)

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = 0.0

        odom.pose.covariance = self.fill_covariance(bool(msg.xy_valid))
        odom.twist.covariance = self.fill_covariance(bool(msg.v_xy_valid))

        self.pub.publish(odom)

        if self.msg_count % 50 == 0:
            self.get_logger().info(
                "Published {} odom msgs | pos=({:.3f}, {:.3f}, {:.3f}) "
                "vel=({:.3f}, {:.3f}, {:.3f}) xy_valid={} vxy_valid={}".format(
                    self.msg_count,
                    msg.x,
                    msg.y,
                    msg.z,
                    msg.vx,
                    msg.vy,
                    msg.vz,
                    bool(msg.xy_valid),
                    bool(msg.v_xy_valid),
                )
            )


def main(args=None):
    rclpy.init(args=args)
    node = LocalPositionToOdom()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
