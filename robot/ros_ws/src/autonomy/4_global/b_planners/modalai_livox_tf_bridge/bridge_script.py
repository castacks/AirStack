import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class OdometryBridge(Node):
    def __init__(self):
        super().__init__('odometry_bridge')

        # PX4 expects reliable QoS
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        # Subscriber
        self.sub = self.create_subscription(
            Odometry,
            '/laser_odometry',
            self.odom_callback,
            qos
        )

        # Publishers
        self.pub_visual = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_visual_odometry',
            qos
        )

        self.pub_mocap = self.create_publisher(
            VehicleOdometry,
            '/fmu/in/vehicle_mocap_odometry',
            qos
        )

    def odom_callback(self, msg: Odometry):
        vo = VehicleOdometry()

        # -----------------------------
        # ✅ Timestamp (CRITICAL)
        # -----------------------------
        now_us = int(self.get_clock().now().nanoseconds / 1000)
        vo.timestamp = now_us
        vo.timestamp_sample = now_us

        # -----------------------------
        # ✅ ENU → NED conversion
        # -----------------------------
        # Position
        vo.position[0] = msg.pose.pose.position.y*0.870 + msg.pose.pose.position.z*0.492
        vo.position[1] = -msg.pose.pose.position.x
        vo.position[2] = 0.492*msg.pose.pose.position.z - msg.pose.pose.position.y*0.870

        # Orientation (quaternion)
        vo.q[0] = msg.pose.pose.orientation.w
        vo.q[1] = msg.pose.pose.orientation.y
        vo.q[2] = -msg.pose.pose.orientation.x
        vo.q[3] = msg.pose.pose.orientation.z

        # Linear velocity
        vo.velocity[0] = msg.twist.twist.linear.y
        vo.velocity[1] = -msg.twist.twist.linear.x
        vo.velocity[2] = msg.twist.twist.linear.z

        # Angular velocity
        vo.angular_velocity[0] = msg.twist.twist.angular.y
        vo.angular_velocity[1] = -msg.twist.twist.angular.x
        vo.angular_velocity[2] = msg.twist.twist.angular.z

        # -----------------------------
        # ✅ Required PX4 fields
        # -----------------------------
        vo.pose_frame = VehicleOdometry.POSE_FRAME_NED
        vo.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED

        # Variances (non-zero!)
        vo.position_variance = [0.01, 0.01, 0.01]
        vo.orientation_variance = [0.01, 0.01, 0.01]
        vo.velocity_variance = [0.01, 0.01, 0.01]

        # Quality (0–100)
        vo.quality = 100

        # -----------------------------
        # Publish
        # -----------------------------
        self.pub_visual.publish(vo)
        self.pub_mocap.publish(vo)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
