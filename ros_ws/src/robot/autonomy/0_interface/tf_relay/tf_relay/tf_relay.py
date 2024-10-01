import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


class TFRelay(Node):

    def __init__(self, default_namespace="robot_1"):
        super().__init__("tf_relay_" + default_namespace)

        # the parent namespace preceding /tf
        self.declare_parameter("namespace", default_namespace)
        namespace = self.get_parameter("namespace").get_parameter_value().string_value

        # the output tf_prefix that gets prepended to frame IDs
        self.declare_parameter("frame_prefix", namespace)
        frame_prefix = (
            self.get_parameter("frame_prefix").get_parameter_value().string_value
        )

        tf_topic = "/" + str(namespace) + "/tf"
        self.frame_prefix = frame_prefix + "/"
        self.subscription = self.create_subscription(
            msg_type=TFMessage,
            topic=tf_topic,
            callback=self.tf_callback,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=10,
            ),
        )

        self.publisher = self.create_publisher(
            msg_type=TFMessage,
            topic="/tf",
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.VOLATILE,
                depth=10,
            ),
        )

    def tf_callback(self, msg):
        for transform in msg.transforms:
            transform.header.frame_id = self.frame_prefix + transform.header.frame_id
            transform.child_frame_id = self.frame_prefix + transform.child_frame_id
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    tf_relay = TFRelay()

    rclpy.spin(tf_relay)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
