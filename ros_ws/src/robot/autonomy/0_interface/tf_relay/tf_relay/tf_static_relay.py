import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy


class TFStaticRelay(Node):

    def __init__(self, default_namespace="robot_1"):
        super().__init__("tf_static_relay_" + default_namespace)

        # the parent namespace preceding /tf
        self.declare_parameter("namespace", default_namespace)
        namespace = self.get_parameter("namespace").get_parameter_value().string_value

        # the output tf_prefix that gets prepended to frame IDs
        self.declare_parameter("frame_prefix", namespace)
        frame_prefix = (
            self.get_parameter("frame_prefix").get_parameter_value().string_value
        )

        tf_static_topic = "/" + str(namespace) + "/tf_static"
        self.frame_prefix = frame_prefix + "/"
        self.static_subscription = self.create_subscription(
            msg_type=TFMessage,
            topic=tf_static_topic,
            callback=self.static_tf_callback,
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=10,
            ),
        )

        self.static_publisher = self.create_publisher(
            msg_type=TFMessage,
            topic="/tf_static",
            qos_profile=QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                depth=10,
            ),
        )

    def static_tf_callback(self, msg):
        for transform in msg.transforms:
            transform.header.frame_id = self.frame_prefix + transform.header.frame_id
            transform.child_frame_id = self.frame_prefix + transform.child_frame_id
        self.static_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    tf_static_relay = TFStaticRelay()

    rclpy.spin(tf_static_relay)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
