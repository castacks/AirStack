"""Xbox controller teleop for the moving-obstacle drone.

Publishes geometry_msgs/TwistStamped (world-frame ENU velocity) on the swarm
commander's teleop topic, same as keyboard_teleop.py. Reads sensor_msgs/Joy
from the standard ROS 2 `joy` node (run `ros2 run joy joy_node` in another
terminal first).

Continuous, proportional stick input replaces keyboard_teleop's discrete
speed-step latching. The swarm commander does not know or care which node is
driving the teleop topic — as long as this drone is not also listed in
`exempt_drones`, its command passes through the same CBF filter as every
autonomous drone (see swarm_commander.py control_loop / teleop_command), so
a stick pushed straight at another drone still gets projected onto the safe
set before it reaches the vehicle.

Axis convention (matches keyboard_teleop.py's w/s a/d r/f):
    left stick vertical    : +x forward / -x back
    left stick horizontal  : +y left    / -y right
    right stick vertical   : +z up      / -z down
Default axis indices follow the common Linux xpad/joy_node Xbox mapping but
vary by driver — override via parameters if your pad reads differently
(echo `ros2 topic echo /joy` and wiggle one stick at a time to find yours).
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import Joy


class XboxTeleop(Node):

    def __init__(self):
        super().__init__('xbox_teleop')
        self.declare_parameter('drone', 'drone_3')
        self.declare_parameter('teleop_topic_template', '/svg/{name}/teleop_command')
        self.declare_parameter('joy_topic', '/joy')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('max_speed_mps', 1.0)
        self.declare_parameter('deadzone', 0.15)
        # Fail-safe: zero velocity if /joy goes quiet (disconnect, dead battery,
        # dropped USB dongle) instead of latching the last command forever.
        self.declare_parameter('joy_timeout_s', 0.5)

        self.declare_parameter('x_axis', 1)
        self.declare_parameter('y_axis', 0)
        self.declare_parameter('z_axis', 4)
        self.declare_parameter('x_sign', 1.0)
        self.declare_parameter('y_sign', 1.0)
        self.declare_parameter('z_sign', 1.0)

        self.max_speed  = float(self.get_parameter('max_speed_mps').value)
        self.deadzone   = float(self.get_parameter('deadzone').value)
        self.joy_timeout = float(self.get_parameter('joy_timeout_s').value)
        self.x_axis, self.y_axis, self.z_axis = (
            int(self.get_parameter('x_axis').value),
            int(self.get_parameter('y_axis').value),
            int(self.get_parameter('z_axis').value),
        )
        self.x_sign, self.y_sign, self.z_sign = (
            float(self.get_parameter('x_sign').value),
            float(self.get_parameter('y_sign').value),
            float(self.get_parameter('z_sign').value),
        )

        self.velocity = [0.0, 0.0, 0.0]
        self.last_joy_time = None

        topic = str(self.get_parameter('teleop_topic_template').value).format(
            name=str(self.get_parameter('drone').value))
        self.publisher = self.create_publisher(TwistStamped, topic, 10)
        self.create_subscription(Joy, str(self.get_parameter('joy_topic').value),
                                 self.joy_callback, 10)

        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.publish)

        self.get_logger().info(
            f'Xbox teleop on {self.publisher.topic_name}, reading '
            f'{self.get_parameter("joy_topic").value} — max_speed='
            f'{self.max_speed} m/s, deadzone={self.deadzone}')

    @staticmethod
    def _apply_deadzone(v: float, dz: float) -> float:
        if abs(v) < dz:
            return 0.0
        # Rescale so output still reaches ±1 at full deflection instead of
        # jumping from 0 to (1-dz) at the deadzone boundary.
        return (abs(v) - dz) / (1.0 - dz) * (1.0 if v > 0 else -1.0)

    def joy_callback(self, msg: Joy):
        self.last_joy_time = self.get_clock().now()

        def axis(idx, sign):
            if idx < 0 or idx >= len(msg.axes):
                return 0.0
            return sign * self._apply_deadzone(msg.axes[idx], self.deadzone)

        self.velocity = [
            axis(self.x_axis, self.x_sign) * self.max_speed,
            axis(self.y_axis, self.y_sign) * self.max_speed,
            axis(self.z_axis, self.z_sign) * self.max_speed,
        ]

    def publish(self):
        stale = (self.last_joy_time is None
                 or (self.get_clock().now() - self.last_joy_time).nanoseconds * 1e-9
                 > self.joy_timeout)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        vel = [0.0, 0.0, 0.0] if stale else self.velocity
        msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z = vel
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = XboxTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Final zero command so a Ctrl-C doesn't leave the last stick
        # deflection latched as the drone's nominal velocity.
        node.velocity = [0.0, 0.0, 0.0]
        node.publish()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
