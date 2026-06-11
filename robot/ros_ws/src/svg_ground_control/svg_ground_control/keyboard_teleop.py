"""Keyboard teleop for the moving-obstacle drone.

Publishes geometry_msgs/TwistStamped (world-frame ENU velocity) on the swarm
commander's teleop topic at a fixed rate. Run it in its own terminal — it
puts the TTY in raw mode.

Keys (velocity setpoints latch until changed):
    w / s : +x / -x        a / d : +y / -y
    r / f : +z / -z        space : stop (zero velocity)
    + / - : increase / decrease speed step
    q     : quit (publishes a final zero command)
"""

import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped


class KeyboardTeleop(Node):

    def __init__(self):
        super().__init__('keyboard_teleop')
        # Which drone to drive; resolves the topic via teleop_topic_template
        # (must match the swarm commander's template).
        self.declare_parameter('drone', 'drone_3')
        self.declare_parameter('teleop_topic_template', '/svg/{name}/teleop_command')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('speed_step_mps', 0.4)

        self.speed = float(self.get_parameter('speed_step_mps').value)
        self.velocity = [0.0, 0.0, 0.0]
        self.lock = threading.Lock()
        self.running = True

        topic = str(self.get_parameter('teleop_topic_template').value).format(
            name=str(self.get_parameter('drone').value))
        self.publisher = self.create_publisher(TwistStamped, topic, 10)
        rate = float(self.get_parameter('publish_rate_hz').value)
        self.timer = self.create_timer(1.0 / rate, self.publish)

        self.key_thread = threading.Thread(target=self.key_loop, daemon=True)
        self.key_thread.start()
        self.get_logger().info(
            f'Keyboard teleop on {self.publisher.topic_name} — '
            'w/s a/d r/f to move, space to stop, +/- speed, q to quit')

    def publish(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        with self.lock:
            msg.twist.linear.x = self.velocity[0]
            msg.twist.linear.y = self.velocity[1]
            msg.twist.linear.z = self.velocity[2]
        self.publisher.publish(msg)

    def key_loop(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while self.running:
                key = sys.stdin.read(1)
                with self.lock:
                    if key == 'w':
                        self.velocity[0] = self.speed
                    elif key == 's':
                        self.velocity[0] = -self.speed
                    elif key == 'a':
                        self.velocity[1] = self.speed
                    elif key == 'd':
                        self.velocity[1] = -self.speed
                    elif key == 'r':
                        self.velocity[2] = self.speed
                    elif key == 'f':
                        self.velocity[2] = -self.speed
                    elif key == ' ':
                        self.velocity = [0.0, 0.0, 0.0]
                    elif key == '+':
                        self.speed = min(self.speed + 0.1, 2.0)
                    elif key == '-':
                        self.speed = max(self.speed - 0.1, 0.1)
                    elif key in ('q', '\x03'):  # q or Ctrl-C
                        self.velocity = [0.0, 0.0, 0.0]
                        self.running = False
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def is_running(self):
        return self.running


def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    try:
        while rclpy.ok() and node.is_running():
            rclpy.spin_once(node, timeout_sec=0.1)
        node.publish()  # final zero command
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
