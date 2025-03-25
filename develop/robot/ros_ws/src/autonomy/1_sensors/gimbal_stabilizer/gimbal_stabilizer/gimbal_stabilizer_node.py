import threading
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
# from transforms3d.euler import quat2euler
from std_msgs.msg import Float64  # Assuming the desired yaw is published as a Float64

class GimbalStabilizerNode(Node):
    def __init__(self):
        super().__init__('gimbal_stabilizer')
        
        # Publisher to send joint commands
        self.joint_pub = self.create_publisher(JointState, 'gimbal/joint_command', 10)
        
        # Subscriber to receive drone odometry
        self.create_subscription(Odometry, 'odometry_conversion/odometry', self.odometry_callback, 10)
        self.create_subscription(JointState, 'gimbal/joint_states', self.joint_callback, 10)
        self.create_subscription(Float64, 'gimbal/desired_gimbal_yaw', self.yaw_callback, 10)
        self.create_subscription(Float64, 'gimbal/desired_gimbal_pitch', self.pitch_callback, 10)

        # Initialize joint state message
        self.joint_command = JointState()
        self.joint_command.name = ["yaw_joint","roll_joint", "pitch_joint"]
        self.joint_command.position = [0.0, 0.0, 0.0]
        self.desired_yaw = 0.0
        self.desired_pitch = 0.0

    def yaw_callback(self, msg):
        self.desired_yaw = msg.data
        # self.get_logger().info(f"Received desired yaw angle: {self.desired_yaw}")

    def pitch_callback(self, msg):
        self.desired_pitch = msg.data

    def joint_callback(self, msg):
        self.got_joint_states = True
        # Inverse the drone angles to stabilize the gimbal
        # self.joint_command.position[0] = -roll  # roll joint
        # self.joint_command.position[1] = -pitch  # pitch joint
        # self.joint_command.position[2] = -yaw  # yaw joint

        # self.joint_command.effort = [100000000.0, 100000000.0, 100000000.0]

        # self.joint_command.position[0] = -20.0/180*3.14  # yaw joint
        # self.joint_command.position[1] = 10.0/180*3.14  # roll joint
        # self.joint_command.position[2] = 20.0/180*3.14  # pitch joint
        # self.joint_command.velocity = [float('nan'), float('nan'), float('nan')]
        # self.joint_command.velocity = [-1.0, -1.0, -1.0]

        # Publish the joint command
        # self.joint_pub.publish(self.joint_command)

    def odometry_callback(self, msg):
        # Extract quaternion from odometry message
        orientation_q = msg.pose.pose.orientation
        quaternion = [
            orientation_q.w,
            orientation_q.x,
            orientation_q.y,
            orientation_q.z
        ]

        # Convert quaternion to Euler angles (roll, pitch, yaw)
        # roll, pitch, yaw = quat2euler(quaternion, axes='sxyz')
        
        # Inverse the drone angles to stabilize the gimbal
        # self.joint_command.position[0] = -roll  # roll joint
        # self.joint_command.position[1] = -pitch  # pitch joint
        # self.joint_command.position[2] = -yaw  # yaw joint

        self.joint_command.position[0] = -self.desired_yaw/180*3.14  # yaw joint
        self.joint_command.position[1] = -0.0/180*3.14  # roll joint
        self.joint_command.position[2] = self.desired_pitch/180*3.14  # pitch joint
        self.joint_command.velocity = [float('nan'), float('nan'), float('nan')]
        # self.joint_command.velocity = [-1.0, -1.0, -1.0]

        # Publish the joint command
        self.joint_pub.publish(self.joint_command)

def main():
    rclpy.init()
    node = GimbalStabilizerNode()

    try:
        rclpy.spin(node)  # Run the node in a single thread
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()