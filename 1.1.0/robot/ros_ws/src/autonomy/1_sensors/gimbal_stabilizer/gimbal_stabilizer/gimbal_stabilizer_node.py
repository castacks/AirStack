import threading
import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from transforms3d.euler import quat2euler

class GimbalStabilizerNode(Node):
    def __init__(self):
        super().__init__('gimbal_stabilizer')
        
        # Publisher to send joint commands
        self.joint_pub = self.create_publisher(JointState, '/joint_command', 10)
        
        # Subscriber to receive drone odometry
        self.create_subscription(Odometry, '/robot_1/odometry_conversion/odometry', self.odometry_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

        # Initialize joint state message
        self.joint_command = JointState()
        self.joint_command.name = ["yaw_joint","roll_joint", "pitch_joint"]
        self.joint_command.position = [0.0, 0.0, 0.0]

    def joint_callback(self, msg):
        # Inverse the drone angles to stabilize the gimbal
        # self.joint_command.position[0] = -roll  # roll joint
        # self.joint_command.position[1] = -pitch  # pitch joint
        # self.joint_command.position[2] = -yaw  # yaw joint

        # self.joint_command.effort = [100000000.0, 100000000.0, 100000000.0]

        # self.joint_command.position[0] = -20.0/180*3.14  # yaw joint
        # self.joint_command.position[1] = 10.0/180*3.14  # roll joint
        # self.joint_command.position[2] = 20.0/180*3.14  # pitch joint
        self.joint_command.velocity = [float('nan'), float('nan'), float('nan')]
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
        roll, pitch, yaw = quat2euler(quaternion, axes='sxyz')
        
        # Inverse the drone angles to stabilize the gimbal
        # self.joint_command.position[0] = -roll  # roll joint
        # self.joint_command.position[1] = -pitch  # pitch joint
        # self.joint_command.position[2] = -yaw  # yaw joint

        self.joint_command.position[0] = -0.0/180*3.14  # yaw joint
        self.joint_command.position[1] = -roll  # roll joint
        self.joint_command.position[2] = 0.0/180*3.14  # pitch joint
        self.joint_command.velocity = [float('nan'), float('nan'), float('nan')]
        self.joint_command.velocity = [-1.0, -1.0, -1.0]

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
