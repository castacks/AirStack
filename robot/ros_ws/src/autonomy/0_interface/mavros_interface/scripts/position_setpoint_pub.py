#!/usr/bin/python3
import os

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from airstack_msgs.msg import Odometry
from nav_msgs.msg import Odometry as NavOdometry
from geometry_msgs.msg import PoseStamped, TwistStamped
from scipy.spatial.transform import Rotation as R
import numpy as np
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class OdomModifier(Node):
    def __init__(self):
        super().__init__('odom_modifier')
        '''
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
            history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
            depth=1)
        '''

        self.declare_parameter('command_pose', True)
        self.command_pose = self.get_parameter('command_pose').get_parameter_value().bool_value
        
        self.declare_parameter('max_velocity', 0.5)
        self.max_velocity = self.get_parameter('max_velocity').get_parameter_value().double_value

        self.declare_parameter('target_frame', 'base_link')
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.odom_subscriber = self.create_subscription(NavOdometry, "/" + os.getenv('ROBOT_NAME', "") + '/odometry_conversion/odometry', self.odom_callback, 1)
        self.tracking_point_subscriber = self.create_subscription(Odometry, "/" + os.getenv('ROBOT_NAME', "") + '/trajectory_controller/tracking_point', self.tracking_point_callback, 1)
        self.odom_publisher = self.create_publisher(PoseStamped, 'cmd_pose', 1)
        self.vel_publisher = self.create_publisher(TwistStamped, 'cmd_velocity', 1)

        #self.odom_pos = [0., 0., 0.]
        self.odom = None

    def get_yaw(self, q):
        yaw = R.from_quat([q.x, q.y, q.z, q.w])
        yaw = yaw.as_euler('xyz')
        return yaw[2]

    def odom_callback(self, msg):
        self.odom = msg
        #self.odom_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        #self.odom_yaw = self.get_yaw(msg.pose.pose.orientation)

    def get_tf(self, target_frame, source_frame, time):
        try:
            t = self.tf_buffer.lookup_transform(target_frame, source_frame, time)
            return t
        except tf2_ros.TransformException as ex:
            self.get_logger().info(
                f'Could not transform {target_frame} to {source_frame}: {ex}')
        return None

    def tracking_point_callback(self, msg):
        if self.odom == None:
            return
        
        if self.command_pose:
            out = PoseStamped()
            out.header = msg.header
            out.header.frame_id = 'base_link'
            out.pose = msg.pose
            self.odom_publisher.publish(out)
        else:
            t = self.get_tf(self.target_frame, msg.header.frame_id, rclpy.time.Time(seconds=0, nanoseconds=0))
            if t == None:
                return

            rot = R.from_quat([t.transform.rotation.x,
                               t.transform.rotation.y,
                               t.transform.rotation.z,
                               t.transform.rotation.w])
            vel = np.array([msg.pose.position.x - self.odom.pose.pose.position.x,
                            msg.pose.position.y - self.odom.pose.pose.position.y,
                            msg.pose.position.z - self.odom.pose.pose.position.z])
            vel = rot.apply(vel)
            mag = np.linalg.norm(vel)
            if mag > self.max_velocity:
                vel = vel/mag * self.max_velocity

            tracking_point_q = rot*R.from_quat([msg.pose.orientation.x,
                                                msg.pose.orientation.y,
                                                msg.pose.orientation.z,
                                                msg.pose.orientation.w])
            tracking_point_yaw_tf = tracking_point_q.as_euler('xyz')[2]
            tracking_point_yaw = self.get_yaw(msg.pose.orientation)
            #self.get_logger().info('tp: ' + str(tracking_point_yaw) + ' ' + str(tracking_point_q.as_euler('xyz')[2]))

            odom_yaw = self.get_yaw(self.odom.pose.pose.orientation)
            odom_t = self.get_tf(self.target_frame, self.odom.header.frame_id, self.odom.header.stamp)
            if odom_t == None:
                return
            odom_rot = R.from_quat([odom_t.transform.rotation.x,
                                    odom_t.transform.rotation.y,
                                    odom_t.transform.rotation.z,
                                    odom_t.transform.rotation.w])
            odom_q = R.from_quat([self.odom.pose.pose.orientation.x,
                                  self.odom.pose.pose.orientation.y,
                                  self.odom.pose.pose.orientation.z,
                                  self.odom.pose.pose.orientation.w])
            odom_yaw_tf = (odom_rot*odom_q).as_euler('xyz')[2]
            #self.get_logger().info('odom: ' + str(odom_yaw) + ' ' + str(odom_yaw_tf))
            #self.get_logger().info(str(odom_rot.as_euler('xyz')[2]) + ' ' + str(odom_q.as_euler('xyz')[2]))
            
            yawrate = np.arctan2(np.sin(tracking_point_yaw_tf - odom_yaw_tf), np.cos(tracking_point_yaw_tf - odom_yaw_tf))
            
            out = TwistStamped()
            out.header = msg.header
            out.header.frame_id = self.target_frame
            out.twist.linear.x = vel[0]#msg.pose.position.x - self.odom_pos[0]
            out.twist.linear.y = vel[1]#msg.pose.position.y - self.odom_pos[1]
            out.twist.linear.z = vel[2]#msg.pose.position.z - self.odom_pos[2]
            out.twist.angular.z = yawrate
            self.vel_publisher.publish(out)

if __name__ == '__main__':
    rclpy.init(args=None)
    odom_modifier_node = OdomModifier()
    rclpy.spin(odom_modifier_node)
    odom_modifier_node.destroy_node()
    rclpy.shutdown()
