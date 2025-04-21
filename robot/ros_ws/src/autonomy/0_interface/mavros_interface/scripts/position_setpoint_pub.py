#!/usr/bin/python3
import os

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.executors import MultiThreadedExecutor, Executor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from airstack_msgs.msg import Odometry
from nav_msgs.msg import Odometry as NavOdometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, TwistStamped
from scipy.spatial.transform import Rotation as R
import numpy as np
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import time

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

        self.declare_parameter('publish_goal', False)
        self.publish_goal = self.get_parameter('publish_goal').get_parameter_value().bool_value
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.odom_subscriber = self.create_subscription(NavOdometry, "/" + os.getenv('ROBOT_NAME', "") + '/odometry_conversion/odometry', self.odom_callback, 1)
        self.projected_pose_subsriber = self.create_subscription(PoseStamped, '/' + os.getenv('ROBOT_NAME', '') + '/trajectory_controller/projected_drone_pose', self.projected_pose_callback, 1)

        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.tracking_point_subscriber = self.create_subscription(Odometry, "/" + os.getenv('ROBOT_NAME', "") + '/trajectory_controller/tracking_point', self.tracking_point_callback, 1, callback_group=self.callback_group)
        self.odom_publisher = self.create_publisher(PoseStamped, 'cmd_pose', 1)
        self.vel_publisher = self.create_publisher(TwistStamped, 'cmd_velocity', 1)
        self.path_publisher = self.create_publisher(Path, "/" + os.getenv('ROBOT_NAME', "") + '/global_plan', 1)
        self.cross_track_publisher = self.create_publisher(PoseStamped, "/" + os.getenv('ROBOT_NAME', "") + '/cross_track_error', 1)

        #self.odom_pos = [0., 0., 0.]
        self.odom = None
        self.projected_pose = None

        self.path = Path()
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path.header.frame_id = 'map'
        pose1 = PoseStamped()
        pose1.header = self.path.header
        pose1.pose.position.x = 50.0
        pose1.pose.position.y = 0.0
        pose1.pose.position.z = 15.0
        pose1.pose.orientation.w = 1.0
        pose2 = PoseStamped()
        pose2.header = self.path.header
        pose2.pose.position.x = 60.0
        pose2.pose.position.y = 0.0
        pose2.pose.position.z = 15.0
        pose2.pose.orientation.w = 1.0
        self.path.poses.append(pose1)
        self.path.poses.append(pose2)

    def time_diff(self, t1, t2):
        return (t1.sec + t1.nanosec*1e-9) - (t2.sec + t2.nanosec*1e-9)

    def get_yaw(self, q):
        yaw = R.from_quat([q.x, q.y, q.z, q.w])
        yaw = yaw.as_euler('xyz')
        return yaw[2]

    def odom_callback(self, msg):
        '''
        if self.odom == None:
            q = R.from_quat([msg.pose.pose.orientation.x,
                             msg.pose.pose.orientation.y,
                             msg.pose.pose.orientation.z,
                             msg.pose.pose.orientation.w])
            p0 = np.array([self.path.poses[0].pose.position.x,
                           self.path.poses[0].pose.position.y,
                           self.path.poses[0].pose.position.z])
            p1 = np.array([self.path.poses[1].pose.position.x,
                           self.path.poses[1].pose.position.y,
                           self.path.poses[1].pose.position.z])
            p0 = q.apply(p0)
            p1 = q.apply(p1)

            self.path.poses[0].pose.position.x = p0[0]
            self.path.poses[0].pose.position.y = p0[1]
            self.path.poses[0].pose.position.z = p0[2]
            self.path.poses[1].pose.position.x = p1[0]
            self.path.poses[1].pose.position.y = p1[1]
            self.path.poses[1].pose.position.z = p1[2]
        '''
        self.odom = msg
        
        #self.odom_pos = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
        #self.odom_yaw = self.get_yaw(msg.pose.pose.orientation)

    def projected_pose_callback(self, msg):
        self.projected_pose = msg

    def get_tf(self, target_frame, source_frame, lookup_time, timeout=Duration(nanoseconds=0)):
        exception = False
        try:
            t = self.tf_buffer.lookup_transform(target_frame, source_frame, lookup_time)
            return t
        except tf2_ros.TransformException as ex:
            self.get_logger().info(
                f'Could not transform {target_frame} to {source_frame}: {ex}')
            exception = True

        start_time = time.time()
        if exception:
            try:
                t = self.tf_buffer.lookup_transform(target_frame, source_frame, lookup_time, timeout)
                self.get_logger().info('elapsed %f' % (time.time() - start_time))
                return t
            except tf2_ros.TransformException as ex:
                self.get_logger().info(
                    f'Attempt 2: {target_frame} to {source_frame}: {ex}')
                self.get_logger().info('elapsed %f' % (time.time() - start_time))
        return None

    def tracking_point_callback(self, msg):
        if self.odom == None:
            return

        if self.publish_goal:
            self.path.header.stamp = self.get_clock().now().to_msg()
            self.path.poses[0].header.stamp = self.path.header.stamp
            self.path.poses[1].header.stamp = self.path.header.stamp
            self.path_publisher.publish(self.path)
        
        if self.command_pose:
            out = PoseStamped()
            out.header = msg.header
            out.header.frame_id = 'base_link'
            out.pose = msg.pose
            self.odom_publisher.publish(out)
        else:
            t = self.get_tf(self.target_frame, msg.header.frame_id, rclpy.time.Time(seconds=0, nanoseconds=0),
                            Duration(seconds=2, nanoseconds=0))
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

            cross_track_x = 0.
            cross_track_y = 0.
            cross_track_z = 0.
            if self.projected_pose != None and abs(self.time_diff(self.odom.header.stamp, self.projected_pose.header.stamp)) < 0.5:
                #pose_t = self.get_tf(self.target_frame, self.projected_pose.header.frame_id, self.projected_pose.heaer.stamp)
                cross_track_x = self.projected_pose.pose.position.x - self.odom.pose.pose.position.x
                cross_track_y = self.projected_pose.pose.position.y - self.odom.pose.pose.position.y
                cross_track_z = self.projected_pose.pose.position.z - self.odom.pose.pose.position.z
            self.get_logger().info('cross track: ' + str(cross_track_x) + ' ' + str(cross_track_y) + ' ' + str(cross_track_z))
            ct = PoseStamped()
            ct.header = self.odom.header
            ct.pose.position.x = cross_track_x
            ct.pose.position.y = cross_track_y
            ct.pose.position.z = cross_track_z
            self.cross_track_publisher.publish(ct)
                

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
            out.twist.linear.x = vel[0]# + cross_track_x#msg.pose.position.x - self.odom_pos[0]
            out.twist.linear.y = vel[1]# + cross_track_y#msg.pose.position.y - self.odom_pos[1]
            out.twist.linear.z = vel[2]*0.5# + cross_track_z#msg.pose.position.z - self.odom_pos[2]
            out.twist.angular.z = yawrate
            self.vel_publisher.publish(out)

if __name__ == '__main__':
    rclpy.init(args=None)
    odom_modifier_node = OdomModifier()

    executor = MultiThreadedExecutor()
    executor.add_node(odom_modifier_node)
    executor.spin()
    
    #rclpy.spin(odom_modifier_node)
    #odom_modifier_node.destroy_node()
    rclpy.shutdown()
