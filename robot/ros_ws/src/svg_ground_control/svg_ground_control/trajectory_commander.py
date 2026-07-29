"""Scripted trajectory commander for SVG ground control.

This node generates scripted world-frame goals. The default backend is the SVG
goal topic consumed by swarm_commander; an experimental FMU pose backend
publishes directly to px4_interface position mode.
"""

from __future__ import annotations

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Float32
from std_srvs.srv import Trigger

from svg_ground_control.trajectory_commander_core import (
    ScriptedTrajectory,
    clamp_position,
    vector3,
)


class TrajectoryCommander(Node):
    def __init__(self):
        super().__init__('trajectory_commander')

        self.declare_parameter('drone', 'drone_2')
        self.declare_parameter('backend', 'svg_goal')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('bounds_min', [-3.0, -3.0, 0.3])
        self.declare_parameter('bounds_max', [3.0, 3.0, 2.0])
        self.declare_parameter('odom_topic_template',
                               '/{name}/odometry_conversion/odometry')

        self.declare_parameter('script_type', 'circle')
        self.declare_parameter('script_auto_start', True)
        self.declare_parameter('center_mode', 'current_at_start')
        self.declare_parameter('center', [0.0, 0.0, 1.1])
        self.declare_parameter('center_offset', [0.0, 0.0, 0.0])
        self.declare_parameter('radius_m', 0.25)
        self.declare_parameter('amplitude', [0.25, 0.15])
        self.declare_parameter('period_s', 16.0)
        self.declare_parameter('waypoints',
                               [0.0, 0.0, 1.0, 1.0, 0.0, 1.0])
        self.declare_parameter('waypoints_relative_to_center', False)
        self.declare_parameter('segment_durations', [2.0])
        self.declare_parameter('loop', True)

        self.declare_parameter('svg_goal_topic_template',
                               '/svg/{name}/goal_command')
        self.declare_parameter('svg_speed_topic_template',
                               '/svg/{name}/speed_command')
        self.declare_parameter('fmu_pose_topic_template',
                               '/{name}/fmu/pose_command')
        self.declare_parameter('speed_mps', 0.3)
        self.declare_parameter('log_goal_updates', True)
        self.declare_parameter('log_goal_period_s', 1.0)

        self.drone = str(self.get_parameter('drone').value)
        self.backend = str(self.get_parameter('backend').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        if self.backend not in ('svg_goal', 'fmu_pose'):
            raise ValueError('backend must be svg_goal or fmu_pose')

        self.bounds_min = vector3(self.get_parameter('bounds_min').value, 'bounds_min')
        self.bounds_max = vector3(self.get_parameter('bounds_max').value, 'bounds_max')
        if np.any(self.bounds_min >= self.bounds_max):
            raise ValueError('bounds_min must be strictly less than bounds_max')

        self.latest_position = None
        self.script_center = None
        self.script_start_time = None
        self.script_running = bool(
            self.get_parameter('script_auto_start').value)
        self.last_goal_log_time = None
        self.log_goal_updates = bool(self.get_parameter('log_goal_updates').value)
        self.log_goal_period_s = float(self.get_parameter('log_goal_period_s').value)

        odom_topic = str(self.get_parameter('odom_topic_template').value).format(
            name=self.drone)
        self.create_subscription(Odometry, odom_topic, self.odom_callback, 10)
        self._maybe_init_fixed_script()
        self.create_service(Trigger, '~/start', self.handle_script_start)

        if self.backend == 'svg_goal':
            goal_topic = str(
                self.get_parameter('svg_goal_topic_template').value).format(
                    name=self.drone)
            speed_topic = str(
                self.get_parameter('svg_speed_topic_template').value).format(
                    name=self.drone)
            self.goal_pub = self.create_publisher(PoseStamped, goal_topic, 10)
            self.speed_pub = self.create_publisher(Float32, speed_topic, 10)
            self.goal_topic = goal_topic
        else:
            pose_topic = str(
                self.get_parameter('fmu_pose_topic_template').value).format(
                    name=self.drone)
            self.goal_pub = self.create_publisher(PoseStamped, pose_topic, 10)
            self.speed_pub = None
            self.goal_topic = pose_topic

        rate = float(self.get_parameter('publish_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)

        self.get_logger().info(
            f'TrajectoryCommander up | drone={self.drone} '
            f'backend={self.backend} odom={odom_topic}')

    def _maybe_init_fixed_script(self):
        center_mode = str(self.get_parameter('center_mode').value)
        if center_mode not in ('current_at_start', 'fixed'):
            raise ValueError('center_mode must be current_at_start or fixed')
        if center_mode == 'fixed':
            self._start_script(vector3(self.get_parameter('center').value, 'center'))

    def _start_script(self, center):
        self.script_center = clamp_position(center, self.bounds_min, self.bounds_max)
        waypoints = np.asarray(
            self.get_parameter('waypoints').value, dtype=float).reshape(-1, 3)
        if bool(self.get_parameter('waypoints_relative_to_center').value):
            waypoints = waypoints + self.script_center
        self.script = ScriptedTrajectory(
            script_type=str(self.get_parameter('script_type').value),
            center=self.script_center,
            radius_m=float(self.get_parameter('radius_m').value),
            amplitude=list(self.get_parameter('amplitude').value),
            period_s=float(self.get_parameter('period_s').value),
            waypoints=waypoints.reshape(-1).tolist(),
            segment_durations=list(self.get_parameter('segment_durations').value),
            loop=bool(self.get_parameter('loop').value),
        )
        if self.script_running:
            self.script_start_time = self.get_clock().now()
            state = 'started'
        else:
            state = 'prepared; call ~/start to begin motion'
        self.get_logger().info(
            f'Scripted trajectory {state}: {self.script.script_type} '
            f'center={self.script_center}')

    def handle_script_start(self, request, response):
        if self.script_center is None:
            response.success = False
            response.message = 'waiting for odometry to anchor trajectory'
            return response
        self.script_start_time = self.get_clock().now()
        self.script_running = True
        response.success = True
        response.message = f'{self.script.script_type} trajectory started'
        self.get_logger().info(response.message)
        return response

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.latest_position = np.array([p.x, p.y, p.z], dtype=float)

    def timer_callback(self):
        goal = self._scripted_goal()
        if goal is None:
            return
        self.publish_goal(goal)

    def _scripted_goal(self):
        if self.script_center is None:
            if self.latest_position is None:
                self.get_logger().warn(
                    'Waiting for odometry to anchor scripted trajectory',
                    throttle_duration_sec=2.0)
                return None
            center = (self.latest_position
                      + vector3(self.get_parameter('center_offset').value,
                                'center_offset'))
            self._start_script(center)

        elapsed = 0.0
        if self.script_running:
            elapsed = (self.get_clock().now()
                       - self.script_start_time).nanoseconds * 1e-9
        return clamp_position(
            self.script.sample(elapsed), self.bounds_min, self.bounds_max)

    def publish_goal(self, goal):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.pose.position.x = float(goal[0])
        msg.pose.position.y = float(goal[1])
        msg.pose.position.z = float(goal[2])
        msg.pose.orientation.w = 1.0
        self.goal_pub.publish(msg)

        if self.speed_pub is not None:
            speed = float(self.get_parameter('speed_mps').value)
            if speed > 0.0:
                speed_msg = Float32()
                speed_msg.data = speed
                self.speed_pub.publish(speed_msg)

        self.log_goal(goal)

    def log_goal(self, goal):
        if not self.log_goal_updates:
            return
        now = self.get_clock().now()
        if self.last_goal_log_time is not None and self.log_goal_period_s > 0.0:
            if (now - self.last_goal_log_time) < Duration(seconds=self.log_goal_period_s):
                return
        self.last_goal_log_time = now
        self.get_logger().info(
            f'Published goal on {self.goal_topic}: '
            f'[{goal[0]:.2f}, {goal[1]:.2f}, {goal[2]:.2f}] '
            f'backend={self.backend}')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryCommander()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
