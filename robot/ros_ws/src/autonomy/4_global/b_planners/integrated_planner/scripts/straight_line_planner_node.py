#!/usr/bin/env python3
"""Minimal straight-line planner for ideal open-space testing.

Subscribes to /goal_point, reads current pose from TF, builds a straight-line
path at constant speed, and publishes MultiDOFJointTrajectory on /cmd_trajectory.
"""

import math
from dataclasses import dataclass, field
from enum import Enum, auto
from typing import List, Optional

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import Point, Pose, Transform, Twist, Vector3
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from visualization_msgs.msg import Marker

import tf2_ros
from tf2_ros import TransformException


@dataclass
class TimedXYZYaw:
    pos: List[float]
    yaw: float = 0.0
    time_from_start: float = 0.0
    desired_vel: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])


class PlannerStatus(Enum):
    WAIT_GOAL = auto()
    PATH_EXEC = auto()


def wrap_angle(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def yaw_from_transform(transform: Transform) -> float:
    q = transform.rotation
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def yaw_to_quaternion(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def s_curve_blend(s: float) -> float:
    return 3.0 * s * s - 2.0 * s * s * s


class StraightLinePlannerNode(Node):
    def __init__(self):
        super().__init__('straight_line_planner_node')

        self.declare_parameter('world_frame_id', 'map')
        self.declare_parameter('robot_frame_id', 'body')
        self.declare_parameter('pub_global_plan_topic', '/global_plan_viz')
        self.declare_parameter('pub_trajectory_viz_topic', '/traj_viz')
        self.declare_parameter('cruise_speed', 0.5)
        self.declare_parameter('interpolate_step', 0.1)
        self.declare_parameter('goal_reached_threshold', 0.2)

        self.world_frame_id = self.get_parameter('world_frame_id').value
        self.robot_frame_id = self.get_parameter('robot_frame_id').value
        self.pub_trajectory_viz_topic = self.get_parameter('pub_trajectory_viz_topic').value
        self.pub_global_plan_topic = self.get_parameter('pub_global_plan_topic').value
        self.cruise_speed = float(self.get_parameter('cruise_speed').value)
        self.interpolate_step = float(self.get_parameter('interpolate_step').value)
        self.goal_reached_threshold = float(self.get_parameter('goal_reached_threshold').value)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        goal_qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Pose, '/goal_point', self.goal_pose_callback, goal_qos)

        self.pub_trajectory = self.create_publisher(MultiDOFJointTrajectory, '/cmd_trajectory', 10)
        self.pub_trajectory_viz = self.create_publisher(Marker, self.pub_trajectory_viz_topic, 10)
        self.pub_global_plan_viz = self.create_publisher(Marker, self.pub_global_plan_topic, 10)

        self.status = PlannerStatus.WAIT_GOAL
        self.current_path_dense: List[TimedXYZYaw] = []
        self.path_start: Optional[List[float]] = None
        self.path_goal: Optional[List[float]] = None
        self.curr_goal_xyz = [0.0, 0.0, 0.0]
        self.curr_goal_yaw = 0.0
        self.received_first_robot_tf = False

        # Slow timer only for arrival monitoring, not replanning.
        self.create_timer(1.0, self.monitor_callback)

        self.get_logger().info(
            f'Straight-line planner ready: frame={self.world_frame_id}->{self.robot_frame_id}, '
            f'speed={self.cruise_speed} m/s, step={self.interpolate_step} m, '
            f'publish-once mode'
        )

    def goal_pose_callback(self, msg: Pose):
        self.get_logger().warn('Receiving a goal')
        self.curr_goal_xyz = [msg.position.x, msg.position.y, msg.position.z]

        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.curr_goal_yaw = math.atan2(siny_cosp, cosy_cosp)

        transform = self.lookup_robot_transform()
        if transform is None:
            self.get_logger().error('Cannot plan: robot TF unavailable.')
            return

        current_pos = [
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ]
        current_yaw = yaw_from_transform(transform)

        self.path_start = list(current_pos)
        self.path_goal = list(self.curr_goal_xyz)
        self.current_path_dense = self.interpolate_straight_line(
            current_pos,
            current_yaw,
            self.curr_goal_xyz,
            self.curr_goal_yaw,
        )

        if len(self.current_path_dense) < 2:
            self.get_logger().warn('Goal not reachable (degenerate path).')
            self.status = PlannerStatus.WAIT_GOAL
            return

        self.status = PlannerStatus.PATH_EXEC
        self.publish_plan()
        self.get_logger().info(
            f'Published straight-line trajectory with {len(self.current_path_dense)} points'
        )

    def lookup_robot_transform(self) -> Optional[Transform]:
        try:
            transform_stamped = self.tf_buffer.lookup_transform(
                self.world_frame_id,
                self.robot_frame_id,
                rclpy.time.Time(),
            )
            return transform_stamped.transform
        except TransformException as ex:
            self.get_logger().error(f'Robot tf not received: {ex}')
            return None

    def interpolate_straight_line(self, start_pos, start_yaw, goal_pos, goal_yaw, t_offset=0.0):
        if self.interpolate_step <= 0.0 or self.cruise_speed <= 0.0:
            self.get_logger().warn('interpolate_step or cruise_speed cannot be <= 0.')
            return []

        path_raw = [start_pos, goal_pos]
        dense: List[TimedXYZYaw] = []
        t = t_offset

        p0 = path_raw[0]
        p1 = path_raw[1]
        diff_1 = [p1[i] - p0[i] for i in range(3)]
        diff_norm = math.sqrt(sum(v * v for v in diff_1))

        first_pt = TimedXYZYaw(pos=list(p0), yaw=0.0, time_from_start=t)
        if diff_norm > 1e-6:
            scale = self.cruise_speed / diff_norm
            first_pt.desired_vel = [diff_1[i] * scale for i in range(3)]
        dense.append(first_pt)

        seg = [p1[i] - p0[i] for i in range(3)]
        seg_len = math.sqrt(sum(v * v for v in seg))
        if seg_len < 1e-6:
            return dense

        direction = [seg[i] / seg_len for i in range(3)]
        last_pos = list(p0)
        dt = self.interpolate_step / self.cruise_speed

        dist = self.interpolate_step
        while dist < seg_len:
            pos = [p0[i] + direction[i] * dist for i in range(3)]
            t += dt
            dense.append(
                TimedXYZYaw(
                    pos=pos,
                    yaw=0.0,
                    time_from_start=t,
                    desired_vel=[direction[i] * self.cruise_speed for i in range(3)],
                )
            )
            last_pos = pos
            dist += self.interpolate_step

        ds = math.sqrt(sum((p1[i] - last_pos[i]) ** 2 for i in range(3)))
        t += ds / self.cruise_speed
        dense.append(TimedXYZYaw(pos=list(p1), yaw=0.0, time_from_start=t))

        t_start = dense[0].time_from_start
        t_end = dense[-1].time_from_start
        duration = t_end - t_start
        yaw_start = wrap_angle(start_yaw)
        yaw_diff = wrap_angle(goal_yaw - yaw_start)

        if duration <= 1e-6:
            for pt in dense:
                pt.yaw = wrap_angle(goal_yaw)
            return dense

        for pt in dense:
            s = (pt.time_from_start - t_start) / duration
            s = max(0.0, min(1.0, s))
            pt.yaw = wrap_angle(yaw_start + s_curve_blend(s) * yaw_diff)

        return dense

    def monitor_callback(self):
        if self.status != PlannerStatus.PATH_EXEC or self.path_goal is None:
            if self.status == PlannerStatus.WAIT_GOAL:
                self.get_logger().warn('Waiting for goal.', throttle_duration_sec=5.0)
            return

        transform = self.lookup_robot_transform()
        if transform is None:
            return

        if not self.received_first_robot_tf:
            self.received_first_robot_tf = True
            self.get_logger().warn('Received first robot_tf')

        current_pos = [
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
        ]
        dist_to_goal = math.sqrt(
            sum((current_pos[i] - self.path_goal[i]) ** 2 for i in range(3))
        )
        if dist_to_goal <= self.goal_reached_threshold:
            self.get_logger().info('Reached goal.')
            self.current_path_dense.clear()
            self.path_start = None
            self.path_goal = None
            self.status = PlannerStatus.WAIT_GOAL

    def publish_plan(self):
        if len(self.current_path_dense) < 1:
            self.get_logger().warn('Current path is empty, no traj to publish.')
            return

        traj_msg = MultiDOFJointTrajectory()
        traj_msg.header.stamp = self.get_clock().now().to_msg()
        traj_msg.header.frame_id = self.world_frame_id
        traj_msg.joint_names = [self.robot_frame_id]

        for pt in self.current_path_dense:
            pt_msg = MultiDOFJointTrajectoryPoint()
            qx, qy, qz, qw = yaw_to_quaternion(pt.yaw)
            tf_msg = Transform()
            tf_msg.translation = Vector3(x=pt.pos[0], y=pt.pos[1], z=pt.pos[2])
            tf_msg.rotation.x = qx
            tf_msg.rotation.y = qy
            tf_msg.rotation.z = qz
            tf_msg.rotation.w = qw
            pt_msg.transforms = [tf_msg]
            pt_msg.velocities = [
                Twist(
                    linear=Vector3(
                        x=pt.desired_vel[0],
                        y=pt.desired_vel[1],
                        z=pt.desired_vel[2],
                    )
                )
            ]
            pt_msg.time_from_start = Duration(seconds=pt.time_from_start).to_msg()
            traj_msg.points.append(pt_msg)

        self.pub_trajectory.publish(traj_msg)
        self.publish_path_viz()

    def publish_path_viz(self):
        if self.path_start is None or self.path_goal is None:
            return

        stamp = self.get_clock().now().to_msg()
        start_pt = Point(x=self.path_start[0], y=self.path_start[1], z=self.path_start[2])
        goal_pt = Point(x=self.path_goal[0], y=self.path_goal[1], z=self.path_goal[2])

        local_line = Marker()
        local_line.header.stamp = stamp
        local_line.header.frame_id = self.world_frame_id
        local_line.ns = 'straight_line_local'
        local_line.id = 0
        local_line.type = Marker.LINE_STRIP
        local_line.action = Marker.ADD
        local_line.pose.orientation.w = 1.0
        local_line.scale.x = 0.08
        local_line.color.r = 0.0
        local_line.color.g = 1.0
        local_line.color.b = 0.0
        local_line.color.a = 1.0
        local_line.points = [start_pt, goal_pt]
        self.pub_trajectory_viz.publish(local_line)

        global_line = Marker()
        global_line.header = local_line.header
        global_line.ns = 'straight_line_global'
        global_line.id = 0
        global_line.type = Marker.LINE_STRIP
        global_line.action = Marker.ADD
        global_line.pose.orientation.w = 1.0
        global_line.scale.x = 0.05
        global_line.color.r = 1.0
        global_line.color.g = 0.0
        global_line.color.b = 0.0
        global_line.color.a = 1.0
        global_line.points = [start_pt, goal_pt]
        self.pub_global_plan_viz.publish(global_line)


def main():
    rclpy.init()
    node = StraightLinePlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
