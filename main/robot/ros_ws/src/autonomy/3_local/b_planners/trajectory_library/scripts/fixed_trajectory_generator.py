#!/usr/bin/python3
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
from airstack_msgs.msg import WaypointXYZVYaw
from airstack_msgs.msg import TrajectoryXYZVYaw
from airstack_msgs.msg import FixedTrajectory
import time
import copy
import rclpy
from rclpy.node import Node

logger = None

def get_velocities(traj, velocity, max_acc):
    v_prev = 0.0

    for i in range(len(traj.waypoints)):
        j = (i + 1) % len(traj.waypoints)
        dx = traj.waypoints[j].position.x - traj.waypoints[i].position.x
        dy = traj.waypoints[j].position.y - traj.waypoints[i].position.y

        dist = np.sqrt(dx**2 + dy**2)
        v_limit = np.sqrt(v_prev**2 + 2 * max_acc * dist)
        traj.waypoints[i].velocity = min(velocity, v_limit)
        v_prev = traj.waypoints[i].velocity


def get_velocities_dual(traj, velocity, max_acc):
    v_prev = 0.0

    for i in range(len(traj.waypoints)):
        j = (i + 1) % len(traj.waypoints)
        dx = traj.waypoints[j].position.x - traj.waypoints[i].position.x
        dy = traj.waypoints[j].position.y - traj.waypoints[i].position.y

        dist = np.sqrt(dx**2 + dy**2)
        v_limit = np.sqrt(v_prev**2 + 2 * max_acc * dist)
        traj.waypoints[i].velocity = min(velocity, v_limit)
        v_prev = traj.waypoints[i].velocity

    v_prev = 0.0
    for i in range(len(traj.waypoints) - 1, int(len(traj.waypoints) * 0.85), -1):
        j = (i - 1) % len(traj.waypoints)
        dx = traj.waypoints[j].position.x - traj.waypoints[i].position.x
        dy = traj.waypoints[j].position.y - traj.waypoints[i].position.y

        dist = np.sqrt(dx**2 + dy**2)
        v_limit = np.sqrt(v_prev**2 + 2 * max_acc * dist)
        traj.waypoints[i].velocity = min(velocity, v_limit)
        v_prev = traj.waypoints[i].velocity

    traj.waypoints[len(traj.waypoints) - 1].velocity = 0.0

def get_velocities_adjust(traj, max_acc):
    slow_down_adjust = []
    speed_up_adjust = []
    
    for i in range(len(traj.waypoints)):
        if i != len(traj.waypoints)-1:
            if traj.waypoints[i].velocity > traj.waypoints[i+1].velocity:
                v_start = traj.waypoints[i+1].velocity
                v_target = traj.waypoints[i].velocity
                slow_down_adjust.append((i+1, v_start, v_target))
            elif traj.waypoints[i].velocity < traj.waypoints[i+1].velocity:
                v_start = traj.waypoints[i].velocity
                v_target = traj.waypoints[i+1].velocity
                speed_up_adjust.append((i, v_start, v_target))

    for start_index, v_start, v_target in slow_down_adjust:
        prev_i = start_index
        v_prev = v_start
        for i in range(start_index-1, -1, -1):
            dx = traj.waypoints[prev_i].position.x - traj.waypoints[i].position.x
            dy = traj.waypoints[prev_i].position.y - traj.waypoints[i].position.y
            dist = np.sqrt(dx**2 + dy**2)
            v_limit = np.sqrt(v_prev**2 + 2 * max_acc * dist)
            traj.waypoints[i].velocity = min(v_target, v_limit)
            v_prev = traj.waypoints[i].velocity
            if v_prev >= v_target:
                break

    for start_index, v_start, v_target in speed_up_adjust:
        prev_i = start_index
        v_prev = v_start
        for i in range(start_index+1, len(traj.waypoints)):
            dx = traj.waypoints[prev_i].position.x - traj.waypoints[i].position.x
            dy = traj.waypoints[prev_i].position.y - traj.waypoints[i].position.y
            dist = np.sqrt(dx**2 + dy**2)
            v_limit = np.sqrt(v_prev**2 + 2 * max_acc * dist)
            if v_limit > traj.waypoints[i].velocity:
                break
            traj.waypoints[i].velocity = min(v_target, v_limit)
            v_prev = traj.waypoints[i].velocity
            if v_prev >= v_target:
                break


def get_accelerations(traj):
    a_prev = 0.0

    for i in range(len(traj.waypoints) - 2):
        j = (i + 1) % len(traj.waypoints)
        k = (i + 2) % len(traj.waypoints)

        dx1 = traj.waypoints[j].position.x - traj.waypoints[i].position.x
        dy1 = traj.waypoints[j].position.y - traj.waypoints[i].position.y
        dist1 = np.sqrt(dx1**2 + dy1**2)

        v_currx = traj.waypoints[i].velocity * (dx1) / dist1
        v_curry = traj.waypoints[i].velocity * (dy1) / dist1

        dx2 = traj.waypoints[k].position.x - traj.waypoints[j].position.x
        dy2 = traj.waypoints[k].position.y - traj.waypoints[j].position.y
        dist2 = np.sqrt(dx2**2 + dy2**2)

        v_nextx = traj.waypoints[j].velocity * (dx2) / dist2
        v_nexty = traj.waypoints[j].velocity * (dy2) / dist2

        acc_limit = 50.
        if abs(dx1 - 0) < 1e-6:
            traj.waypoints[i].acceleration.x = 0.0
        else:
            traj.waypoints[i].acceleration.x = min(
                (v_nextx**2 - v_currx**2) / (2 * dx1), acc_limit
            )

        if abs(dy1 - 0) < 1e-6:
            traj.waypoints[i].acceleration.y = 0.0
        else:
            traj.waypoints[i].acceleration.y = min(
                (v_nexty**2 - v_curry**2) / (2 * dy1), acc_limit
            )

        traj.waypoints[i].acceleration.z = 0.0

    traj.waypoints[len(traj.waypoints) - 2].acceleration.x = 0.0
    traj.waypoints[len(traj.waypoints) - 2].acceleration.y = 0.0
    traj.waypoints[len(traj.waypoints) - 2].acceleration.z = 0.0
    traj.waypoints[len(traj.waypoints) - 1].acceleration.x = 0.0
    traj.waypoints[len(traj.waypoints) - 1].acceleration.y = 0.0
    traj.waypoints[len(traj.waypoints) - 1].acceleration.z = 0.0


def get_racetrack_waypoints(attributes):  # length, width, height):
    frame_id = str(attributes["frame_id"])
    length = float(attributes["length"])
    width = float(attributes["width"])
    height = float(attributes["height"])
    velocity = float(attributes["velocity"])
    turn_velocity = float(attributes["turn_velocity"])
    max_acceleration = float(attributes["max_acceleration"])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id
    straightaway_length = length - width

    # first straightaway
    xs1 = np.linspace(0, straightaway_length, 80)
    ys1 = np.zeros(xs1.shape)
    vs1 = np.ones(xs1.shape)*velocity
    yaws1 = np.zeros(xs1.shape)

    # first turn
    t = np.linspace(-np.pi / 2, np.pi / 2, 50)[1:-1]
    xs2 = width / 2.0 * np.cos(t) + straightaway_length
    ys2 = width / 2.0 * np.sin(t) + width / 2.0
    xs2d = -width / 2.0 * np.sin(t)  # derivative of xs
    ys2d = width / 2.0 * np.cos(t)  # derivative of ys
    vs2 = np.ones(xs2.shape)*turn_velocity
    yaws2 = np.arctan2(ys2d, xs2d)

    # second straightaway
    xs3 = np.linspace(straightaway_length, 0, 80)
    ys3 = width * np.ones(xs3.shape)
    vs3 = np.ones(xs3.shape)*velocity
    yaws3 = np.pi * np.ones(xs3.shape)

    # second turn
    t = np.linspace(np.pi / 2, 3 * np.pi / 2, 50)[1:-1]
    xs4 = width / 2.0 * np.cos(t)
    ys4 = width / 2.0 * np.sin(t) + width / 2.0
    vs4 = np.ones(xs4.shape)*turn_velocity
    yaws4 = yaws2 + np.pi

    xs = np.hstack((xs1, xs2, xs3, xs4))
    ys = np.hstack((ys1, ys2, ys3, ys4))
    vs = np.hstack((vs1, vs2, vs3, vs4))
    vs[0] = 0.
    vs[-1] = 0.
    yaws = np.hstack((yaws1, yaws2, yaws3, yaws4))

    # now = rospy.Time.now()
    for i in range(xs.shape[0]):
        wp = WaypointXYZVYaw()
        wp.position.x = xs[i]
        wp.position.y = ys[i]
        wp.position.z = height
        wp.velocity = vs[i]

        #logger.info(str(i) + ' ' + str(wp.velocity))
        
        wp.yaw = yaws[i]

        traj.waypoints.append(wp)

    #get_velocities_dual(traj, velocity, max_acceleration)
    get_velocities_adjust(traj, max_acceleration)
    get_accelerations(traj)

    return traj


def get_figure8_waypoints(attributes):  # length, width, height):
    frame_id = str(attributes["frame_id"])
    length = float(attributes["length"])
    width = float(attributes["width"])
    height = float(attributes["height"])
    velocity = float(attributes["velocity"])
    max_acceleration = float(attributes["max_acceleration"])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id
    # now = rospy.Time.now()

    # figure 8 points
    t = np.linspace(0, 2 * np.pi, 600)
    x = np.cos(t) * length - length
    y = np.cos(t) * np.sin(t) * 2 * width

    # derivative of figure 8 curve, used to find yaw
    xd = -np.sin(t) * length
    yd = (np.cos(t) ** 2 - np.sin(t) ** 2) * 2 * width

    for i in range(t.shape[0] - 1):
        x1 = x[i]
        y1 = y[i]
        x2 = x1 + xd[i]
        y2 = y1 + yd[i]

        yaw = np.arctan2(y2 - y1, x2 - x1)

        wp = WaypointXYZVYaw()
        wp.position.x = x1
        wp.position.y = y1
        wp.position.z = height
        wp.yaw = yaw

        traj.waypoints.append(wp)

    get_velocities_dual(traj, velocity, max_acceleration)
    get_accelerations(traj)

    return traj


def get_line_waypoints(attributes):  # length, height):
    frame_id = str(attributes["frame_id"])
    length = float(attributes["length"])
    height = float(attributes["height"])
    velocity = float(attributes["velocity"])
    max_acceleration = float(attributes["max_acceleration"])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id

    for y in np.arange(0, -length, -0.5):
        wp = WaypointXYZVYaw()
        wp.position.x = -y
        wp.position.y = 0
        wp.position.z = height
        wp.yaw = 0

        traj.waypoints.append(wp)

    get_velocities(traj, velocity, max_acceleration)

    return traj


def get_point_waypoints(attributes):  # length, height):
    frame_id = str(attributes["frame_id"])
    x = float(attributes["x"])
    y = float(attributes["y"])
    height = float(attributes["height"])
    velocity = float(attributes["velocity"])
    max_acceleration = float(attributes["max_acceleration"])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id

    # add first point
    wp = WaypointXYZVYaw()
    wp.position.x = x
    wp.position.y = y
    wp.position.z = height
    wp.yaw = 0

    traj.waypoints.append(wp)

    get_velocities(traj, velocity, max_acceleration)

    return traj


def get_box_waypoints(attributes):  # length, height):
    frame_id = str(attributes["frame_id"])
    length = float(attributes["length"])
    height = float(attributes["height"])
    velocity = float(attributes["velocity"])
    max_acceleration = float(attributes["max_acceleration"])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id

    wp1 = WaypointXYZVYaw()
    wp1.position.x = 0.0
    wp1.position.y = 0.0
    wp1.position.z = height
    wp1.yaw = 0
    traj.waypoints.append(wp1)

    wp2 = WaypointXYZVYaw()
    wp2.position.x = length
    wp2.position.y = 0.0
    wp2.position.z = height
    wp2.yaw = 0.0
    traj.waypoints.append(wp2)

    wp3 = WaypointXYZVYaw()
    wp3.position.x = length
    wp3.position.y = 0.0
    wp3.position.z = height + height
    wp3.yaw = 0.0
    traj.waypoints.append(wp3)

    wp4 = WaypointXYZVYaw()
    wp4.position.x = 0.0
    wp4.position.y = 0.0
    wp4.position.z = height + height
    wp4.yaw = 0.0
    traj.waypoints.append(wp4)

    return traj


def get_lawnmower_waypoints(attributes):  # length, width, height, velocity):
    frame_id = str(attributes["frame_id"])
    length = float(attributes["length"])
    width = float(attributes["width"])
    height = float(attributes["height"])
    velocity = float(attributes["velocity"])
    vertical = bool(int(attributes["vertical"]))

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id

    for i in range(abs(int(height / width))):
        wp1 = WaypointXYZVYaw()
        wp1.position.x = 0.0
        wp1.position.y = 0.0
        wp1.position.z = np.sign(height) * (i + 1) * width
        wp1.yaw = 0.0
        wp1.velocity = 0.1

        wp1_ = WaypointXYZVYaw()
        wp1_.position.x = 0.0
        wp1_.position.y = 0.5
        wp1_.position.z = np.sign(height) * (i + 1) * width
        wp1_.yaw = 0.0
        wp1_.velocity = velocity

        wp2 = WaypointXYZVYaw()
        wp2.position.x = 0.0
        wp2.position.y = length
        wp2.position.z = np.sign(height) * (i + 1) * width
        wp2.yaw = 0.0
        wp2.velocity = 0.1

        wp2_ = WaypointXYZVYaw()
        wp2_.position.x = 0.0
        wp2_.position.y = length - 0.5
        wp2_.position.z = np.sign(height) * (i + 1) * width
        wp2_.yaw = 0.0
        wp2_.velocity = velocity

        yaw = np.arctan2(wp2.position.y - wp1.position.y, wp2.position.z - wp1.position.z)

        if i % 2 == 0:
            if not vertical:
                wp1.yaw = yaw
                wp1_.yaw = yaw
                wp2_.yaw = yaw
                wp2.yaw = yaw
            
            traj.waypoints.append(wp1)
            wp1_slow = copy.deepcopy(wp1_)
            wp1_slow.velocity = 0.1
            traj.waypoints.append(wp1_slow)
            traj.waypoints.append(wp1_)
            traj.waypoints.append(wp2_)
            traj.waypoints.append(wp2)
        else:
            if not vertical:
                wp2.yaw = -yaw
                wp2_.yaw = -yaw
                wp1_.yaw = -yaw
                wp1.yaw = -yaw
            
            traj.waypoints.append(wp2)
            wp2_slow = copy.deepcopy(wp2_)
            wp2_slow.velocity = 0.1
            traj.waypoints.append(wp2_slow)
            traj.waypoints.append(wp2_)
            traj.waypoints.append(wp1_)
            traj.waypoints.append(wp1)

    wp = WaypointXYZVYaw()
    wp.position.x = 0.0
    wp.position.y = 0.0
    wp.position.z = 0.0
    wp.yaw = 0.0
    wp.velocity = 0.5
    traj.waypoints.append(wp)
    
    if not vertical:
        for i, wp in enumerate(traj.waypoints):
            wp.position.x, wp.position.y, wp.position.z = wp.position.z, wp.position.y, wp.position.x

    return traj


def get_circle_waypoints(attributes):  # radius, velocity, frame_id):
    frame_id = str(attributes["frame_id"])
    radius = float(attributes["radius"])
    velocity = float(attributes["velocity"])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id
    # traj.header.stamp = rospy.Time.now()

    wp0 = WaypointXYZVYaw()
    wp0.position.x = 0.0
    wp0.position.y = 0.0
    wp0.position.z = 0.0
    wp0.yaw = 0.0
    wp0.velocity = velocity
    traj.waypoints.append(wp0)

    wp1 = WaypointXYZVYaw()
    wp1.position.x = radius
    wp1.position.y = 0.0
    wp1.position.z = 0.0
    wp1.yaw = 0.0
    wp1.velocity = velocity
    traj.waypoints.append(wp1)

    for angle in np.arange(0, 2 * np.pi, 10.0 * np.pi / 180.0):
        wp = WaypointXYZVYaw()
        wp.position.x = radius * np.cos(angle)
        wp.position.y = radius * np.sin(angle)
        wp.position.z = 0.0
        wp.yaw = 0.0
        wp.velocity = velocity
        traj.waypoints.append(wp)

    wp_end0 = WaypointXYZVYaw()
    wp_end0.position.x = radius
    wp_end0.position.y = 0.0
    wp_end0.position.z = 0.0
    wp_end0.yaw = 0.0
    wp_end0.velocity = velocity
    traj.waypoints.append(wp_end0)

    wp_end1 = WaypointXYZVYaw()
    wp_end1.position.x = 0.0
    wp_end1.position.y = 0.0
    wp_end1.position.z = 0.0
    wp_end1.yaw = 0.0
    wp_end1.velocity = velocity
    traj.waypoints.append(wp_end1)

    return traj


class FixedTrajectoryGenerator(Node):
    def __init__(self):
        super().__init__("fixed_trajectory_generator")
        self.fixed_trajectory_sub = self.create_subscription(
            FixedTrajectory,
            "fixed_trajectory_command",
            self.fixed_trajectory_callback,
            1,
        )
        self.trajectory_override_pub = self.create_publisher(
            TrajectoryXYZVYaw, "trajectory_override", 1
        )

    def fixed_trajectory_callback(self, msg):
        print("generating")
        attributes = {}

        for key_value in msg.attributes:
            attributes[key_value.key] = key_value.value

        trajectory_msg = None

        if msg.type == "Figure8":
            trajectory_msg = get_figure8_waypoints(attributes)
        elif msg.type == "Circle":
            trajectory_msg = get_circle_waypoints(attributes)
        elif msg.type == "Racetrack":
            trajectory_msg = get_racetrack_waypoints(attributes)
        elif msg.type == "Line":
            trajectory_msg = get_line_waypoints(attributes)
        elif msg.type == "Point":
            trajectory_msg = get_point_waypoints(attributes)
        elif msg.type == "Lawnmower":
            trajectory_msg = get_lawnmower_waypoints(attributes)

        if trajectory_msg != None:
            self.trajectory_override_pub.publish(trajectory_msg)
        else:
            print("No trajectory sent.")

#import matplotlib.pyplot as plt

if __name__ == "__main__":
    '''
    dct = {'frame_id': 'base_link', 'length': 2, 'width': 1, 'height': 0, 'velocity': 2, 'turn_velocity': 0.5, 'max_acceleration': 0.1}
    traj1 = get_racetrack_waypoints(dct)
    dct['max_acceleration'] = 100000000.
    traj2 = get_racetrack_waypoints(dct)

    v1 = [wp.velocity for wp in traj1.waypoints]
    v2 = [wp.velocity for wp in traj2.waypoints]

    plt.plot(v1, '.-')
    plt.plot(v2, '.-')
    plt.show()
    exit()

    '''
    rclpy.init(args=None)

    node = FixedTrajectoryGenerator()
    logger = node.get_logger()
    
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
