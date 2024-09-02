#!/usr/bin/python
import numpy as np
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point
import rospy
import tf.transformations as trans
import argparse
#from at_fcs_mavros.msg import Trajectory
#from at_fcs_mavros.msg import Waypoint
#from ca_nav_msgs.msg import XYZVPsi
#from ca_nav_msgs.msg import PathXYZVPsi
from core_trajectory_msgs.msg import WaypointXYZVYaw
from core_trajectory_msgs.msg import TrajectoryXYZVYaw
from core_trajectory_controller.msg import Trajectory
from core_trajectory_msgs.msg import FixedTrajectory
import time
import copy

def get_velocities(traj, velocity, max_acc):
    v_prev = 0.

    for i in range(len(traj.waypoints)):
        j = (i+1) % len(traj.waypoints)
        dx = traj.waypoints[j].position.x - traj.waypoints[i].position.x
        dy = traj.waypoints[j].position.y - traj.waypoints[i].position.y

        dist = np.sqrt(dx**2 + dy**2)
        v_limit = np.sqrt(v_prev**2 + 2*max_acc*dist)
        traj.waypoints[i].velocity = min(velocity, v_limit)
        v_prev = traj.waypoints[i].velocity



def get_racetrack_waypoints(attributes):#length, width, height):
    frame_id = str(attributes['frame_id'])
    length = float(attributes['length'])
    width = float(attributes['width'])
    height = float(attributes['height'])
    velocity = float(attributes['velocity'])
    max_acceleration = float(attributes['max_acceleration'])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id
    straightaway_length = length - width

    # first straightaway
    xs1 = np.linspace(0, straightaway_length, 20)
    ys1 = np.zeros(xs1.shape)
    yaws1 = np.zeros(xs1.shape)

    # first turn
    t = np.linspace(-np.pi/2, np.pi/2, 30)[1:-1]
    xs2 = width/2.*np.cos(t) + straightaway_length
    ys2 = width/2.*np.sin(t) + width/2.
    xs2d = -width/2.*np.sin(t) # derivative of xs
    ys2d = width/2.*np.cos(t) # derivative of ys
    yaws2 = np.arctan2(ys2d, xs2d)

    # second straightaway
    xs3 = np.linspace(straightaway_length, 0, 20)
    ys3 = width*np.ones(xs3.shape)
    yaws3 = np.pi*np.ones(xs3.shape)

    # second turn
    t = np.linspace(np.pi/2, 3*np.pi/2, 30)[1:-1]
    xs4 = width/2.*np.cos(t)
    ys4 = width/2.*np.sin(t) + width/2.
    yaws4 = yaws2 + np.pi

    xs = np.hstack((xs1, xs2, xs3, xs4))
    ys = np.hstack((ys1, ys2, ys3, ys4))
    yaws = np.hstack((yaws1, yaws2, yaws3, yaws4))

    now = rospy.Time.now()
    for i in range(xs.shape[0]):
        wp = WaypointXYZVYaw()
        wp.position.x = xs[i]
        wp.position.y = ys[i]
        wp.position.z = height
        wp.yaw = yaws[i]

        traj.waypoints.append(wp)

    get_velocities(traj, velocity, max_acceleration)

    return traj


def get_figure8_waypoints(attributes):#length, width, height):
    frame_id = str(attributes['frame_id'])
    length = float(attributes['length'])
    width = float(attributes['width'])
    height = float(attributes['height'])
    velocity = float(attributes['velocity'])
    max_acceleration = float(attributes['max_acceleration'])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id
    now = rospy.Time.now()

    # figure 8 points
    t = np.linspace(0, 2*np.pi, 100)
    x = np.cos(t) * length - length
    y = np.cos(t)*np.sin(t) * 2*width

    # derivative of figure 8 curve, used to find yaw
    xd = -np.sin(t) * length
    yd = (np.cos(t)**2 - np.sin(t)**2) * 2*width

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

    get_velocities(traj, velocity, max_acceleration)

    return traj

def get_line_waypoints(attributes):#length, height):
    frame_id = str(attributes['frame_id'])
    length = float(attributes['length'])
    height = float(attributes['height'])
    velocity = float(attributes['velocity'])
    max_acceleration = float(attributes['max_acceleration'])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id

    for y in np.arange(0, -length, -0.5):
        wp = WaypointXYZVYaw()
        wp.position.x = 0
        wp.position.y = y
        wp.position.z = height
        wp.yaw = 0

        traj.waypoints.append(wp)

    get_velocities(traj, velocity, max_acceleration)

    return traj

def get_point_waypoints(attributes):#length, height):
    frame_id = str(attributes['frame_id'])
    x = float(attributes['x'])
    y = float(attributes['y'])
    height = float(attributes['height'])
    velocity = float(attributes['velocity'])
    max_acceleration = float(attributes['max_acceleration'])

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

def get_box_waypoints(attributes):#length, height):
    frame_id = str(attributes['frame_id'])
    length = float(attributes['length'])
    height = float(attributes['height'])
    velocity = float(attributes['velocity'])
    max_acceleration = float(attributes['max_acceleration'])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id

    wp1 = WaypointXYZVYaw()
    wp1.position.x = 0.
    wp1.position.y = 0.
    wp1.position.z = height
    wp1.yaw = 0
    traj.waypoints.append(wp1)

    wp2 = WaypointXYZVYaw()
    wp2.position.x = length
    wp2.position.y = 0.
    wp2.position.z = height
    wp2.yaw = 0
    traj.waypoints.append(wp2)

    wp3 = WaypointXYZVYaw()
    wp3.position.x = length
    wp3.position.y = 0.
    wp3.position.z = height + height
    wp3.yaw = 0
    traj.waypoints.append(wp3)

    wp4 = WaypointXYZVYaw()
    wp4.position.x = 0.
    wp4.position.y = 0.
    wp4.position.z = height + height
    wp4.yaw = 0
    traj.waypoints.append(wp4)

    return traj

def get_vertical_lawnmower_waypoints(attributes):#length, width, height, velocity):
    frame_id = str(attributes['frame_id'])
    length = float(attributes['length'])
    width = float(attributes['width'])
    height = float(attributes['height'])
    velocity = float(attributes['velocity'])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id

    for i in range(abs(int(height/width))):
        wp1 = WaypointXYZVYaw()
        wp1.position.x = 0
        wp1.position.y = 0
        wp1.position.z = np.sign(height)*(i+1)*width
        wp1.yaw = 0
        wp1.velocity = 0.1

        wp1_ = WaypointXYZVYaw()
        wp1_.position.x = 0
        wp1_.position.y = 0.5
        wp1_.position.z = np.sign(height)*(i+1)*width
        wp1_.yaw = 0
        wp1_.velocity = velocity

        wp2 = WaypointXYZVYaw()
        wp2.position.x = 0
        wp2.position.y = length
        wp2.position.z = np.sign(height)*(i+1)*width
        wp2.yaw = 0
        wp2.velocity = 0.1

        wp2_ = WaypointXYZVYaw()
        wp2_.position.x = 0
        wp2_.position.y = length - 0.5
        wp2_.position.z = np.sign(height)*(i+1)*width
        wp2_.yaw = 0
        wp2_.velocity = velocity

        if i%2 == 0:
            traj.waypoints.append(wp1)
            wp1_slow = copy.deepcopy(wp1_)
            wp1_slow.velocity = 0.1
            traj.waypoints.append(wp1_slow)
            traj.waypoints.append(wp1_)
            traj.waypoints.append(wp2_)
            traj.waypoints.append(wp2)
        else:
            traj.waypoints.append(wp2)
            wp2_slow = copy.deepcopy(wp2_)
            wp2_slow.velocity = 0.1
            traj.waypoints.append(wp2_slow)
            traj.waypoints.append(wp2_)
            traj.waypoints.append(wp1_)
            traj.waypoints.append(wp1)

    wp = WaypointXYZVYaw()
    wp.position.x = 0
    wp.position.y = 0
    wp.position.z = 0
    wp.yaw = 0
    wp.velocity = 0.5
    traj.waypoints.append(wp)

    return traj

def get_circle_waypoints(attributes):#radius, velocity, frame_id):
    frame_id = str(attributes['frame_id'])
    radius = float(attributes['radius'])
    velocity = float(attributes['velocity'])

    traj = TrajectoryXYZVYaw()
    traj.header.frame_id = frame_id
    traj.header.stamp = rospy.Time.now()

    wp0 = WaypointXYZVYaw()
    wp0.position.x = 0
    wp0.position.y = 0
    wp0.position.z = 0
    wp0.yaw = 0
    wp0.velocity = velocity
    traj.waypoints.append(wp0)

    wp1 = WaypointXYZVYaw()
    wp1.position.x = radius
    wp1.position.y = 0
    wp1.position.z = 0
    wp1.yaw = 0
    wp1.velocity = velocity
    traj.waypoints.append(wp1)

    for angle in np.arange(0, 2*np.pi, 10.*np.pi/180.):
        wp = WaypointXYZVYaw()
        wp.position.x = radius*np.cos(angle)
        wp.position.y = radius*np.sin(angle)
        wp.position.z = 0
        wp.yaw = 0
        wp.velocity = velocity
        traj.waypoints.append(wp)

    wp_end0 = WaypointXYZVYaw()
    wp_end0.position.x = radius
    wp_end0.position.y = 0
    wp_end0.position.z = 0
    wp_end0.yaw = 0
    wp_end0.velocity = velocity
    traj.waypoints.append(wp_end0)

    wp_end1 = WaypointXYZVYaw()
    wp_end1.position.x = 0
    wp_end1.position.y = 0
    wp_end1.position.z = 0
    wp_end1.yaw = 0
    wp_end1.velocity = velocity
    traj.waypoints.append(wp_end1)

    return traj


def fixed_trajectory_callback(msg):
    attributes = {}

    for key_value in msg.attributes:
        attributes[key_value.key] = key_value.value

    trajectory_msg = None

    if msg.type == 'Figure8':
        trajectory_msg = get_figure8_waypoints(attributes)
    elif msg.type == 'Circle':
        trajectory_msg = get_circle_waypoints(attributes)
    elif msg.type == 'Racetrack':
        trajectory_msg = get_racetrack_waypoints(attributes)
    elif msg.type == 'Line':
        trajectory_msg = get_line_waypoints(attributes)
    elif msg.type == 'Point':
        trajectory_msg = get_point_waypoints(attributes)

    if trajectory_msg != None:
        trajectory_track_pub.publish(trajectory_msg)
    else:
        print('No trajectory sent.')

if __name__ == '__main__':
    rospy.init_node('fixed_trajectory_generator')

    fixed_trajectory_sub = rospy.Subscriber('fixed_trajectory', FixedTrajectory, fixed_trajectory_callback)

    trajectory_track_pub = rospy.Publisher('trajectory_track', TrajectoryXYZVYaw, queue_size=1)

    rospy.spin()
