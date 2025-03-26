#!/usr/bin/python3
import rosbag2_py
import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sys
import math

def plot(bag_path, odom_topic_name, tracking_point_topic_name):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='', output_serialization_format='')
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    msg_type = get_message(type_map[odom_topic_name])

    start_time = None
    
    odoms = []
    odom_times = []
    tps = []
    tp_times = []
    yaws = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == odom_topic_name:
            msg = deserialize_message(data, get_message(type_map[odom_topic_name]))
            odoms.append([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])

            t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            if start_time == None:
                start_time = t
            odom_times.append(t - start_time)
            
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            yaws.append(yaw)
        elif topic == tracking_point_topic_name:
            msg = deserialize_message(data, get_message(type_map[tracking_point_topic_name]))
            tps.append([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

            t = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9
            if start_time == None:
                start_time = t
            tp_times.append(t - start_time)
            

    odoms = np.array(odoms)
    tps = np.array(tps)
    
    fig = plt.figure()
    ax1 = fig.add_subplot(211)
    ax2 = fig.add_subplot(212)
    ax1.plot(odoms[:, 0], odoms[:, 1], marker='.', linestyle='-', label='odom')
    ax1.plot(tps[:, 0], tps[:, 1], marker='.', linestyle='-', label='tracking point')
    
    ax2.plot(odom_times, odoms[:, 2], linestyle='-', label='odom')
    ax2.plot(tp_times, tps[:, 2], linestyle='-', label='tracking point')

    for i, yaw in enumerate(yaws):
        x = odoms[i, 0]
        y = odoms[i, 1]
        ax1.arrow(x, y, 0.3 * math.cos(yaw), 0.3 * math.sin(yaw), head_width=0.05, head_length=0.05, fc='r', ec='r')
    
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.axis('equal')
    ax1.legend()
    
    ax2.set_xlabel('time (seconds)')
    ax2.set_ylabel('Z Position (m)')
    ax2.legend()
    plt.show()

if __name__ == "__main__":
    bag_path = sys.argv[1]
    namespace = '/' + sys.argv[2]
    odom_topic_name = namespace + '/odometry_conversion/odometry'
    tracking_point_topic_name = namespace + '/trajectory_controller/tracking_point'

    plot(bag_path, odom_topic_name, tracking_point_topic_name)
