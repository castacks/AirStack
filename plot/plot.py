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

    odoms = []
    tps = []
    yaws = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == odom_topic_name:
            msg = deserialize_message(data, get_message(type_map[odom_topic_name]))
            odoms.append([msg.pose.pose.position.x, msg.pose.pose.position.y])
            
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            yaws.append(yaw)
        elif topic == tracking_point_topic_name:
            msg = deserialize_message(data, get_message(type_map[tracking_point_topic_name]))
            tps.append([msg.pose.position.x, msg.pose.position.y])
            

    odoms = np.array(odoms)
    tps = np.array(tps)
    
    plt.figure()
    plt.plot(odoms[:, 0], odoms[:, 1], marker='.', linestyle='-', label='odom')
    plt.plot(tps[:, 0], tps[:, 1], marker='.', linestyle='-', label='tracking point')

    for i, yaw in enumerate(yaws):
        x = odoms[i, 0]
        y = odoms[i, 1]
        plt.arrow(x, y, 0.3 * math.cos(yaw), 0.3 * math.sin(yaw), head_width=0.05, head_length=0.05, fc='r', ec='r')
    
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.axis('equal')
    plt.legend()
    plt.show()

if __name__ == "__main__":
    bag_path = sys.argv[1]
    namespace = '/' + sys.argv[2]
    odom_topic_name = namespace + '/odometry_conversion/odometry'
    tracking_point_topic_name = namespace + '/trajectory_controller/tracking_point'

    plot(bag_path, odom_topic_name, tracking_point_topic_name)
