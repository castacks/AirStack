#!/usr/bin/python3
import rosbag2_py
import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import sys
import math

def get_time(stamp):
    return stamp.sec + stamp.nanosec*1e-9

def plot(bag_path, odom_topic_name, tracking_point_topic_name):
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='', output_serialization_format='')
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}
    msg_type = get_message(type_map[odom_topic_name])

    start_time = None
    
    odoms = []
    vels = []
    odom_times = []
    tps = []
    tp_times = []
    yaws = []

    split_times = []
    last_time = 0.
    
    while reader.has_next():
        topic, data, recorded_time = reader.read_next()
        if start_time != None:
            last_time = recorded_time/1e9 - start_time
        if topic == odom_topic_name:
            msg = deserialize_message(data, get_message(type_map[odom_topic_name]))
            odoms.append([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z])
            vels.append([np.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2 + msg.twist.twist.linear.z**2)])

            t = get_time(msg.header.stamp)
            if start_time == None:
                start_time = t
            odom_times.append(t - start_time)
            
            q = msg.pose.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            yaws.append(yaw)
        elif topic == tracking_point_topic_name:
            msg = deserialize_message(data, get_message(type_map[tracking_point_topic_name]))
            tps.append([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])

            t = get_time(msg.header.stamp)
            if start_time == None:
                start_time = t
            tp_times.append(t - start_time)
        elif topic == fixed_trajectory_command_name:
            split_times.append(last_time)
    
    if len(split_times) == 0:
        split_times.append(last_time - 1.)
    print(last_time)
            

    odoms = np.array(odoms)
    tps = np.array(tps)

    prev_odom_split_index = 0
    prev_tp_split_index = 0
    fig_count = 0
    for split_time in split_times:
        fig = plt.figure()
        ax1 = fig.add_subplot(111)
        #ax2 = fig.add_subplot(212)

        odom_start_index = prev_odom_split_index
        odom_end_index = np.where(split_time < np.array(odom_times))[0][0]
        tp_start_index = prev_tp_split_index
        tp_end_index = np.where(split_time < np.array(tp_times))[0][0]
        
        fig.suptitle('Max Velocity: %0.02f m/s' % np.max(vels[odom_start_index:odom_end_index]))
        
        ax1.plot(odoms[odom_start_index:odom_end_index, 0], odoms[odom_start_index:odom_end_index, 1], marker='.', linestyle='-', label='odom')
        ax1.plot(tps[tp_start_index:tp_end_index, 0], tps[tp_start_index:tp_end_index, 1], marker='.', linestyle='-', label='tracking point')

        #ax2.plot(odom_times[odom_start_index:odom_end_index], odoms[odom_start_index:odom_end_index, 2], linestyle='-', label='odom')
        #ax2.plot(tp_times[tp_start_index:tp_end_index], tps[tp_start_index:tp_end_index, 2], linestyle='-', label='tracking point')

        for i, yaw in enumerate(yaws[odom_start_index:odom_end_index]):
            x = odoms[i, 0]
            y = odoms[i, 1]
            #ax1.arrow(x, y, 0.3 * math.cos(yaw), 0.3 * math.sin(yaw), head_width=0.05, head_length=0.05, fc='r', ec='r')

        ax1.set_xlabel('X Position (m)')
        ax1.set_ylabel('Y Position (m)')
        ax1.axis('equal')
        ax1.legend()

        #ax2.set_xlabel('time (seconds)')
        #ax2.set_ylabel('Z Position (m)')
        #ax2.legend()
        fig.savefig(str(fig_count) + '.png')
        fig_count += 1
        plt.show()
        
        prev_odom_split_index = odom_end_index
        prev_tp_split_index = tp_end_index

if __name__ == "__main__":
    bag_path = sys.argv[1]
    namespace = '/' + sys.argv[2]
    odom_topic_name = namespace + '/odometry_conversion/odometry'
    tracking_point_topic_name = namespace + '/trajectory_controller/tracking_point'
    fixed_trajectory_command_name = namespace + '/fixed_trajectory_generator/fixed_trajectory_command'

    plot(bag_path, odom_topic_name, tracking_point_topic_name)
