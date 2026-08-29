#!/usr/bin/env python 
# AUTHOR: Kris Frey <kfrey@mit.edu>

#import roslib
import rospy

import tf
from geometry_msgs.msg import PoseStamped

def convert_pose2tf(pose_msg):
  
  if use_local_time:
    ts = rospy.Time.now()
  else:
    ts = pose_msg.header.stamp

  position = (pose_msg.pose.position.x, \
    pose_msg.pose.position.y,           \
    pose_msg.pose.position.z)
  
  q = (pose_msg.pose.orientation.x,     \
    pose_msg.pose.orientation.y,        \
    pose_msg.pose.orientation.z,        \
    pose_msg.pose.orientation.w) 

  br.sendTransform(position,
    q,
    ts,
    tf_child_frame_id,
    pose_msg.header.frame_id
    )
  return

rospy.init_node('pose2tf')

tf_child_frame_id = rospy.get_param('~child_frame_id')

use_local_time = False;
if rospy.has_param('~use_local_time'):
  use_local_time = rospy.get_param('~use_local_time')

br = tf.TransformBroadcaster()
rospy.Subscriber('pose_in', PoseStamped, convert_pose2tf)

rospy.spin()
