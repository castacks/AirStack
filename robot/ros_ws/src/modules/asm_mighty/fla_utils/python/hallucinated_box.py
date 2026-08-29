#!/usr/bin/env python 

import rclpy
import math
from geometry_msgs.msg import PoseStamped
from fla_interfaces.msg import Box

def OnPose(pose_msg):
  global box_pub
  rclpy.logging.get_logger("pose") 
  position = (pose_msg.pose.position.x, \
    pose_msg.pose.position.y,           \
    pose_msg.pose.position.z)
  x_box = 25
  y_box = -5.6
  box_size_x = 1.0
  box_size_y = 1.0
  distance_threshold = 5.0
  x = x_box - position[0]
  y = y_box - position[1]

  if math.sqrt(x*x+y*y) < distance_threshold:
    box_msg = Box()
    box_msg.origin.x = x_box
    box_msg.origin.y = y_box
    box_msg.size.x = box_size_x
    box_msg.size.y = box_size_y
    box_pub.publish(box_msg)
  
  return

# rospy.init_node('hallucinated_box')
node = rclpy.create_node('hallucinated_box')
rclpy.get_global_executor().add_node(node)

# box_pub = rospy.Publisher('/fsm/fake_obstacle', Box, queue_size=1)
box_pub = node.create_publisher(Box, '/fsm/fake_obstacle', 1)

# rospy.Subscriber('/FLA_ACL02/pose', PoseStamped, OnPose)
sub = node.create_subscription(PoseStamped, '/FLA_ACL02/pose', OnPose)

rclpy.spin()
