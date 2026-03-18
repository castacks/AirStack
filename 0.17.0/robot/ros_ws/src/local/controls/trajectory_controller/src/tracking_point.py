#!/usr/bin/python
import rospy
from nav_msgs.msg import Odometry
import numpy as np
from tf import transformations as trans

if __name__ == '__main__':
    rospy.init_node('tracking_point', anonymous=True)

    odom_pub = rospy.Publisher('/tracking_point', Odometry, queue_size=10)
    odom = Odometry()
    odom.header.frame_id = 'map'
    odom.child_frame_id = 'base_link'
    odom.pose.pose.position.x = 16.
    odom.pose.pose.position.y = 5.
    odom.pose.pose.position.z = 3.

    q = trans.quaternion_from_euler(0., 0., np.pi/2.)
    
    odom.pose.pose.orientation.x = q[0]
    odom.pose.pose.orientation.y = q[1]
    odom.pose.pose.orientation.z = q[2]
    odom.pose.pose.orientation.w = q[3]

    rate = rospy.Rate(50.)

    while not rospy.is_shutdown():
        odom_pub.publish(odom)
        rate.sleep()
