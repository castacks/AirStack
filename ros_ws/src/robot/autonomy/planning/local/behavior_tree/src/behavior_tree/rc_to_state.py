#!/usr/bin/python
import rospy
from mavros_msgs.msg import RCIn
from std_msgs.msg import String

switch = None
prev_switch = None
channel_upper_bound = 2000
channel_lower_bound = 1000
toggles = 0


def rc_callback(rc):
    global prev_switch, switch, toggles
    channel = rc.channels[8]
    if channel >= channel_upper_bound:
        switch = True
    elif channel <= channel_lower_bound:
        switch = False
    
    if prev_switch == None or switch == None:
        prev_switch = switch
        return
    
    if switch != prev_switch:
        toggles += 1
        transition = String()

        if toggles == 1:
            transition.data = 'rewind'
            transition_pub.publish(transition)
        elif toggles == 2:
            transition.data = 'land'
            transition_pub.publish(transition)
    
    prev_switch = switch


if __name__ == '__main__':
    rospy.init_node('rc_to_state')
    
    rc_sub = rospy.Subscriber('/mavros/rc/in', RCIn, rc_callback)
    transition_pub = rospy.Publisher('/transition', String, queue_size=1)

    rospy.spin()
