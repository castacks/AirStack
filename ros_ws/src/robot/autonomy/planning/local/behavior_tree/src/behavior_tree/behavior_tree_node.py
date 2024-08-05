#!/usr/bin/python
import rospy
from std_msgs.msg import String, Bool
import behavior_tree as bt
import behavior_tree_graphviz as gv
#import cv2
import zlib

class BehaviorTreeNode:
    def __init__(self, config_filename):
        self.tree = bt.BehaviorTree(config_filename)
        for node in self.tree.nodes:
            node.init_ros()

def timer_callback(event):
    node.tree.tick()#root.tick(True)

    source = gv.get_graphviz(node.tree)
    source_msg = String()
    source_msg.data = source
    graphviz_pub.publish(source_msg)

    compressed = String()
    compressed.data = zlib.compress(source)
    compressed_pub.publish(compressed)
    '''
    img = gv.get_graphviz_image(source)
    cv2.imshow('img', img)
    cv2.waitKey(1)
    '''

if __name__ == '__main__':
    rospy.init_node('behavior_tree_node')
    
    config_filename = rospy.get_param('~config', '')
    
    node = BehaviorTreeNode(config_filename)

    graphviz_pub = rospy.Publisher('behavior_tree_graphviz', String, queue_size=1)
    compressed_pub = rospy.Publisher('behavior_tree_graphviz_compressed', String, queue_size=1)
    timer = rospy.Timer(rospy.Duration(0.05), timer_callback)

    rospy.spin()
