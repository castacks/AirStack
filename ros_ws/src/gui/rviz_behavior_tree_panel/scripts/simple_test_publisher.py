#!/usr/bin/env python3

"""
Simple test publisher for behavior tree visualization.
Publishes a basic graphviz tree to test the panel.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from behavior_tree_msgs.msg import GraphVizXdot

class SimpleBehaviorTreePublisher(Node):
    def __init__(self):
        super().__init__('simple_behavior_tree_publisher')
        
        # Create publisher for behavior tree graphviz data
        self.publisher = self.create_publisher(GraphVizXdot, 'behavior_tree_graphviz', 10)
        
        # Create timer to publish test data every 5 seconds (slower)
        self.timer = self.create_timer(5.0, self.publish_simple_tree)
        
        # Counter to make each message slightly different
        self.counter = 0
        
        self.get_logger().info('Simple Behavior Tree Test Publisher started')

    def publish_simple_tree(self):
        """Publish a very simple behavior tree with changing states"""
        self.counter += 1
        
        # Alternate between different states to show refresh working
        if self.counter % 3 == 1:
            action1_color = "lightgreen"
            action1_label = "Action 1 (SUCCESS)"
            action2_color = "lightcoral" 
            action2_label = "Action 2 (IDLE)"
        elif self.counter % 3 == 2:
            action1_color = "yellow"
            action1_label = "Action 1 (RUNNING)"
            action2_color = "lightgreen"
            action2_label = "Action 2 (SUCCESS)"
        else:
            action1_color = "lightcoral"
            action1_label = "Action 1 (FAILURE)"
            action2_color = "yellow"
            action2_label = "Action 2 (RUNNING)"
        
        simple_tree = f'''digraph G {{
    rankdir=TB;
    node [shape=box, style=filled];
    
    root [label="Root (Update {self.counter})", fillcolor=lightblue];
    action1 [label="{action1_label}", fillcolor={action1_color}];
    action2 [label="{action2_label}", fillcolor={action2_color}];
    
    root -> action1;
    root -> action2;
}}'''
        
        msg = GraphVizXdot()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.xdot.data = simple_tree
        self.publisher.publish(msg)
        self.get_logger().info(f'Published behavior tree update {self.counter}')

def main(args=None):
    rclpy.init(args=args)
    
    publisher = SimpleBehaviorTreePublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.get_logger().info('Simple test publisher stopped')
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()