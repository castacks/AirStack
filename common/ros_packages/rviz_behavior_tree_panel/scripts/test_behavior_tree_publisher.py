#!/usr/bin/env python3

"""
Test publisher for behavior tree visualization.
Publishes sample graphviz xdot data to the behavior_tree_graphviz topic.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class BehaviorTreeTestPublisher(Node):
    def __init__(self):
        super().__init__('behavior_tree_test_publisher')
        
        # Create publisher for behavior tree graphviz data
        self.publisher = self.create_publisher(String, 'behavior_tree_graphviz', 10)
        
        # Create timer to publish test data every 3 seconds
        self.timer = self.create_timer(3.0, self.publish_test_data)
        
        # Sample behavior tree scenarios
        self.test_scenarios = [
            self.create_simple_behavior_tree(),
            self.create_complex_behavior_tree(),
            self.create_running_behavior_tree()
        ]
        self.current_scenario = 0
        
        self.get_logger().info('Behavior Tree Test Publisher started')
        self.get_logger().info('Publishing to topic: behavior_tree_graphviz')

    def create_simple_behavior_tree(self):
        """Create a simple behavior tree with basic nodes"""
        return '''digraph G {
    graph [bgcolor=white, rankdir=TB, ranksep=0.3, nodesep=0.3];
    node [shape=box, style=filled, fontname="Arial", fontsize=10];
    
    // Root node
    root [label="Root\\nSequence", fillcolor=lightblue, shape=ellipse];
    
    // Child nodes
    check [label="Check\\nCondition", fillcolor=lightgreen];
    move [label="Move to\\nTarget", fillcolor=lightcoral];
    grab [label="Grab\\nObject", fillcolor=lightyellow];
    
    // Connections
    root -> check;
    root -> move;
    root -> grab;
}'''

    def create_complex_behavior_tree(self):
        """Create a more complex behavior tree with different node types"""
        return '''digraph G {
    graph [bgcolor=white, rankdir=TB, ranksep=0.4, nodesep=0.4];
    node [shape=box, style=filled, fontname="Arial", fontsize=9];
    
    // Root
    root [label="Root\\nSelector", fillcolor=lightblue, shape=ellipse];
    
    // Main branches
    emergency [label="Emergency\\nSequence", fillcolor=orange];
    normal [label="Normal\\nOperation", fillcolor=lightgreen];
    
    // Emergency branch
    detect_danger [label="Detect\\nDanger", fillcolor=red];
    stop_motors [label="Stop\\nMotors", fillcolor=red];
    alert [label="Send\\nAlert", fillcolor=red];
    
    // Normal operation branch
    nav_seq [label="Navigation\\nSequence", fillcolor=lightgreen];
    check_battery [label="Check\\nBattery", fillcolor=yellow];
    move_to_goal [label="Move to\\nGoal", fillcolor=lightcoral];
    avoid_obstacles [label="Avoid\\nObstacles", fillcolor=lightcoral];
    
    // Connections
    root -> emergency;
    root -> normal;
    
    emergency -> detect_danger;
    emergency -> stop_motors;
    emergency -> alert;
    
    normal -> nav_seq;
    nav_seq -> check_battery;
    nav_seq -> move_to_goal;
    nav_seq -> avoid_obstacles;
}'''

    def create_running_behavior_tree(self):
        """Create a behavior tree showing running states"""
        return '''digraph G {
    graph [bgcolor=white, rankdir=TB, ranksep=0.4, nodesep=0.4];
    node [shape=box, style=filled, fontname="Arial", fontsize=9];
    
    // Root (currently running)
    root [label="Root\\nSequence\\n[RUNNING]", fillcolor=cyan, shape=ellipse];
    
    // Completed nodes
    init [label="Initialize\\n[SUCCESS]", fillcolor=lightgreen];
    calibrate [label="Calibrate\\nSensors\\n[SUCCESS]", fillcolor=lightgreen];
    
    // Currently running node
    navigate [label="Navigate\\n[RUNNING]", fillcolor=yellow];
    
    // Pending nodes
    pickup [label="Pickup\\nObject\\n[IDLE]", fillcolor=lightgray];
    deliver [label="Deliver\\nObject\\n[IDLE]", fillcolor=lightgray];
    
    // Sub-navigation tree
    nav_check [label="Check\\nPath\\n[SUCCESS]", fillcolor=lightgreen];
    nav_move [label="Move\\nForward\\n[RUNNING]", fillcolor=yellow];
    nav_adjust [label="Adjust\\nHeading\\n[IDLE]", fillcolor=lightgray];
    
    // Connections
    root -> init;
    root -> calibrate;
    root -> navigate;
    root -> pickup;
    root -> deliver;
    
    navigate -> nav_check;
    navigate -> nav_move;
    navigate -> nav_adjust;
}'''

    def publish_test_data(self):
        """Publish the current test scenario"""
        msg = String()
        msg.data = self.test_scenarios[self.current_scenario]
        
        self.publisher.publish(msg)
        
        scenario_names = ["Simple Tree", "Complex Tree", "Running Tree"]
        self.get_logger().info(f'Published {scenario_names[self.current_scenario]}')
        
        # Cycle through scenarios
        self.current_scenario = (self.current_scenario + 1) % len(self.test_scenarios)

def main(args=None):
    rclpy.init(args=args)
    
    publisher = BehaviorTreeTestPublisher()
    
    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        publisher.get_logger().info('Test publisher stopped')
    finally:
        publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()