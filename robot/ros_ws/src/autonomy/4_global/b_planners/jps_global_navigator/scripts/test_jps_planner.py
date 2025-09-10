#!/usr/bin/env python3
"""
Simple test script to verify JPS Global Navigator functionality.

This script creates a simple cost map and sends a planning request to test
the JPS algorithm implementation.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point, PoseStamped
from task_msgs.action import NavigationTask
import struct
import time


class JPSPlannerTester(Node):
    def __init__(self):
        super().__init__('jps_planner_tester')
        
        # Create action client
        self.action_client = ActionClient(self, NavigationTask, 'navigation_task')
        
        # Create cost map publisher
        self.cost_map_pub = self.create_publisher(PointCloud2, '/cost_map', 10)
        
        self.get_logger().info('JPS Planner Tester initialized')

    def create_simple_cost_map(self):
        """Create a simple 3D cost map for testing."""
        points = []
        costs = []
        
        # Create a 10x10x10 grid with some obstacles
        for x in range(10):
            for y in range(10):
                for z in range(10):
                    # Add obstacle in the middle
                    if 4 <= x <= 6 and 4 <= y <= 6 and 4 <= z <= 6:
                        cost = 100.0  # High cost obstacle
                    else:
                        cost = 1.0    # Low cost free space
                    
                    points.append([float(x), float(y), float(z)])
                    costs.append(cost)
        
        # Create PointCloud2 message
        cloud_msg = PointCloud2()
        cloud_msg.header.frame_id = 'map'
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Define fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='cost', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud_msg.fields = fields
        
        # Pack data
        cloud_data = []
        for i, point in enumerate(points):
            cloud_data.append(struct.pack('ffff', point[0], point[1], point[2], costs[i]))
        
        cloud_msg.data = b''.join(cloud_data)
        cloud_msg.height = 1
        cloud_msg.width = len(points)
        cloud_msg.point_step = 16
        cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width
        cloud_msg.is_dense = True
        
        return cloud_msg

    def test_planning(self):
        """Test the JPS planning functionality."""
        self.get_logger().info('Starting JPS planning test...')
        
        # Wait for action server
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return False
        
        # Publish cost map
        cost_map = self.create_simple_cost_map()
        self.cost_map_pub.publish(cost_map)
        self.get_logger().info('Published cost map')
        
        # Wait a bit for cost map to be processed
        time.sleep(1.0)
        
        # Create goal
        goal_msg = NavigationTask.Goal()
        
        # Start position (0, 0, 0)
        start_pose = PoseStamped()
        start_pose.header.frame_id = 'map'
        start_pose.header.stamp = self.get_clock().now().to_msg()
        start_pose.pose.position.x = 0.0
        start_pose.pose.position.y = 0.0
        start_pose.pose.position.z = 0.0
        start_pose.pose.orientation.w = 1.0
        
        # Goal position (9, 9, 9) - should navigate around obstacle
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = 9.0
        goal_pose.pose.position.y = 9.0
        goal_pose.pose.position.z = 9.0
        goal_pose.pose.orientation.w = 1.0
        
        goal_msg.goal_poses = [goal_pose]
        goal_msg.max_planning_seconds = 30.0
        
        self.get_logger().info('Sending planning goal...')
        
        # Send goal
        send_goal_future = self.action_client.async_send_goal(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False
        
        self.get_logger().info('Goal accepted, waiting for result...')
        
        # Wait for result
        get_result_future = self.action_client.async_get_result(goal_handle)
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result = get_result_future.result()
        if result.result.success:
            self.get_logger().info(f'Planning successful! Path length: {len(result.result.path.poses)}')
            self.get_logger().info(f'Final position: ({result.result.final_pose.pose.position.x:.2f}, '
                                 f'{result.result.final_pose.pose.position.y:.2f}, '
                                 f'{result.result.final_pose.pose.position.z:.2f})')
            return True
        else:
            self.get_logger().error('Planning failed!')
            return False


def main():
    rclpy.init()
    
    tester = JPSPlannerTester()
    
    try:
        success = tester.test_planning()
        if success:
            tester.get_logger().info('JPS planner test PASSED!')
        else:
            tester.get_logger().error('JPS planner test FAILED!')
    except Exception as e:
        tester.get_logger().error(f'Test error: {e}')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()