#!/usr/bin/env python3

"""
Test script for verifying time constraint functionality in RRTStarGlobalNavigator.
This script sends navigation goals with different time constraints and monitors the results.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from task_msgs.action import NavigationTask
from geometry_msgs.msg import PoseStamped
import time


class TimeConstraintTester(Node):
    def __init__(self):
        super().__init__('time_constraint_tester')
        
        self.action_client = ActionClient(self, NavigationTask, '/rrtstar_navigator')
        
        # Wait for action server
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Action server available!')

    def create_test_goal(self, x, y, z, max_planning_seconds):
        """Create a test navigation goal."""
        goal_msg = NavigationTask.Goal()
        
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.w = 1.0
        
        goal_msg.goal_poses = [pose]
        goal_msg.max_planning_seconds = float(max_planning_seconds)
        
        return goal_msg

    def test_time_constraint(self, x, y, z, max_planning_seconds, test_name):
        """Test navigation with specific time constraint."""
        self.get_logger().info(f'Starting test: {test_name}')
        self.get_logger().info(f'Goal: ({x}, {y}, {z}), Time limit: {max_planning_seconds}s')
        
        goal_msg = self.create_test_goal(x, y, z, max_planning_seconds)
        
        # Send goal
        start_time = time.time()
        send_goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        
        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected for test: {test_name}')
            return False
        
        self.get_logger().info(f'Goal accepted for test: {test_name}')
        
        # Wait for result
        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)
        
        result = get_result_future.result().result
        actual_time = time.time() - start_time
        
        # Log results
        self.get_logger().info(f'Test {test_name} completed:')
        self.get_logger().info(f'  Success: {result.success}')
        self.get_logger().info(f'  Message: {result.message}')
        self.get_logger().info(f'  Actual time: {actual_time:.2f}s')
        self.get_logger().info(f'  Distance traveled: {result.distance_traveled:.2f}m')
        
        # Verify time constraint was respected
        if max_planning_seconds > 0 and actual_time > max_planning_seconds + 1.0:  # 1s tolerance
            self.get_logger().warn(f'Time constraint violated! Expected: {max_planning_seconds}s, Actual: {actual_time:.2f}s')
            return False
        
        return True

    def feedback_callback(self, feedback_msg):
        """Handle feedback from navigation action."""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback - Phase: {feedback.current_phase}, '
                              f'Distance remaining: {feedback.distance_remaining:.2f}m, '
                              f'Time elapsed: {feedback.time_elapsed:.2f}s')

    def run_tests(self):
        """Run a series of time constraint tests."""
        tests = [
            # (x, y, z, max_planning_seconds, test_name)
            (5.0, 3.0, 1.0, 10.0, "Short time limit"),
            (10.0, 8.0, 2.0, 30.0, "Medium time limit"),
            (15.0, 12.0, 3.0, -1.0, "No time limit"),
            (20.0, 15.0, 1.5, 5.0, "Very short time limit"),
        ]
        
        results = []
        for test_params in tests:
            success = self.test_time_constraint(*test_params)
            results.append((test_params[4], success))
            
            # Wait between tests
            self.get_logger().info('Waiting 5 seconds before next test...')
            time.sleep(5)
        
        # Summary
        self.get_logger().info('=== TEST SUMMARY ===')
        for test_name, success in results:
            status = "PASSED" if success else "FAILED"
            self.get_logger().info(f'{test_name}: {status}')


def main(args=None):
    rclpy.init(args=args)
    
    tester = TimeConstraintTester()
    
    try:
        tester.run_tests()
    except KeyboardInterrupt:
        tester.get_logger().info('Test interrupted by user')
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()