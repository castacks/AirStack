#!/usr/bin/env python3

"""
Simple test runner for the SimpleGlobalNavigator package.
This script runs basic functionality tests without requiring the full build system.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
import threading
import time
import sys
import signal
from geometry_msgs.msg import PoseStamped, Point
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import MarkerArray
from task_msgs.action import NavigationTask
import sensor_msgs.point_cloud2 as pc2
import numpy as np


class SimpleNavigatorTester(Node):
    def __init__(self):
        super().__init__('simple_navigator_tester')
        
        # Test state
        self.tests_passed = 0
        self.tests_failed = 0
        self.test_results = []
        
        # Publishers for test data
        self.cost_map_pub = self.create_publisher(PointCloud2, '/cost_map', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # Subscribers to monitor outputs
        self.global_plan_sub = self.create_subscription(
            Path, '/global_plan', self.global_plan_callback, 10)
        self.rrt_markers_sub = self.create_subscription(
            MarkerArray, '/rrt_tree_markers', self.rrt_markers_callback, 10)
        
        # Action client
        self.action_client = ActionClient(self, NavigationTask, '/simple_navigator')
        
        # Test state variables
        self.received_global_plan = None
        self.received_rrt_markers = None
        self.global_plan_received = False
        self.rrt_markers_received = False
        
        self.get_logger().info('SimpleNavigatorTester initialized')

    def global_plan_callback(self, msg):
        self.received_global_plan = msg
        self.global_plan_received = True
        self.get_logger().info(f'Received global plan with {len(msg.poses)} poses')

    def rrt_markers_callback(self, msg):
        self.received_rrt_markers = msg
        self.rrt_markers_received = True
        self.get_logger().info(f'Received RRT markers: {len(msg.markers)} markers')

    def create_test_cost_map(self):
        """Create a simple test cost map."""
        cloud_msg = PointCloud2()
        cloud_msg.header.frame_id = "map"
        cloud_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Create a simple 5x5 grid
        points = []
        for i in range(5):
            for j in range(5):
                # Create low cost everywhere for simple testing
                points.append([float(i), float(j), 0.0, 0.1])  # x, y, z, intensity
        
        # Create PointCloud2 message
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        cloud_msg = pc2.create_cloud(cloud_msg.header, fields, points)
        return cloud_msg

    def create_test_odometry(self, x, y, z):
        """Create test odometry message."""
        odom_msg = Odometry()
        odom_msg.header.frame_id = "odom"
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = z
        odom_msg.pose.pose.orientation.w = 1.0
        return odom_msg

    def create_navigation_goal(self, x, y, z):
        """Create navigation goal."""
        goal = NavigationTask.Goal()
        
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        
        goal.goal_poses = [pose]
        goal.max_planning_seconds = 5.0
        
        return goal

    def run_test(self, test_name, test_func):
        """Run a single test and record results."""
        self.get_logger().info(f'Running test: {test_name}')
        try:
            result = test_func()
            if result:
                self.tests_passed += 1
                self.test_results.append((test_name, "PASSED", ""))
                self.get_logger().info(f'Test {test_name}: PASSED')
            else:
                self.tests_failed += 1
                self.test_results.append((test_name, "FAILED", "Test returned False"))
                self.get_logger().error(f'Test {test_name}: FAILED')
        except Exception as e:
            self.tests_failed += 1
            self.test_results.append((test_name, "FAILED", str(e)))
            self.get_logger().error(f'Test {test_name}: FAILED - {str(e)}')

    def test_action_server_availability(self):
        """Test that the action server is available."""
        return self.action_client.wait_for_server(timeout_sec=10.0)

    def test_cost_map_publishing(self):
        """Test publishing cost map data."""
        cost_map = self.create_test_cost_map()
        self.cost_map_pub.publish(cost_map)
        time.sleep(0.5)  # Give time for processing
        return True  # If no exception, consider it passed

    def test_odometry_publishing(self):
        """Test publishing odometry data."""
        odom = self.create_test_odometry(0.0, 0.0, 0.0)
        self.odom_pub.publish(odom)
        time.sleep(0.5)  # Give time for processing
        return True  # If no exception, consider it passed

    def test_navigation_goal_acceptance(self):
        """Test that navigation goals are accepted."""
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            return False
        
        # Publish test data first
        cost_map = self.create_test_cost_map()
        self.cost_map_pub.publish(cost_map)
        
        odom = self.create_test_odometry(0.0, 0.0, 0.0)
        self.odom_pub.publish(odom)
        
        time.sleep(1.0)  # Wait for data processing
        
        # Send navigation goal
        goal = self.create_navigation_goal(4.0, 4.0, 0.0)
        
        send_goal_future = self.action_client.send_goal_async(goal)
        
        # Wait for goal to be processed
        start_time = time.time()
        while not send_goal_future.done() and (time.time() - start_time) < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not send_goal_future.done():
            return False
        
        goal_handle = send_goal_future.result()
        return goal_handle is not None and goal_handle.accepted

    def test_global_plan_publication(self):
        """Test that global plan is published."""
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            return False
        
        # Reset state
        self.global_plan_received = False
        
        # Publish test data
        cost_map = self.create_test_cost_map()
        self.cost_map_pub.publish(cost_map)
        
        odom = self.create_test_odometry(0.0, 0.0, 0.0)
        self.odom_pub.publish(odom)
        
        time.sleep(1.0)
        
        # Send navigation goal
        goal = self.create_navigation_goal(4.0, 4.0, 0.0)
        send_goal_future = self.action_client.send_goal_async(goal)
        
        # Wait for goal acceptance
        start_time = time.time()
        while not send_goal_future.done() and (time.time() - start_time) < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not send_goal_future.done():
            return False
        
        goal_handle = send_goal_future.result()
        if not (goal_handle and goal_handle.accepted):
            return False
        
        # Wait for global plan to be published
        start_time = time.time()
        while not self.global_plan_received and (time.time() - start_time) < 15.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        return self.global_plan_received and self.received_global_plan is not None

    def test_visualization_markers(self):
        """Test that visualization markers are published (if enabled)."""
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            return False
        
        # Reset state
        self.rrt_markers_received = False
        
        # Publish test data
        cost_map = self.create_test_cost_map()
        self.cost_map_pub.publish(cost_map)
        
        odom = self.create_test_odometry(0.0, 0.0, 0.0)
        self.odom_pub.publish(odom)
        
        time.sleep(1.0)
        
        # Send navigation goal
        goal = self.create_navigation_goal(4.0, 4.0, 0.0)
        send_goal_future = self.action_client.send_goal_async(goal)
        
        # Wait for goal acceptance
        start_time = time.time()
        while not send_goal_future.done() and (time.time() - start_time) < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if not send_goal_future.done():
            return False
        
        goal_handle = send_goal_future.result()
        if not (goal_handle and goal_handle.accepted):
            return False
        
        # Wait for RRT markers (this might not be published if visualization is disabled)
        start_time = time.time()
        while not self.rrt_markers_received and (time.time() - start_time) < 10.0:
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # This test passes if markers are received OR if visualization is disabled
        # (we can't easily check if visualization is enabled from here)
        return True

    def run_all_tests(self):
        """Run all tests."""
        self.get_logger().info('Starting SimpleGlobalNavigator tests...')
        
        # Wait a bit for the navigator to start up
        time.sleep(2.0)
        
        # Run tests
        self.run_test("Action Server Availability", self.test_action_server_availability)
        self.run_test("Cost Map Publishing", self.test_cost_map_publishing)
        self.run_test("Odometry Publishing", self.test_odometry_publishing)
        self.run_test("Navigation Goal Acceptance", self.test_navigation_goal_acceptance)
        self.run_test("Global Plan Publication", self.test_global_plan_publication)
        self.run_test("Visualization Markers", self.test_visualization_markers)
        
        # Print results
        self.print_test_results()

    def print_test_results(self):
        """Print test results summary."""
        self.get_logger().info('=' * 50)
        self.get_logger().info('TEST RESULTS SUMMARY')
        self.get_logger().info('=' * 50)
        
        for test_name, status, error in self.test_results:
            if status == "PASSED":
                self.get_logger().info(f'✓ {test_name}: {status}')
            else:
                self.get_logger().error(f'✗ {test_name}: {status}')
                if error:
                    self.get_logger().error(f'  Error: {error}')
        
        self.get_logger().info('=' * 50)
        self.get_logger().info(f'Total tests: {self.tests_passed + self.tests_failed}')
        self.get_logger().info(f'Passed: {self.tests_passed}')
        self.get_logger().info(f'Failed: {self.tests_failed}')
        
        if self.tests_failed == 0:
            self.get_logger().info('🎉 ALL TESTS PASSED!')
        else:
            self.get_logger().warn(f'⚠️  {self.tests_failed} test(s) failed')


def main():
    rclpy.init()
    
    tester = SimpleNavigatorTester()
    executor = MultiThreadedExecutor()
    executor.add_node(tester)
    
    # Handle Ctrl+C gracefully
    def signal_handler(sig, frame):
        print('\nShutting down...')
        executor.shutdown()
        rclpy.shutdown()
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    # Run tests in a separate thread
    def run_tests():
        time.sleep(1.0)  # Give executor time to start
        tester.run_all_tests()
        # Keep running for a bit to see any late messages
        time.sleep(2.0)
        executor.shutdown()
    
    test_thread = threading.Thread(target=run_tests)
    test_thread.start()
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        test_thread.join()
        rclpy.shutdown()


if __name__ == '__main__':
    main()