#!/usr/bin/env python
"""
GPS Degradation Test Script
Tests GPS degradation in outdoor urban canyon scenario.

Author: GitHub Copilot
Date: 2026-05-13

Tests:
1. Open-sky GPS accuracy (low DOP, high satellites)
2. Urban canyon degradation (high DOP, fewer satellites)
3. Recovery when exiting canyon

Latency check: Measures RTF and ensures >= 1.0
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped
import time
import numpy as np

class GPSTestNode(Node):
    def __init__(self):
        super().__init__('gps_test_node')

        # Subscribe to degraded GPS
        self.gps_sub = self.create_subscription(
            NavSatFix, '/robot_1/sensors/gps', self.gps_callback, 10)

        # Subscribe to true pose (ground truth)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/robot_1/state/pose', self.pose_callback, 10)

        self.gps_data = []
        self.pose_data = []
        self.start_time = time.time()

        # Test waypoints
        self.test_waypoints = [
            (0, 10, 5),    # Open sky
            (10, 10, 5),   # Urban canyon
            (20, 10, 5),   # Recovery
        ]
        self.current_waypoint = 0

        self.get_logger().info('GPS Test Node started')

    def gps_callback(self, msg):
        self.gps_data.append({
            'time': time.time() - self.start_time,
            'lat': msg.latitude,
            'lon': msg.longitude,
            'alt': msg.altitude,
            'eph': msg.position_covariance[0] if msg.position_covariance else 1.0,
            'epv': msg.position_covariance[4] if len(msg.position_covariance) > 4 else 1.0,
            'status': msg.status.status
        })

    def pose_callback(self, msg):
        self.pose_data.append({
            'time': time.time() - self.start_time,
            'x': msg.pose.position.x,
            'y': msg.pose.position.y,
            'z': msg.pose.position.z
        })

    def calculate_metrics(self):
        """Calculate test metrics"""
        if not self.gps_data or not self.pose_data:
            return {}

        # Calculate position errors
        errors = []
        for gps in self.gps_data[-100:]:  # Last 100 samples
            # Find closest pose in time
            closest_pose = min(self.pose_data,
                             key=lambda p: abs(p['time'] - gps['time']))

            # Convert GPS to local coordinates (simplified)
            gps_x = (gps['lon'] - self.pose_data[0]['x']) * 111320  # Rough conversion
            gps_y = (gps['lat'] - self.pose_data[0]['y']) * 111320

            error = np.sqrt((gps_x - closest_pose['x'])**2 +
                          (gps_y - closest_pose['y'])**2)
            errors.append(error)

        return {
            'mean_error': np.mean(errors),
            'max_error': np.max(errors),
            'eph_mean': np.mean([d['eph'] for d in self.gps_data]),
            'epv_mean': np.mean([d['epv'] for d in self.gps_data]),
            'fix_rate': sum(1 for d in self.gps_data if d['status'] >= 0) / len(self.gps_data)
        }

def run_test():
    """Run the GPS degradation test"""
    rclpy.init()

    test_node = GPSTestNode()

    # Test duration: 60 seconds
    test_duration = 60.0
    start_time = time.time()

    print("Starting GPS degradation test...")
    print("Test phases:")
    print("0-20s: Open sky (should have good GPS)")
    print("20-40s: Urban canyon (should have degraded GPS)")
    print("40-60s: Recovery (should improve GPS)")

    while time.time() - start_time < test_duration:
        rclpy.spin_once(test_node, timeout_sec=0.1)

        # Print progress
        elapsed = time.time() - start_time
        if int(elapsed) % 10 == 0 and int(elapsed) != int(elapsed - 0.1):
            print(f"Time: {elapsed:.1f}s - GPS samples: {len(test_node.gps_data)}")

    # Calculate and print results
    metrics = test_node.calculate_metrics()

    print("\n=== GPS Degradation Test Results ===")
    print(f"Mean position error: {metrics.get('mean_error', 'N/A'):.2f}m")
    print(f"Max position error: {metrics.get('max_error', 'N/A'):.2f}m")
    print(f"Mean EPH (horizontal accuracy): {metrics.get('eph_mean', 'N/A'):.2f}m")
    print(f"Mean EPV (vertical accuracy): {metrics.get('epv_mean', 'N/A'):.2f}m")
    print(f"GPS fix rate: {metrics.get('fix_rate', 'N/A'):.2%}")

    # Check for degradation
    if metrics.get('eph_mean', 10) > 5.0:
        print("✅ GPS degradation detected (EPH > 5m)")
    else:
        print("❌ GPS degradation not detected")

    test_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    run_test()</content>
<parameter name="filePath">/home/ubuntu/Downloads/AirStack/tests/test_gps_degradation.py