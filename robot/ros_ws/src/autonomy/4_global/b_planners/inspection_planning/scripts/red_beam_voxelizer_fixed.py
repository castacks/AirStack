#!/usr/bin/env python3
"""
Red Beam Voxelizer - Fixed Version
Processes red beam point cloud data and publishes voxels based on confidence threshold.
Correctly handles packed RGB format.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
import sensor_msgs_py.point_cloud2 as pc2
from collections import defaultdict
import struct


class RedBeamVoxelizerFixed(Node):
    def __init__(self):
        super().__init__('red_beam_voxelizer_fixed')
        
        # Data storage
        self.voxel_data = defaultdict(list)  # voxel_key -> list of points
        self.voxel_size = 0.2
        self.confidence_threshold = 0.5
        self.min_points_per_voxel = 3
        
        # Subscribers
        self.subscription = self.create_subscription(
            PointCloud2,
            '/rayfronts/queries/red_beam',
            self.point_cloud_callback,
            10
        )
        
        # Publishers
        self.publisher = self.create_publisher(
            Marker,
            '/red_beam_voxels',
            10
        )
        
        self.get_logger().info('Red Beam Voxelizer Fixed initialized')
        self.get_logger().info(f'Input topic: /rayfronts/queries/red_beam')
        self.get_logger().info(f'Output topic: /red_beam_voxels')
        self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')
        self.get_logger().info(f'Voxel size: {self.voxel_size}')
        self.get_logger().info(f'Min points per voxel: {self.min_points_per_voxel}')

    def extract_rgb_from_packed(self, rgb_packed):
        """Extract R, G, B values from packed RGB format (32-bit integer)"""
        # Unpack the 32-bit integer as little-endian unsigned int
        rgb_int = struct.unpack('<I', rgb_packed)[0]
        
        # Extract R, G, B values (8 bits each)
        r = (rgb_int >> 16) & 0xFF
        g = (rgb_int >> 8) & 0xFF
        b = rgb_int & 0xFF
        
        return r, g, b

    def calculate_confidence(self, r, g, b):
        """Calculate confidence based on RGB values - closer to yellow = higher confidence"""
        # Convert to 0-1 range
        r_norm = r / 255.0
        g_norm = g / 255.0
        b_norm = b / 255.0
        
        # Yellow is (1, 1, 0), so confidence is based on how close we are to yellow
        # Higher R and G, lower B = higher confidence
        yellow_distance = np.sqrt((1.0 - r_norm)**2 + (1.0 - g_norm)**2 + (b_norm)**2)
        
        # Convert distance to confidence (closer to yellow = higher confidence)
        # Max distance is sqrt(2) ≈ 1.414, so normalize
        confidence = max(0.0, 1.0 - (yellow_distance / np.sqrt(2.0)))
        
        return confidence

    def point_cloud_callback(self, msg):
        """Process incoming point cloud data"""
        self.get_logger().info(f'Received point cloud with {msg.width} points')
        
        # Clear previous data
        self.voxel_data.clear()
        
        # Process points
        points_processed = 0
        points_above_threshold = 0
        confidence_values = []
        
        for point in pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True):
            x, y, z, rgb_packed = point
            
            # Extract RGB values from packed format
            r, g, b = self.extract_rgb_from_packed(rgb_packed)
            
            # Calculate confidence
            confidence = self.calculate_confidence(r, g, b)
            confidence_values.append(confidence)
            
            # Debug first few points
            if points_processed < 10:
                self.get_logger().info(f'Point {points_processed}: RGB({r},{g},{b}) -> confidence: {confidence:.4f}')
            
            # Only process points above threshold
            if confidence >= self.confidence_threshold:
                points_above_threshold += 1
                
                # Calculate voxel coordinates
                voxel_x = int(np.floor(x / self.voxel_size))
                voxel_y = int(np.floor(y / self.voxel_size))
                voxel_z = int(np.floor(z / self.voxel_size))
                voxel_key = (voxel_x, voxel_y, voxel_z)
                
                # Store point in voxel
                self.voxel_data[voxel_key].append((x, y, z, confidence))
            
            points_processed += 1
        
        # Log statistics
        if confidence_values:
            conf_array = np.array(confidence_values)
            self.get_logger().info('=== CONFIDENCE STATISTICS ===')
            self.get_logger().info(f'Total points processed: {points_processed}')
            self.get_logger().info(f'Points above threshold ({self.confidence_threshold}): {points_above_threshold}')
            self.get_logger().info(f'Confidence range: {conf_array.min():.4f} - {conf_array.max():.4f}')
            self.get_logger().info(f'Average confidence: {conf_array.mean():.4f}')
            self.get_logger().info(f'Median confidence: {np.median(conf_array):.4f}')
            
            # Categorize confidence levels
            very_low = np.sum(conf_array < 0.1)
            low = np.sum((conf_array >= 0.1) & (conf_array < 0.3))
            medium = np.sum((conf_array >= 0.3) & (conf_array < 0.6))
            high = np.sum((conf_array >= 0.6) & (conf_array < 0.8))
            very_high = np.sum(conf_array >= 0.8)
            
            self.get_logger().info(f'Very low (<0.1): {very_low}')
            self.get_logger().info(f'Low (0.1-0.3): {low}')
            self.get_logger().info(f'Medium (0.3-0.6): {medium}')
            self.get_logger().info(f'High (0.6-0.8): {high}')
            self.get_logger().info(f'Very high (>=0.8): {very_high}')
            self.get_logger().info('=============================')
        
        # Generate voxel markers
        self.publish_voxel_markers()

    def publish_voxel_markers(self):
        """Publish voxel markers for valid voxels"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "red_beam_voxels"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        
        # Set voxel size
        marker.scale.x = self.voxel_size
        marker.scale.y = self.voxel_size
        marker.scale.z = self.voxel_size
        
        # Color: red for red beam voxels
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        # Filter voxels with enough points
        valid_voxels = 0
        for voxel_key, points in self.voxel_data.items():
            if len(points) >= self.min_points_per_voxel:
                # Calculate voxel center
                voxel_x, voxel_y, voxel_z = voxel_key
                center_x = (voxel_x + 0.5) * self.voxel_size
                center_y = (voxel_y + 0.5) * self.voxel_size
                center_z = (voxel_z + 0.5) * self.voxel_size
                
                # Add voxel center to marker
                from geometry_msgs.msg import Point
                point = Point()
                point.x = center_x
                point.y = center_y
                point.z = center_z
                marker.points.append(point)
                
                valid_voxels += 1
        
        self.get_logger().info(f'Found {valid_voxels} valid voxels after filtering')
        
        # Publish marker
        self.publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = RedBeamVoxelizerFixed()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 