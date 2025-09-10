#!/usr/bin/env python3
"""
VDB Point Cloud Analyzer
Subscribes to VDB visualization marker and point cloud topics to calculate
the percentage of point cloud points that coincide with VDB grid cells.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from visualization_msgs.msg import Marker
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import Point
import struct


class VDBPointCloudAnalyzer(Node):
    def __init__(self):
        super().__init__('vdb_pointcloud_analyzer')
        
        # Initialize variables
        self.latest_vdb_marker = None
        self.latest_pointcloud = None
        self.vdb_grid_points = set()
        self.grid_resolution = 0.1  # Default grid resolution, will be updated from marker
        
        # Subscribers
        self.vdb_sub = self.create_subscription(
            Marker,
            '/robot_1/exploration_planner/vdb_viz',
            self.vdb_callback,
            10
        )
        
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/rayfronts/voxel_rgb',
            self.pointcloud_callback,
            10
        )
        
        # Timer for periodic analysis
        self.timer = self.create_timer(2.0, self.analyze_coincidence)
        
        self.get_logger().info('VDB Point Cloud Analyzer initialized')
        self.get_logger().info('Subscribing to:')
        self.get_logger().info('  - /robot_1/exploration_planner/vdb_viz (visualization_msgs/Marker)')
        self.get_logger().info('  - /rayfronts/voxel_rgb (sensor_msgs/PointCloud2)')

    def vdb_callback(self, msg):
        """Process VDB visualization marker to accumulate grid points."""
        self.latest_vdb_marker = msg
        
        if msg.type == Marker.CUBE_LIST:
            # Extract grid resolution from scale (assuming uniform voxels)
            self.grid_resolution = msg.scale.x
            
            # Accumulate all cube center positions (don't clear existing ones)
            new_points = 0
            for i in range(0, len(msg.points)):
                point = msg.points[i]
                # Discretize to grid coordinates
                grid_x = round(point.x / self.grid_resolution)
                grid_y = round(point.y / self.grid_resolution)
                grid_z = round(point.z / self.grid_resolution)
                grid_coord = (grid_x, grid_y, grid_z)
                
                if grid_coord not in self.vdb_grid_points:
                    self.vdb_grid_points.add(grid_coord)
                    new_points += 1
                
        elif msg.type == Marker.POINTS:
            # For point markers, accumulate the points
            new_points = 0
            for i in range(0, len(msg.points)):
                point = msg.points[i]
                # Discretize to grid coordinates
                grid_x = round(point.x / self.grid_resolution)
                grid_y = round(point.y / self.grid_resolution)
                grid_z = round(point.z / self.grid_resolution)
                grid_coord = (grid_x, grid_y, grid_z)
                
                if grid_coord not in self.vdb_grid_points:
                    self.vdb_grid_points.add(grid_coord)
                    new_points += 1
        
        self.get_logger().info(f'VDB marker: +{new_points} new, total: {len(self.vdb_grid_points)} grid cells')

    def pointcloud_callback(self, msg):
        """Process point cloud message."""
        self.latest_pointcloud = msg
        self.get_logger().debug('Point cloud received')

    def point_to_grid_coord(self, x, y, z):
        """Convert world coordinates to grid coordinates."""
        grid_x = round(x / self.grid_resolution)
        grid_y = round(y / self.grid_resolution)
        grid_z = round(z / self.grid_resolution)
        return (grid_x, grid_y, grid_z)

    def analyze_coincidence(self):
        """Analyze the coincidence between point cloud and VDB grid."""
        if self.latest_vdb_marker is None or self.latest_pointcloud is None:
            self.get_logger().warn('Waiting for both VDB marker and point cloud data...')
            return
            
        if len(self.vdb_grid_points) == 0:
            self.get_logger().warn('No VDB grid points available')
            return

        try:
            # Extract points from point cloud
            points = []
            for point in pc2.read_points(self.latest_pointcloud, 
                                       field_names=("x", "y", "z"), 
                                       skip_nans=True):
                points.append(point)
            
            if len(points) == 0:
                self.get_logger().warn('No valid points in point cloud')
                return
            
            # Count coincident points
            coincident_count = 0
            total_points = len(points)
            
            for point in points:
                # Handle different point formats
                try:
                    if hasattr(point, '__len__') and len(point) >= 3:
                        x, y, z = point[0], point[1], point[2]
                    elif hasattr(point, 'x'):
                        x, y, z = point.x, point.y, point.z
                    else:
                        # Point might be a scalar - convert to float
                        x, y, z = float(point[0]), float(point[1]), float(point[2])
                    
                    grid_coord = self.point_to_grid_coord(x, y, z)
                    
                    if grid_coord in self.vdb_grid_points:
                        coincident_count += 1
                        
                except (TypeError, IndexError, AttributeError) as e:
                    # Skip invalid points
                    continue
            
            # Calculate percentage
            percentage = (coincident_count / total_points) * 100.0
            
            # Log results
            self.get_logger().info('=== VDB Point Cloud Coincidence Analysis ===')
            self.get_logger().info(f'Grid resolution: {self.grid_resolution:.3f} m')
            self.get_logger().info(f'VDB grid cells: {len(self.vdb_grid_points)}')
            self.get_logger().info(f'Point cloud points: {total_points}')
            self.get_logger().info(f'Coincident points: {coincident_count}')
            self.get_logger().info(f'Coincidence percentage: {percentage:.2f}%')
            self.get_logger().info('=' * 45)
            
        except Exception as e:
            self.get_logger().error(f'Error during analysis: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    analyzer = VDBPointCloudAnalyzer()
    
    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        analyzer.get_logger().info('Shutting down VDB Point Cloud Analyzer...')
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()