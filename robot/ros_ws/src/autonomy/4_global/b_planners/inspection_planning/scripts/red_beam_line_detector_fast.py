#!/usr/bin/env python3
"""
Red Beam Line Detector - Fast Version
Uses optimized line detection algorithms to find red beams.
Only detects lines along 50° ± 15° with respect to X-axis.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2
from collections import defaultdict
import struct
import colorsys
import geometry_msgs.msg
import std_msgs.msg
import math


class RedBeamLineDetectorFast(Node):
    def __init__(self):
        super().__init__('red_beam_line_detector_fast')
        
        # Data storage
        self.voxels = []
        self.voxel_size = 0.2
        self.point_cloud_data = None
        
        # Parameters
        self.confidence_threshold = 0.1
        self.min_points_per_voxel = 3
        self.min_beam_length = 5.0  # Minimum length for a valid beam
        self.beam_thickness_threshold = 0.3  # Maximum width for a valid beam
        self.cluster_distance = 1.0
        
        # Line direction constraints - 50° ± 15° with respect to X-axis
        self.target_direction_angle = 50.0  # 50 degrees from X-axis
        self.angle_tolerance = 15.0  # ±15 degrees tolerance
        self.min_angle_rad = math.radians(self.target_direction_angle - self.angle_tolerance)  # 35°
        self.max_angle_rad = math.radians(self.target_direction_angle + self.angle_tolerance)  # 65°
        
        # Optimized line detection parameters
        self.ransac_threshold = 0.2  # Increased threshold for faster processing
        self.min_line_points = 3  # Reduced minimum points
        self.max_iterations = 200  # Significantly reduced iterations
        self.max_voxels_for_processing = 1000  # Limit voxels for processing
        
        # Subscribers
        self.subscription = self.create_subscription(
            PointCloud2,
            '/rayfronts/queries/red_beam',
            self.point_cloud_callback,
            10
        )
        
        # Publishers
        self.voxel_publisher = self.create_publisher(
            Marker,
            '/red_beam_voxels',
            10
        )
        
        self.beam_publisher = self.create_publisher(
            MarkerArray,
            '/red_beam_clusters',
            10
        )
        
        self.get_logger().info('Red Beam Line Detector Fast initialized')
        self.get_logger().info(f'Input topic: /rayfronts/queries/red_beam')
        self.get_logger().info(f'Voxel output topic: /red_beam_voxels')
        self.get_logger().info(f'Beam output topic: /red_beam_clusters')
        self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')
        self.get_logger().info(f'Voxel size: {self.voxel_size}')
        self.get_logger().info(f'Min points per voxel: {self.min_points_per_voxel}')
        self.get_logger().info(f'Min beam length: {self.min_beam_length}m')
        self.get_logger().info(f'Beam thickness threshold: {self.beam_thickness_threshold}m')
        self.get_logger().info(f'Target direction: {self.target_direction_angle}° ± {self.angle_tolerance}° from X-axis')
        self.get_logger().info(f'Max iterations: {self.max_iterations}')
        self.get_logger().info(f'Max voxels for processing: {self.max_voxels_for_processing}')

    def point_cloud_callback(self, msg):
        self.get_logger().info(f'Received point cloud with {msg.width} points')
        
        # Process point cloud
        self.process_point_cloud(msg)
        
        # Detect lines
        beams = self.detect_lines_fast()
        
        # Publish results
        self.publish_voxels()
        self.publish_beams(beams)

    def process_point_cloud(self, msg):
        """Process point cloud and create voxels"""
        self.voxels = []
        voxel_map = defaultdict(list)
        
        # Read point cloud data
        points = list(pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True))
        
        if not points:
            self.get_logger().warn('No points found in point cloud')
            return
        
        # Process each point
        points_above_threshold = 0
        confidence_values = []
        
        for i, point in enumerate(points):
            x, y, z, rgb = point
            
            # Extract RGB values
            try:
                rgb_bytes = struct.pack('f', float(rgb))
                rgb_int = struct.unpack('I', rgb_bytes)[0]
                
                r = (rgb_int >> 16) & 0xFF
                g = (rgb_int >> 8) & 0xFF
                b = rgb_int & 0xFF
                
            except Exception as e:
                try:
                    rgb_int = int(rgb)
                    r = (rgb_int >> 16) & 0xFF
                    g = (rgb_int >> 8) & 0xFF
                    b = rgb_int & 0xFF
                except:
                    r, g, b = 0, 0, 0
            
            # Calculate confidence
            confidence = (r + g) / (2 * 255.0) - (b / 255.0)
            confidence = max(0.0, min(1.0, confidence))
            
            confidence_values.append(confidence)
            
            if confidence >= self.confidence_threshold:
                points_above_threshold += 1
                
                # Calculate voxel coordinates
                voxel_x = int(x / self.voxel_size)
                voxel_y = int(y / self.voxel_size)
                voxel_z = int(z / self.voxel_size)
                
                voxel_key = (voxel_x, voxel_y, voxel_z)
                voxel_map[voxel_key].append((x, y, z, confidence))
        
        # Debug confidence statistics
        if confidence_values:
            self.get_logger().info(f'Confidence stats: min={min(confidence_values):.4f}, max={max(confidence_values):.4f}, avg={sum(confidence_values)/len(confidence_values):.4f}')
            self.get_logger().info(f'Points with confidence >= {self.confidence_threshold}: {sum(1 for c in confidence_values if c >= self.confidence_threshold)}')
        
        self.get_logger().info(f'Processed {len(points)} points, {points_above_threshold} above threshold')
        
        # Create voxels from points
        for voxel_key, points_in_voxel in voxel_map.items():
            if len(points_in_voxel) >= self.min_points_per_voxel:
                # Calculate voxel center and average confidence
                avg_x = sum(p[0] for p in points_in_voxel) / len(points_in_voxel)
                avg_y = sum(p[1] for p in points_in_voxel) / len(points_in_voxel)
                avg_z = sum(p[2] for p in points_in_voxel) / len(points_in_voxel)
                avg_confidence = sum(p[3] for p in points_in_voxel) / len(points_in_voxel)
                
                self.voxels.append({
                    'center': (avg_x, avg_y, avg_z),
                    'confidence': avg_confidence,
                    'point_count': len(points_in_voxel)
                })
        
        self.get_logger().info(f'Found {len(self.voxels)} valid voxels')

    def detect_lines_fast(self):
        """Fast line detection using optimized approach"""
        if not self.voxels:
            return []
        
        self.get_logger().info(f'Starting fast line detection on {len(self.voxels)} voxels')
        
        # Limit number of voxels for processing to speed up
        if len(self.voxels) > self.max_voxels_for_processing:
            # Sort by confidence and take the best voxels
            sorted_voxels = sorted(self.voxels, key=lambda v: v['confidence'], reverse=True)
            voxels_to_process = sorted_voxels[:self.max_voxels_for_processing]
            self.get_logger().info(f'Limited to {len(voxels_to_process)} highest confidence voxels')
        else:
            voxels_to_process = self.voxels
        
        # Extract 2D points for line detection (project onto XY plane)
        points_2d = np.array([[v['center'][0], v['center'][1]] for v in voxels_to_process])
        
        # Use simple line detection instead of RANSAC
        lines = self.detect_lines_simple(points_2d, voxels_to_process)
        
        self.get_logger().info(f'Total 50° ± 15° lines found: {len(lines)}')
        return lines

    def detect_lines_simple(self, points_2d, voxels):
        """Simple and fast line detection using angle-based clustering"""
        if len(points_2d) < self.min_line_points:
            return []
        
        lines = []
        
        # Group points by their direction from origin
        direction_groups = defaultdict(list)
        
        for i, point in enumerate(points_2d):
            # Calculate angle with X-axis
            angle_rad = math.atan2(point[1], point[0])
            angle_deg = math.degrees(angle_rad)
            
            # Check if angle is within 50° ± 15° range
            if self.min_angle_rad <= angle_rad <= self.max_angle_rad:
                # Group by rounded angle for clustering
                angle_group = round(angle_deg, 1)
                direction_groups[angle_group].append((point, voxels[i]))
        
        # Process each direction group
        for angle_group, point_voxel_pairs in direction_groups.items():
            if len(point_voxel_pairs) < self.min_line_points:
                continue
            
            # Extract points and voxels
            group_points = [pair[0] for pair in point_voxel_pairs]
            group_voxels = [pair[1] for pair in point_voxel_pairs]
            
            # Find the best line in this direction group
            line = self.find_best_line_in_group(group_points, group_voxels, angle_group)
            if line:
                lines.append(line)
        
        return lines

    def find_best_line_in_group(self, points, voxels, angle_group):
        """Find the best line within a group of points with similar directions"""
        if len(points) < self.min_line_points:
            return None
        
        # Convert to numpy array for faster processing
        points_array = np.array(points)
        
        # Calculate line properties
        line_length = self.calculate_line_length_fast(points_array)
        line_width = self.calculate_line_width_fast(points_array)
        
        # Check if line meets criteria
        if line_length >= self.min_beam_length and line_width <= self.beam_thickness_threshold:
            # Calculate line angle
            if len(points) >= 2:
                line_direction = points[1] - points[0]
                line_direction = line_direction / np.linalg.norm(line_direction)
                line_angle = math.degrees(math.atan2(line_direction[1], line_direction[0]))
            else:
                line_angle = angle_group
            
            # Calculate line endpoints (use average height of voxels)
            avg_height = sum(v['center'][2] for v in voxels) / len(voxels)
            start_point, end_point = self.calculate_line_endpoints_fast(points_array, avg_height)
            direction = self.calculate_line_direction_fast(points_array)
            
            self.get_logger().info(f'Found 50° ± 15° line: length={line_length:.2f}m, width={line_width:.2f}m, angle={line_angle:.1f}°, height={avg_height:.1f}m')
            
            return {
                'start': start_point,
                'end': end_point,
                'direction': direction,
                'length': line_length,
                'width': line_width,
                'height': avg_height,
                'voxels': voxels,
                'angle': line_angle
            }
        
        return None

    def calculate_line_length_fast(self, points):
        """Fast calculation of line length using bounding box"""
        if len(points) < 2:
            return 0.0
        
        # Use bounding box for faster calculation
        min_x, max_x = np.min(points[:, 0]), np.max(points[:, 0])
        min_y, max_y = np.min(points[:, 1]), np.max(points[:, 1])
        
        # Calculate diagonal length
        length = math.sqrt((max_x - min_x)**2 + (max_y - min_y)**2)
        return length

    def calculate_line_width_fast(self, points):
        """Fast calculation of line width using projection"""
        if len(points) < 2:
            return 0.0
        
        if len(points) == 2:
            return 0.0
        
        # Use first two points to define line direction
        direction = points[1] - points[0]
        direction = direction / np.linalg.norm(direction)
        
        # Calculate perpendicular distances efficiently
        vecs = points - points[0]
        projections = np.dot(vecs, direction)
        projected_points = points[0] + projections[:, np.newaxis] * direction
        perpendicular_distances = np.linalg.norm(points - projected_points, axis=1)
        
        return np.max(perpendicular_distances)

    def calculate_line_endpoints_fast(self, points, height):
        """Fast calculation of line endpoints using bounding box"""
        if len(points) < 2:
            return np.array([0, 0, height]), np.array([0, 0, height])
        
        # Find extreme points
        min_idx = np.argmin(points[:, 0])
        max_idx = np.argmax(points[:, 0])
        
        start_point = np.array([points[min_idx][0], points[min_idx][1], height])
        end_point = np.array([points[max_idx][0], points[max_idx][1], height])
        
        return start_point, end_point

    def calculate_line_direction_fast(self, points):
        """Fast calculation of line direction"""
        if len(points) < 2:
            return np.array([1, 0, 0])
        
        # Use first two points to define direction
        direction_2d = points[1] - points[0]
        direction_2d = direction_2d / np.linalg.norm(direction_2d)
        
        return np.array([direction_2d[0], direction_2d[1], 0])

    def publish_voxels(self):
        """Publish voxels as markers"""
        if not self.voxels:
            return
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "red_beam_voxels"
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.ADD
        
        # Set scale
        marker.scale.x = self.voxel_size
        marker.scale.y = self.voxel_size
        marker.scale.z = self.voxel_size
        
        # Add voxel centers
        for voxel in self.voxels:
            point = geometry_msgs.msg.Point()
            point.x = voxel['center'][0]
            point.y = voxel['center'][1]
            point.z = voxel['center'][2]
            marker.points.append(point)
            
            # Color based on confidence
            color = std_msgs.msg.ColorRGBA()
            color.r = voxel['confidence']
            color.g = 1.0 - voxel['confidence']
            color.b = 0.0
            color.a = 0.8
            marker.colors.append(color)
        
        self.voxel_publisher.publish(marker)

    def publish_beams(self, beams):
        """Publish beams as markers"""
        if not beams:
            self.get_logger().warn('No beams to publish')
            return
        
        marker_array = MarkerArray()
        
        for i, beam in enumerate(beams):
            # Create arrow marker for beam
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "red_beam_clusters"
            marker.id = i
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            
            # Set beam endpoints
            start_point = geometry_msgs.msg.Point()
            start_point.x = beam['start'][0]
            start_point.y = beam['start'][1]
            start_point.z = beam['start'][2]
            
            end_point = geometry_msgs.msg.Point()
            end_point.x = beam['end'][0]
            end_point.y = beam['end'][1]
            end_point.z = beam['end'][2]
            
            marker.points = [start_point, end_point]
            
            # Set scale
            marker.scale.x = 0.2  # Arrow width
            marker.scale.y = 0.2  # Arrow height
            marker.scale.z = 0.0  # Not used for arrows
            
            # Set color (different color for each beam)
            hue = (i * 137.5) % 360  # Golden angle for good color distribution
            r, g, b = colorsys.hsv_to_rgb(hue / 360.0, 1.0, 1.0)
            
            marker.color.r = r
            marker.color.g = g
            marker.color.b = b
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
        
        self.beam_publisher.publish(marker_array)
        self.get_logger().info(f'Published {len(beams)} beam markers')


def main(args=None):
    rclpy.init(args=args)
    node = RedBeamLineDetectorFast()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main() 