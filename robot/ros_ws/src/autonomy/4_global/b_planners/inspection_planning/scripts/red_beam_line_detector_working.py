#!/usr/bin/env python3
"""
Red Beam Line Detector
Uses line detection algorithms to find red beams at each height level.
Only detects lines along 30° ± 20° with respect to X-axis.
Maintains a list of detected lines and removes duplicates, keeping longer lines.
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


class RedBeamLineDetector(Node):
    def __init__(self):
        super().__init__('red_beam_line_detector')
        
        # Data storage
        self.voxels = []
        self.voxel_size = 0.2
        self.point_cloud_data = None
        
        # Maintained list of detected lines
        self.detected_lines = []
        self.line_similarity_threshold = 1.0  # Distance threshold for considering lines similar
        self.line_angle_threshold = 10.0  # Angle difference threshold in degrees
        
        # Parameters
        self.confidence_threshold = 0.3
        self.min_points_per_voxel = 3
        self.min_beam_length = 5.0  # Minimum length for a valid beam
        self.beam_thickness_threshold = 0.3  # Maximum width for a valid beam
        self.cluster_distance = 1.0
        
        # Line direction constraints - 30° ± 20° with respect to X-axis
        self.target_direction_angle = 25.0  # 30 degrees from X-axis
        self.angle_tolerance = 15.0  # ±20 degrees tolerance
        self.min_angle_rad = math.radians(self.target_direction_angle - self.angle_tolerance)  # 10°
        self.max_angle_rad = math.radians(self.target_direction_angle + self.angle_tolerance)  # 50°
        
        # Line detection parameters
        self.ransac_threshold = 0.1  # Distance threshold for RANSAC
        self.min_line_points = 25  # Minimum points to form a line
        self.max_iterations = 200  # RANSAC iterations
        
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
        
        self.get_logger().info('Red Beam Line Detector initialized')
        self.get_logger().info(f'Input topic: /rayfronts/queries/red_beam')
        self.get_logger().info(f'Voxel output topic: /red_beam_voxels')
        self.get_logger().info(f'Beam output topic: /red_beam_clusters')
        self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')
        self.get_logger().info(f'Voxel size: {self.voxel_size}')
        self.get_logger().info(f'Min points per voxel: {self.min_points_per_voxel}')
        self.get_logger().info(f'Min beam length: {self.min_beam_length}m')
        self.get_logger().info(f'Beam thickness threshold: {self.beam_thickness_threshold}m')
        self.get_logger().info(f'Target direction: {self.target_direction_angle}° ± {self.angle_tolerance}° from X-axis')
        self.get_logger().info(f'Angle range: {self.target_direction_angle - self.angle_tolerance}° to {self.target_direction_angle + self.angle_tolerance}°')
        self.get_logger().info(f'Line similarity threshold: {self.line_similarity_threshold}m')
        self.get_logger().info(f'Line angle threshold: {self.line_angle_threshold}°')
        self.get_logger().info(f'RANSAC threshold: {self.ransac_threshold}m')
        self.get_logger().info(f'Min line points: {self.min_line_points}')

    def point_cloud_callback(self, msg):
        self.get_logger().info(f'Received point cloud with {msg.width} points')
        
        # Process point cloud
        self.process_point_cloud(msg)
        
        # Detect lines at each height
        new_beams = self.detect_lines_at_heights()
        
        # Update maintained list of lines
        self.update_detected_lines(new_beams)
        
        # Publish results
        self.publish_voxels()
        self.publish_beams(self.detected_lines)

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

    def detect_lines_at_heights(self):
        """Detect lines at each height level using RANSAC with 30° ± 20° constraint"""
        if not self.voxels:
            return []
        
        self.get_logger().info(f'Starting line detection on {len(self.voxels)} voxels')
        
        # Group voxels by Z coordinate (height)
        height_groups = defaultdict(list)
        for voxel in self.voxels:
            z = voxel['center'][2]
            height_groups[round(z, 1)].append(voxel)
        
        self.get_logger().info(f'Found {len(height_groups)} height groups')
        
        all_beams = []
        beam_id = 0
        
        for height, voxels_at_height in height_groups.items():
            self.get_logger().info(f'Processing height {height}m with {len(voxels_at_height)} voxels')
            
            if len(voxels_at_height) < self.min_line_points:
                continue
            
            # Extract 2D points for line detection
            points_2d = np.array([[v['center'][0], v['center'][1]] for v in voxels_at_height])
            
            # Detect lines using RANSAC with 30° ± 20° constraint
            lines = self.detect_lines_ransac_30deg(points_2d, voxels_at_height, height)
            
            for line in lines:
                line['id'] = beam_id
                line['height'] = height
                all_beams.append(line)
                beam_id += 1
                self.get_logger().info(f'Found 30° ± 20° line {beam_id-1} at height {height}m: length={line["length"]:.2f}m, width={line["width"]:.2f}m, angle={line["angle"]:.1f}°')
        
        self.get_logger().info(f'Total 30° ± 20° lines found: {len(all_beams)}')
        return all_beams

    def detect_lines_ransac_30deg(self, points_2d, voxels, height):
        """Detect lines using RANSAC algorithm, constrained to 30° ± 20° direction"""
        if len(points_2d) < self.min_line_points:
            return []
        
        lines = []
        remaining_points = points_2d.copy()
        remaining_voxels = voxels.copy()
        
        while len(remaining_points) >= self.min_line_points:
            best_line = None
            best_inliers = []
            best_inlier_voxels = []
            best_score = 0
            
            # RANSAC iterations
            for _ in range(self.max_iterations):
                # Randomly select 2 points
                if len(remaining_points) < 2:
                    break
                    
                indices = np.random.choice(len(remaining_points), 2, replace=False)
                p1, p2 = remaining_points[indices[0]], remaining_points[indices[1]]
                
                # Skip if points are too close
                if np.linalg.norm(p1 - p2) < 0.1:
                    continue
                
                # Calculate line direction and check if it's within 30° ± 20° tolerance
                line_direction = p2 - p1
                line_direction = line_direction / np.linalg.norm(line_direction)
                
                # Calculate angle with X-axis
                angle_rad = math.atan2(line_direction[1], line_direction[0])
                
                # Check if angle is within 30° ± 20° tolerance (10° to 50°)
                if not (self.min_angle_rad <= angle_rad <= self.max_angle_rad):
                    continue
                
                # Fit line through these points
                line_params = self.fit_line_2d(p1, p2)
                if line_params is None:
                    continue
                
                # Find inliers
                inliers, inlier_voxels = self.find_line_inliers(
                    remaining_points, remaining_voxels, line_params
                )
                
                # Score based on number of inliers and line length
                if len(inliers) >= self.min_line_points:
                    line_length = self.calculate_line_length(inliers)
                    score = len(inliers) * line_length  # Favor longer lines with more points
                    
                    if score > best_score:
                        best_score = score
                        best_line = line_params
                        best_inliers = inliers
                        best_inlier_voxels = inlier_voxels
            
            if best_line is not None and len(best_inliers) >= self.min_line_points:
                # Calculate line properties
                line_length = self.calculate_line_length(best_inliers)
                line_width = self.calculate_line_width(best_inliers)
                
                # Calculate line angle for validation
                line_direction = best_inliers[1] - best_inliers[0] if len(best_inliers) >= 2 else np.array([1, 0])
                line_direction = line_direction / np.linalg.norm(line_direction)
                line_angle = math.degrees(math.atan2(line_direction[1], line_direction[0]))
                
                # Check if line meets criteria
                if line_length >= self.min_beam_length and line_width <= self.beam_thickness_threshold:
                    # Calculate line endpoints
                    start_point, end_point = self.calculate_line_endpoints(best_inliers, height)
                    direction = self.calculate_line_direction(best_inliers)
                    
                    lines.append({
                        'start': start_point,
                        'end': end_point,
                        'direction': direction,
                        'length': line_length,
                        'width': line_width,
                        'height': height,
                        'voxels': best_inlier_voxels,
                        'line_params': best_line,
                        'angle': line_angle
                    })
                
                # Remove inliers from remaining points
                inlier_indices = []
                for i, point in enumerate(remaining_points):
                    if any(np.allclose(point, inlier) for inlier in best_inliers):
                        inlier_indices.append(i)
                
                remaining_points = np.delete(remaining_points, inlier_indices, axis=0)
                remaining_voxels = [v for i, v in enumerate(remaining_voxels) if i not in inlier_indices]
            else:
                break
        
        return lines

    def update_detected_lines(self, new_lines):
        """Update the maintained list of detected lines, removing duplicates and keeping longer lines"""
        if not new_lines:
            return
        
        self.get_logger().info(f'Processing {len(new_lines)} new lines against {len(self.detected_lines)} existing lines')
        
        for new_line in new_lines:
            # Check if this new line is similar to any existing line
            similar_lines = []
            for i, existing_line in enumerate(self.detected_lines):
                if self.are_lines_similar(new_line, existing_line):
                    similar_lines.append((i, existing_line))
            
            if similar_lines:
                # Found similar lines - keep the longest one
                all_lines = [new_line] + [line for _, line in similar_lines]
                longest_line = max(all_lines, key=lambda x: x['length'])
                
                # Remove all similar lines from detected_lines
                for i, _ in reversed(similar_lines):  # Reverse to maintain indices
                    self.detected_lines.pop(i)
                
                # Add the longest line
                self.detected_lines.append(longest_line)
                
                self.get_logger().info(f'Replaced {len(similar_lines)} similar lines with longer line (length: {longest_line["length"]:.2f}m)')
            else:
                # No similar lines found - add the new line
                self.detected_lines.append(new_line)
                self.get_logger().info(f'Added new line (length: {new_line["length"]:.2f}m)')
        
        self.get_logger().info(f'Total maintained lines: {len(self.detected_lines)}')

    def are_lines_similar(self, line1, line2):
        """Check if two lines are similar (close and parallel)"""
        # Calculate distance between line segments
        distance = self.calculate_line_segment_distance(
            line1['start'], line1['end'],
            line2['start'], line2['end']
        )
        
        # Check if lines are close enough
        if distance > self.cluster_distance:
            return False
        
        # Check if lines are roughly parallel
        dir1 = np.array(line1['direction'])
        dir2 = np.array(line2['direction'])
        
        # Calculate angle between directions
        dot_product = np.dot(dir1, dir2)
        dot_product = np.clip(dot_product, -1.0, 1.0)  # Clamp to avoid numerical errors
        angle = math.degrees(math.acos(abs(dot_product)))
        
        # Lines are similar if they're close and roughly parallel (within 30 degrees)
        return angle < 30.0

    def calculate_line_segment_distance(self, start1, end1, start2, end2):
        """Calculate minimum distance between two line segments by sampling points along both lines"""
        # Convert to numpy arrays
        p1 = np.array(start1)
        q1 = np.array(end1)
        p2 = np.array(start2)
        q2 = np.array(end2)
        
        # Calculate line directions
        d1 = q1 - p1
        d2 = q2 - p2
        
        # Calculate line lengths
        len1 = np.linalg.norm(d1)
        len2 = np.linalg.norm(d2)
        
        if len1 < 1e-6 or len2 < 1e-6:
            # One or both lines are degenerate points
            return min(
                np.linalg.norm(p1 - p2),
                np.linalg.norm(p1 - q2),
                np.linalg.norm(q1 - p2),
                np.linalg.norm(q1 - q2)
            )
        
        # Normalize directions
        d1 = d1 / len1
        d2 = d2 / len2
        
        # Sample points along both line segments
        num_samples = max(5, int(max(len1, len2) / self.voxel_size))
        min_distance = float('inf')
        
        for i in range(num_samples + 1):
            # Parameter along first line
            t1 = i / num_samples
            point1 = p1 + t1 * d1 * len1
            
            for j in range(num_samples + 1):
                # Parameter along second line
                t2 = j / num_samples
                point2 = p2 + t2 * d2 * len2
                
                # Calculate distance between these two points
                distance = np.linalg.norm(point1 - point2)
                min_distance = min(min_distance, distance)
        
        return min_distance

    def fit_line_2d(self, p1, p2):
        """Fit a line through two 2D points"""
        if np.allclose(p1, p2):
            return None
        
        # Line equation: ax + by + c = 0
        # Direction vector: (p2 - p1)
        direction = p2 - p1
        direction = direction / np.linalg.norm(direction)
        
        # Normal vector (perpendicular to direction)
        normal = np.array([-direction[1], direction[0]])
        
        # Line equation: normal[0]*x + normal[1]*y + c = 0
        # Using point p1: c = -normal[0]*p1[0] - normal[1]*p1[1]
        c = -np.dot(normal, p1)
        
        return {'normal': normal, 'c': c, 'direction': direction}

    def find_line_inliers(self, points, voxels, line_params):
        """Find points that are close to the line"""
        inliers = []
        inlier_voxels = []
        
        for i, point in enumerate(points):
            # Distance from point to line
            distance = abs(np.dot(line_params['normal'], point) + line_params['c'])
            
            if distance <= self.ransac_threshold:
                inliers.append(point)
                inlier_voxels.append(voxels[i])
        
        return inliers, inlier_voxels

    def calculate_line_length(self, points):
        """Calculate the length of a line segment"""
        if len(points) < 2:
            return 0.0
        
        # Find the two points that are farthest apart
        max_distance = 0.0
        for i in range(len(points)):
            for j in range(i + 1, len(points)):
                distance = np.linalg.norm(points[i] - points[j])
                max_distance = max(max_distance, distance)
        
        return max_distance

    def calculate_line_width(self, points):
        """Calculate the width (perpendicular spread) of a line"""
        if len(points) < 2:
            return 0.0
        
        # Project points onto the line direction
        if len(points) == 2:
            return 0.0
        
        # Use first two points to define line direction
        direction = points[1] - points[0]
        direction = direction / np.linalg.norm(direction)
        
        # Calculate perpendicular distances
        perpendicular_distances = []
        for point in points:
            # Vector from first point to current point
            vec = point - points[0]
            # Project onto perpendicular direction
            perp_distance = np.linalg.norm(vec - np.dot(vec, direction) * direction)
            perpendicular_distances.append(perp_distance)
        
        return max(perpendicular_distances)

    def calculate_line_endpoints(self, points, height):
        """Calculate the endpoints of a line segment"""
        if len(points) < 2:
            return np.array([0, 0, height]), np.array([0, 0, height])
        
        # Find the two points that are farthest apart
        max_distance = 0.0
        best_pair = (0, 1)
        
        for i in range(len(points)):
            for j in range(i + 1, len(points)):
                distance = np.linalg.norm(points[i] - points[j])
                if distance > max_distance:
                    max_distance = distance
                    best_pair = (i, j)
        
        start_point = np.array([points[best_pair[0]][0], points[best_pair[0]][1], height])
        end_point = np.array([points[best_pair[1]][0], points[best_pair[1]][1], height])
        
        return start_point, end_point

    def calculate_line_direction(self, points):
        """Calculate the direction vector of a line"""
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
    node = RedBeamLineDetector()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main() 