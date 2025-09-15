#!/usr/bin/env python3
"""
Red Beam Clusterer - Fixed Version
Clusters red beam voxels into unique beams parallel to XY plane.
Correctly handles the RGB data format.
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


class RedBeamClustererFixed(Node):
    def __init__(self):
        super().__init__('red_beam_clusterer_fixed')
        
        # Data storage
        self.voxels = []
        self.voxel_size = 0.2
        self.point_cloud_data = None
        
        # Parameters - Corrected for red beams
        self.confidence_threshold = 0.1
        self.min_points_per_voxel = 3
        self.min_beam_length = 5.0  # Reasonable minimum length for a beam
        self.beam_thickness_threshold = 0.3  # Very thin - just 1-2 voxels wide
        self.cluster_distance = 1.0
        
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
        
        self.get_logger().info('Red Beam Clusterer Fixed initialized')
        self.get_logger().info(f'Input topic: /rayfronts/queries/red_beam')
        self.get_logger().info(f'Voxel output topic: /red_beam_voxels')
        self.get_logger().info(f'Beam output topic: /red_beam_clusters')
        self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')
        self.get_logger().info(f'Voxel size: {self.voxel_size}')
        self.get_logger().info(f'Min points per voxel: {self.min_points_per_voxel}')
        self.get_logger().info(f'Min beam length: {self.min_beam_length}m')
        self.get_logger().info(f'Beam thickness threshold: {self.beam_thickness_threshold}m')
        self.get_logger().info(f'Cluster distance: {self.cluster_distance}m')

    def point_cloud_callback(self, msg):
        self.get_logger().info(f'Received point cloud with {msg.width} points')
        
        # Process point cloud
        self.process_point_cloud(msg)
        
        # Cluster voxels into beams
        beams = self.cluster_voxels_into_beams()
        
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
            
            # Extract RGB values - the issue is that rgb is a very small float
            # We need to interpret it as a packed 32-bit integer
            try:
                # Convert the float to its binary representation and then to int
                # The float represents a packed RGB value
                rgb_bytes = struct.pack('f', float(rgb))
                rgb_int = struct.unpack('I', rgb_bytes)[0]  # Unpack as unsigned int
                
                # Now extract R, G, B from the packed integer
                r = (rgb_int >> 16) & 0xFF
                g = (rgb_int >> 8) & 0xFF
                b = rgb_int & 0xFF
                
            except Exception as e:
                # Fallback: try direct conversion
                try:
                    rgb_int = int(rgb)
                    r = (rgb_int >> 16) & 0xFF
                    g = (rgb_int >> 8) & 0xFF
                    b = rgb_int & 0xFF
                except:
                    r, g, b = 0, 0, 0
            
            # Calculate confidence (closer to yellow = higher confidence)
            # Yellow is (255, 255, 0), so we want high R and G, low B
            confidence = (r + g) / (2 * 255.0) - (b / 255.0)
            confidence = max(0.0, min(1.0, confidence))  # Clamp to [0,1]
            
            confidence_values.append(confidence)
            
            if i < 5:  # Debug first 5 points
                self.get_logger().info(f'Point {i}: RGB={rgb:.2e} -> int={rgb_int} -> r={r}, g={g}, b={b} -> confidence={confidence:.4f}')
            
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

    def cluster_voxels_into_beams(self):
        """Cluster voxels into beams parallel to XY plane"""
        if not self.voxels:
            return []
        
        self.get_logger().info(f'Starting clustering of {len(self.voxels)} voxels')
        
        # Group voxels by Z coordinate (height)
        height_groups = defaultdict(list)
        for voxel in self.voxels:
            z = voxel['center'][2]
            height_groups[round(z, 1)].append(voxel)  # Round to 0.1m precision
        
        self.get_logger().info(f'Found {len(height_groups)} height groups')
        
        beams = []
        beam_id = 0
        
        for height, voxels_at_height in height_groups.items():
            self.get_logger().info(f'Processing height {height}m with {len(voxels_at_height)} voxels')
            
            # Cluster voxels at this height
            clusters = self.cluster_voxels_at_height(voxels_at_height)
            
            for cluster in clusters:
                if len(cluster) >= 2:  # Need at least 2 voxels for a beam
                    # Check if cluster forms a valid beam
                    beam = self.validate_beam(cluster, height)
                    if beam:
                        beam['id'] = beam_id
                        beam['height'] = height
                        beams.append(beam)
                        beam_id += 1
                        self.get_logger().info(f'Found valid beam {beam_id-1} at height {height}m with {len(cluster)} voxels')
        
        self.get_logger().info(f'Total beams found: {len(beams)}')
        return beams

    def cluster_voxels_at_height(self, voxels):
        """Cluster voxels at the same height using simple distance-based clustering"""
        if len(voxels) < 2:
            return [voxels] if voxels else []
        
        clusters = []
        used = set()
        
        for i, voxel in enumerate(voxels):
            if i in used:
                continue
                
            cluster = [voxel]
            used.add(i)
            
            # Find nearby voxels
            for j, other_voxel in enumerate(voxels):
                if j in used:
                    continue
                    
                # Calculate distance
                dx = voxel['center'][0] - other_voxel['center'][0]
                dy = voxel['center'][1] - other_voxel['center'][1]
                distance = np.sqrt(dx*dx + dy*dy)
                
                if distance <= self.cluster_distance:
                    cluster.append(other_voxel)
                    used.add(j)
            
            clusters.append(cluster)
        
        return clusters

    def validate_beam(self, cluster, height):
        """Validate if a cluster forms a valid beam"""
        if len(cluster) < 2:
            return None
        
        # Calculate beam properties
        x_coords = [v['center'][0] for v in cluster]
        y_coords = [v['center'][1] for v in cluster]
        
        min_x, max_x = min(x_coords), max(x_coords)
        min_y, max_y = min(y_coords), max(y_coords)
        
        # Calculate dimensions
        x_span = max_x - min_x
        y_span = max_y - min_y
        
        # Determine if beam is along X-axis or Y-axis
        if x_span > y_span:
            # Beam is along X-axis
            beam_length = x_span
            beam_width = y_span
            direction = np.array([1, 0, 0])
            start_point = np.array([min_x, (min_y + max_y) / 2, height])
            end_point = np.array([max_x, (min_y + max_y) / 2, height])
        else:
            # Beam is along Y-axis
            beam_length = y_span
            beam_width = x_span
            direction = np.array([0, 1, 0])
            start_point = np.array([(min_x + max_x) / 2, min_y, height])
            end_point = np.array([(min_x + max_x) / 2, max_y, height])
        
        self.get_logger().info(f'Beam candidate: length={beam_length:.2f}m, width={beam_width:.2f}m, height={height}m')
        
        # Check if beam meets criteria:
        # - Length should be long (>= min_beam_length)
        # - Width should be very thin (<= beam_thickness_threshold)
        if beam_length >= self.min_beam_length and beam_width <= self.beam_thickness_threshold:
            return {
                'start': start_point,
                'end': end_point,
                'direction': direction,
                'length': beam_length,
                'width': beam_width,
                'height': height,
                'voxels': cluster
            }
        
        return None

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
    node = RedBeamClustererFixed()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main() 