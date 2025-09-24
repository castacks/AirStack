#!/usr/bin/env python3
"""
Red Beam Clusterer
Clusters red beam voxels into unique beams parallel to XY plane.
Publishes each beam as a distinct colored marker.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
import sensor_msgs_py.point_cloud2 as pc2
from collections import defaultdict
import struct
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
import colorsys


class RedBeamClusterer(Node):
    def __init__(self):
        super().__init__('red_beam_clusterer')
        
        # Data storage
        self.voxel_data = defaultdict(list)  # voxel_key -> list of points
        self.voxel_size = 0.2
        self.confidence_threshold = 0.1
        self.min_points_per_voxel = 3
        self.min_beam_length = 10.0  # meters
        self.beam_thickness_threshold = 0.5  # meters
        
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
        
        self.get_logger().info('Red Beam Clusterer initialized')
        self.get_logger().info(f'Input topic: /rayfronts/queries/red_beam')
        self.get_logger().info(f'Voxel output topic: /red_beam_voxels')
        self.get_logger().info(f'Beam output topic: /red_beam_clusters')
        self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')
        self.get_logger().info(f'Voxel size: {self.voxel_size}')
        self.get_logger().info(f'Min points per voxel: {self.min_points_per_voxel}')
        self.get_logger().info(f'Min beam length: {self.min_beam_length}m')
        self.get_logger().info(f'Beam thickness threshold: {self.beam_thickness_threshold}m')

    def extract_rgb_from_packed(self, rgb_packed):
        """Extract R, G, B values from packed RGB format (32-bit integer)"""
        rgb_int = struct.unpack('<I', rgb_packed)[0]
        r = (rgb_int >> 16) & 0xFF
        g = (rgb_int >> 8) & 0xFF
        b = rgb_int & 0xFF
        return r, g, b

    def calculate_confidence(self, r, g, b):
        """Calculate confidence based on RGB values - closer to yellow = higher confidence"""
        r_norm = r / 255.0
        g_norm = g / 255.0
        b_norm = b / 255.0
        
        yellow_distance = np.sqrt((1.0 - r_norm)**2 + (1.0 - g_norm)**2 + (b_norm)**2)
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
        
        for point in pc2.read_points(msg, field_names=("x", "y", "z", "rgb"), skip_nans=True):
            x, y, z, rgb_packed = point
            
            # Extract RGB values from packed format
            r, g, b = self.extract_rgb_from_packed(rgb_packed)
            
            # Calculate confidence
            confidence = self.calculate_confidence(r, g, b)
            
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
        
        self.get_logger().info(f'Processed {points_processed} points, {points_above_threshold} above threshold')
        
        # Generate voxel markers
        self.publish_voxel_markers()
        
        # Cluster voxels into beams
        self.cluster_voxels_into_beams()

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
        
        self.get_logger().info(f'Found {valid_voxels} valid voxels')
        
        # Publish marker
        self.voxel_publisher.publish(marker)

    def cluster_voxels_into_beams(self):
        """Cluster valid voxels into beams parallel to XY plane"""
        # Collect valid voxel centers
        valid_voxels = []
        voxel_centers = []
        
        for voxel_key, points in self.voxel_data.items():
            if len(points) >= self.min_points_per_voxel:
                voxel_x, voxel_y, voxel_z = voxel_key
                center_x = (voxel_x + 0.5) * self.voxel_size
                center_y = (voxel_y + 0.5) * self.voxel_size
                center_z = (voxel_z + 0.5) * self.voxel_size
                
                valid_voxels.append(voxel_key)
                voxel_centers.append([center_x, center_y, center_z])
        
        if len(valid_voxels) < 2:
            self.get_logger().warn('Not enough valid voxels for clustering')
            return
        
        # Convert to numpy array
        voxel_centers = np.array(voxel_centers)
        
        # Cluster voxels using DBSCAN
        # Use a larger eps for 3D clustering
        eps = self.voxel_size * 2.0  # Allow some gap between voxels
        min_samples = 3
        
        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(voxel_centers)
        labels = clustering.labels_
        
        # Group voxels by cluster
        clusters = defaultdict(list)
        for i, label in enumerate(labels):
            if label != -1:  # Ignore noise points
                clusters[label].append((valid_voxels[i], voxel_centers[i]))
        
        self.get_logger().info(f'Found {len(clusters)} clusters')
        
        # Process each cluster to identify beams
        beam_markers = []
        beam_id = 0
        
        for cluster_id, cluster_voxels in clusters.items():
            if len(cluster_voxels) < 3:  # Need at least 3 voxels for a beam
                continue
            
            # Extract points for this cluster
            cluster_points = np.array([voxel[1] for voxel in cluster_voxels])
            
            # Check if cluster forms a beam parallel to XY plane
            beam_info = self.analyze_beam_cluster(cluster_points)
            
            if beam_info is not None:
                length, direction, start_point, end_point, thickness = beam_info
                
                if length >= self.min_beam_length and thickness <= self.beam_thickness_threshold:
                    # Create beam marker
                    beam_marker = self.create_beam_marker(beam_id, start_point, end_point, thickness)
                    beam_markers.append(beam_marker)
                    
                    self.get_logger().info(f'Beam {beam_id}: length={length:.2f}m, thickness={thickness:.2f}m, '
                                         f'direction={direction}, points={len(cluster_voxels)}')
                    beam_id += 1
        
        # Publish beam markers
        if beam_markers:
            marker_array = MarkerArray()
            marker_array.markers = beam_markers
            self.beam_publisher.publish(marker_array)
            self.get_logger().info(f'Published {len(beam_markers)} beam markers')
        else:
            self.get_logger().warn('No valid beams found')

    def analyze_beam_cluster(self, points):
        """Analyze a cluster of points to determine if it forms a beam"""
        if len(points) < 3:
            return None
        
        # Project points onto XY plane
        xy_points = points[:, :2]
        
        # Fit a line to the XY projection using PCA
        from sklearn.decomposition import PCA
        pca = PCA(n_components=2)
        pca.fit(xy_points)
        
        # Get the principal direction (first component)
        direction = pca.components_[0]
        
        # Project all points onto the principal axis
        projected_points = np.dot(xy_points, direction)
        
        # Calculate beam length
        beam_length = np.max(projected_points) - np.min(projected_points)
        
        # Calculate beam thickness (perpendicular to principal axis)
        perpendicular_direction = pca.components_[1]
        perpendicular_projections = np.dot(xy_points, perpendicular_direction)
        beam_thickness = np.max(perpendicular_projections) - np.min(perpendicular_projections)
        
        # Check if beam is roughly parallel to XY plane
        # Calculate the angle between the principal direction and XY plane
        z_variance = np.var(points[:, 2])
        xy_variance = np.var(xy_points.flatten())
        
        # If Z variance is much smaller than XY variance, beam is parallel to XY plane
        if z_variance > xy_variance * 0.1:  # Allow some Z variation
            return None
        
        # Calculate start and end points
        min_proj_idx = np.argmin(projected_points)
        max_proj_idx = np.argmax(projected_points)
        
        start_point = points[min_proj_idx]
        end_point = points[max_proj_idx]
        
        # Calculate average Z coordinate for the beam
        avg_z = np.mean(points[:, 2])
        start_point[2] = avg_z
        end_point[2] = avg_z
        
        return beam_length, direction, start_point, end_point, beam_thickness

    def create_beam_marker(self, beam_id, start_point, end_point, thickness):
        """Create a marker for a beam"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "red_beam_clusters"
        marker.id = beam_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        
        # Calculate beam center and orientation
        center = (start_point + end_point) / 2.0
        direction = end_point - start_point
        length = np.linalg.norm(direction)
        
        # Set position
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]
        
        # Set orientation (align cylinder with beam direction)
        # Calculate rotation from Z-axis to beam direction
        z_axis = np.array([0, 0, 1])
        beam_direction = direction / length
        
        # Calculate rotation axis and angle
        rotation_axis = np.cross(z_axis, beam_direction)
        rotation_angle = np.arccos(np.clip(np.dot(z_axis, beam_direction), -1.0, 1.0))
        
        if np.linalg.norm(rotation_axis) > 1e-6:
            rotation_axis = rotation_axis / np.linalg.norm(rotation_axis)
            # Convert to quaternion
            qw = np.cos(rotation_angle / 2.0)
            qx = rotation_axis[0] * np.sin(rotation_angle / 2.0)
            qy = rotation_axis[1] * np.sin(rotation_angle / 2.0)
            qz = rotation_axis[2] * np.sin(rotation_angle / 2.0)
        else:
            qw, qx, qy, qz = 1.0, 0.0, 0.0, 0.0
        
        marker.pose.orientation.w = qw
        marker.pose.orientation.x = qx
        marker.pose.orientation.y = qy
        marker.pose.orientation.z = qz
        
        # Set scale (length, diameter, diameter)
        marker.scale.x = length
        marker.scale.y = max(thickness, 0.1)  # Minimum thickness
        marker.scale.z = max(thickness, 0.1)  # Minimum thickness
        
        # Set color (distinct color for each beam)
        hue = (beam_id * 0.618) % 1.0  # Golden ratio for good color distribution
        rgb = colorsys.hsv_to_rgb(hue, 0.8, 0.9)
        
        marker.color.r = rgb[0]
        marker.color.g = rgb[1]
        marker.color.b = rgb[2]
        marker.color.a = 0.7
        
        return marker


def main(args=None):
    rclpy.init(args=args)
    node = RedBeamClusterer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 