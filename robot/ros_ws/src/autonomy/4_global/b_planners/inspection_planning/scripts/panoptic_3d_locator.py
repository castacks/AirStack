#!/usr/bin/env python3
"""
Panoptic 3D Locator for RMF Owl
Processes panoptic segmentation + depth to get unique 3D object instances
Based on Gazebo Fortress panoptic segmentation documentation
"""

import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from geometry_msgs.msg import Pose, PoseArray, Point
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration
import struct
import geometry_msgs.msg

class PanopticLocator(Node):
    def __init__(self):
        super().__init__('panoptic_locator')
        self.bridge = CvBridge()

        # Publishers
        self.pointcloud_pub = self.create_publisher(PointCloud2, '/panoptic/pointcloud', 10)
        self.markers_pub = self.create_publisher(MarkerArray, '/panoptic/markers', 10)
        self.poses_pub = self.create_publisher(PoseArray, '/panoptic/poses', 10)
        
        # Subscribers
        self.labels_sub = self.create_subscription(Image, '/panoptic/labels_map', self.labels_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/depth_camera_front', self.depth_callback, 10)
        self.depth_info_sub = self.create_subscription(CameraInfo, '/panoptic/camera_info1', self.depth_info_callback, 10)

        # Camera parameters (default values for 96x54)
        self.fx = 50.60
        self.fy = 50.60
        self.cx = 48.0
        self.cy = 27.0
        self.frame_id = 'base_link'

        # Store latest data
        self.latest_labels = None
        self.latest_depth = None
        
        # Semantic class names
        self.class_names = {
            0: 'background',
            1: 'ground', 
            10: 'red_object',
            20: 'blue_object',
            30: 'green_object',
            40: 'yellow_object'
        }
        
        # Base colors for each semantic class
        self.base_colors = {
            0: [0.5, 0.5, 0.5],    # gray
            1: [0.6, 0.4, 0.2],    # brown  
            10: [1.0, 0.0, 0.0],   # red
            20: [0.0, 0.0, 1.0],   # blue
            30: [0.0, 1.0, 0.0],   # green
            40: [1.0, 1.0, 0.0],   # yellow
        }

        self.get_logger().info('Panoptic 3D Locator started - tracking unique object instances')
        
        # Process timer
        self.timer = self.create_timer(0.5, self.process_data)

    def depth_info_callback(self, msg):
        """Get camera intrinsics"""
        k = msg.k
        self.fx, self.fy = k[0], k[4]
        self.cx, self.cy = k[2], k[5]
        # self.get_logger().info(f'Camera intrinsics: fx={self.fx:.2f}, fy={self.fy:.2f}, cx={self.cx:.2f}, cy={self.cy:.2f}')

    def labels_callback(self, msg):
        """Store latest panoptic labels data"""
        try:
            # For panoptic segmentation, labels_map is BGR8 format where:
            # - Blue channel (index 0): instance count low byte  
            # - Green channel (index 1): instance count high byte
            # - Red channel (index 2): semantic label
            self.frame_id=msg.header.frame_id
            labels_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_labels = labels_img
        except Exception as e:
            self.get_logger().error(f'Error processing panoptic labels: {e}')

    def depth_callback(self, msg):
        """Store latest depth data"""
        try:
            depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Clean depth data
            depth_clean = depth_img.astype(np.float32)
            depth_clean[np.isinf(depth_clean)] = 0.0
            depth_clean[np.isnan(depth_clean)] = 0.0
            depth_clean[depth_clean < 0.0] = 0.0
            depth_clean[depth_clean > 50.0] = 0.0
            
            self.latest_depth = depth_clean
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth: {e}')

    def extract_panoptic_info(self, labels_img):
        """
        Extract semantic labels and instance IDs from panoptic labels_map
        Based on Gazebo documentation: https://gazebosim.org/api/sensors/8/segmentationcamera_igngazebo.html
        """
        # Extract channels
        instance_low = labels_img[:, :, 0].astype(np.uint16)   # Blue channel
        instance_high = labels_img[:, :, 1].astype(np.uint16)  # Green channel  
        semantic_labels = labels_img[:, :, 2]                   # Red channel
        
        # Combine instance count from two bytes: high_byte * 256 + low_byte
        instance_ids = instance_high * 256 + instance_low
        
        return semantic_labels, instance_ids

    def get_instance_color(self, semantic_label, instance_id):
        """Generate unique color for each instance based on semantic class + instance ID"""
        base_color = self.base_colors.get(semantic_label, [0.5, 0.5, 0.5])
        
        # Vary the color slightly based on instance ID to make each instance unique
        hue_offset = (instance_id * 0.618033988749895) % 1.0  # Golden ratio for good distribution
        
        # Convert to HSV, modify hue, convert back to RGB
        import colorsys
        h, s, v = colorsys.rgb_to_hsv(base_color[0], base_color[1], base_color[2])
        h = (h + hue_offset * 0.3) % 1.0  # Vary hue by up to 30%
        s = min(1.0, s + 0.2)  # Increase saturation slightly
        
        return colorsys.hsv_to_rgb(h, s, v)

    def simple_pca(self, points):
        """
        Simple PCA implementation using NumPy to find the principal axis
        Returns the first principal component (direction of maximum variance)
        """
        if len(points) < 2:
            return np.array([1.0, 0.0, 0.0])  # Default axis
        
        # Center the data
        centered_points = points - np.mean(points, axis=0)
        
        # Compute covariance matrix
        cov_matrix = np.cov(centered_points.T)
        
        # Find eigenvalues and eigenvectors
        eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)
        
        # Return the eigenvector corresponding to the largest eigenvalue
        principal_axis = eigenvectors[:, np.argmax(eigenvalues)]
        
        return principal_axis

    def process_data(self):
        """Process panoptic labels and depth data together"""
        if self.latest_labels is None or self.latest_depth is None:
            self.get_logger().info('Waiting for both panoptic labels and depth data...')
            return
            
        try:
            labels_img = self.latest_labels
            depth = self.latest_depth
            
            # Ensure same dimensions
            if labels_img.shape[:2] != depth.shape:
                self.get_logger().warn(f'Size mismatch: labels {labels_img.shape} vs depth {depth.shape}')
                return

            # Extract panoptic information
            semantic_labels, instance_ids = self.extract_panoptic_info(labels_img)
            
            H, W = depth.shape
            
            # Create coordinate grids
            u_coords, v_coords = np.meshgrid(np.arange(W), np.arange(H))
            
            # Valid depth mask (exclude background label=0)
            valid_mask = (depth > 0.1) & (depth < 10.0) & (semantic_labels > 0)
            
            if not np.any(valid_mask):
                self.get_logger().info('No valid panoptic+depth data')
                return
                
            # Get valid pixels
            valid_u = u_coords[valid_mask].astype(np.float32)
            valid_v = v_coords[valid_mask].astype(np.float32)
            valid_depth = depth[valid_mask]
            valid_semantic = semantic_labels[valid_mask]
            valid_instances = instance_ids[valid_mask]
            
            # Back-project to 3D in camera coordinates
            z_cam = valid_depth  # Z is forward (depth from camera)
            x_cam = (valid_u - self.cx) * z_cam / self.fx  # X is right
            y_cam = (valid_v - self.cy) * z_cam / self.fy  # Y is down
            
            # Transform to FLU (Forward-Left-Up) convention
            x_flu = z_cam      # Forward
            y_flu = -x_cam     # Left  
            z_flu = -y_cam     # Up
            
            # Create point cloud in FLU coordinates
            points_3d = np.column_stack([x_flu, y_flu, z_flu])
            
            # Create and publish point cloud with instance colors
            self.create_and_publish_pointcloud(points_3d, valid_semantic, valid_instances)
            
            # Create instance centroids
            self.create_and_publish_instances(points_3d, valid_semantic, valid_instances)
            
            # Log statistics
            unique_instances = np.unique(np.column_stack([valid_semantic, valid_instances]), axis=0)
            self.get_logger().info(f'Processed {len(points_3d)} points with {len(unique_instances)} unique instances')
                
        except Exception as e:
            self.get_logger().error(f'Error in process_data: {e}')

    def create_and_publish_pointcloud(self, points, semantic_labels, instance_ids):
        """Create colored point cloud with unique colors per instance"""
        if len(points) == 0:
            return
            
        # Create PointCloud2 message
        cloud = PointCloud2()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = self.frame_id
        cloud.height = 1
        cloud.width = len(points)
        cloud.is_bigendian = False
        cloud.is_dense = False

        # Define fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1),
        ]
        cloud.fields = fields
        cloud.point_step = 16
        cloud.row_step = cloud.point_step * cloud.width

        # Pack point data with unique colors per instance
        data = []
        for i in range(len(points)):
            x, y, z = points[i]
            semantic_label = int(semantic_labels[i])
            instance_id = int(instance_ids[i])
            
            # Get unique color for this instance
            color = self.get_instance_color(semantic_label, instance_id)
            rgb = struct.unpack('I', struct.pack('BBBB', 
                int(color[2]*255), int(color[1]*255), int(color[0]*255), 255))[0]
            
            data.append(struct.pack('ffff', x, y, z, rgb))
        
        cloud.data = b''.join(data)
        self.pointcloud_pub.publish(cloud)

    def create_and_publish_instances(self, points, semantic_labels, instance_ids):
        """Create markers for each unique object instance"""
        if len(points) == 0:
            return
            
        # Create pose array and markers
        poses = PoseArray()
        poses.header.stamp = self.get_clock().now().to_msg()
        poses.header.frame_id = self.frame_id
        
        markers = MarkerArray()
        now = self.get_clock().now().to_msg()
        
        # Find unique instances (semantic_label, instance_id) pairs
        unique_instances = {}
        for i in range(len(points)):
            semantic_label = int(semantic_labels[i])
            instance_id = int(instance_ids[i])
            
            if semantic_label == 0:  # Skip background
                continue
                
            key = (semantic_label, instance_id)
            if key not in unique_instances:
                unique_instances[key] = []
            unique_instances[key].append(points[i])
        
        marker_id = 0
        for (semantic_label, instance_id), instance_points in unique_instances.items():
            instance_points = np.array(instance_points)
            
            if len(instance_points) < 10:  # Need minimum points
                continue
                
            # Compute centroid
            centroid = np.mean(instance_points, axis=0)
            class_name = self.class_names.get(semantic_label, f'class_{semantic_label}')
            
            # Compute principal axis using our simple PCA implementation
            principal_axis = self.simple_pca(instance_points)
            
            # Create pose
            pose = Pose()
            pose.position.x = float(centroid[0])
            pose.position.y = float(centroid[1])
            pose.position.z = float(centroid[2])
            pose.orientation.w = 1.0
            poses.poses.append(pose)
            
            # Create marker sphere
            marker = Marker()
            marker.header.stamp = now
            marker.header.frame_id = self.frame_id
            marker.ns = 'panoptic_instances'
            marker.id = marker_id
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose = pose
            marker.scale.x = marker.scale.y = marker.scale.z = 0.2
            
            # Use unique instance color
            color = self.get_instance_color(semantic_label, instance_id)
            marker.color.r, marker.color.g, marker.color.b = [float(c) for c in color]
            marker.color.a = 0.8
            marker.lifetime = Duration(sec=2, nanosec=0)
            
            # Create text marker
            text_marker = Marker()
            text_marker.header = marker.header
            text_marker.ns = 'panoptic_labels'
            text_marker.id = marker_id + 1000  # Offset to avoid ID conflicts
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = pose
            text_marker.pose.position.z += 0.0
            text_marker.scale.z = 0.2
            text_marker.color.r = text_marker.color.g = text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"{class_name}_#{instance_id}"
            text_marker.lifetime = Duration(sec=2, nanosec=0)
            
            # Create arrow marker for principal axis
            arrow_marker = Marker()
            arrow_marker.header.stamp = now
            arrow_marker.header.frame_id = self.frame_id
            arrow_marker.ns = 'panoptic_axes'
            arrow_marker.id = marker_id + 2000  # Offset to avoid ID conflicts
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            
            # Set arrow start point (centroid)
            arrow_marker.points = []
            start_point = geometry_msgs.msg.Point()
            start_point.x = float(centroid[0])
            start_point.y = float(centroid[1])
            start_point.z = float(centroid[2])
            
            # Set arrow end point (centroid + principal_axis * 0.2)
            end_point = geometry_msgs.msg.Point()
            end_point.x = float(centroid[0] + principal_axis[0] * 0.4)
            end_point.y = float(centroid[1] + principal_axis[1] * 0.4)
            end_point.z = float(centroid[2] + principal_axis[2] * 0.4)
            
            arrow_marker.points = [start_point, end_point]
            
            # Set arrow properties
            arrow_marker.scale.x = 0.02  # Shaft diameter
            arrow_marker.scale.y = 0.04  # Head diameter
            arrow_marker.scale.z = 0.0   # Not used for arrows
            
            # Use same color as instance but slightly brighter
            arrow_marker.color.r, arrow_marker.color.g, arrow_marker.color.b = [float(c) for c in color]
            arrow_marker.color.a = 1.0
            arrow_marker.lifetime = Duration(sec=2, nanosec=0)
            
            markers.markers.extend([marker, text_marker, arrow_marker])
            marker_id += 1
        
        # Publish results
        if poses.poses:
            self.poses_pub.publish(poses)
            
        if markers.markers:
            self.markers_pub.publish(markers)

def main():
    rclpy.init()
    node = PanopticLocator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
