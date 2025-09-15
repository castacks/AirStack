#!/usr/bin/env python3
"""
Filter Predicted Semantics
Subscribes to predicted and observed semantics topics and filters out predicted
semantics that are close to observed ones based on a distance threshold.
Also visualizes camera frustum based on camera info and pose.
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math
import struct
from collections import defaultdict
import colorsys
from sensor_msgs.msg import PointCloud2, CameraInfo
import sensor_msgs_py.point_cloud2 as pc2
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, PoseStamped, Quaternion, TransformStamped, Pose
from nav_msgs.msg import Path, Odometry
import tf2_ros
import tf2_geometry_msgs
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import TransformException
import rclpy.duration
import geometry_msgs.msg


class FilterPredictedSemantics(Node):
    def __init__(self):
        super().__init__('filter_predicted_semantics')
        
        # Parameters
        self.declare_parameter('distance_threshold', 1.0)
        self.declare_parameter('camera_fov_distance_threshold', 15.0)  # New parameter for camera FOV distance
        self.distance_threshold = self.get_parameter('distance_threshold').get_parameter_value().double_value
        self.camera_fov_distance_threshold = self.get_parameter('camera_fov_distance_threshold').get_parameter_value().double_value
        
        # Data storage
        self.observed_semantics = None
        self.predicted_semantics = None
        self.camera_info = None
        self.camera_pose = None
        self.odom_pose = None
        self.odom_frame_id = None
        self.camera_to_odom_transform = None
        self.vdb_map_data = None
        self.vdb_map_received = False
        
        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscribers
        self.sub_observed = self.create_subscription(
            MarkerArray,
            'observed_semantics',
            self.observed_callback,
            10
        )
        
        self.sub_predicted = self.create_subscription(
            MarkerArray,
            'predicted_semantics',
            self.predicted_callback,
            10
        )
        
        self.sub_camera_info = self.create_subscription(
            CameraInfo,
            '/panoptic/camera_info',
            self.camera_info_callback,
            10
        )
        
        self.sub_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        
        self.sub_vdb_map = self.create_subscription(
            Marker,
            'vdb_map_visualization',
            self.vdb_map_callback,
            10
        )
        
        # Publishers
        self.pub_filtered = self.create_publisher(
            MarkerArray,
            'filtered_predicted_semantics',
            10
        )
        
        self.pub_stats = self.create_publisher(
            MarkerArray,
            'filtering_stats',
            10
        )
        
        self.pub_camera_frustum = self.create_publisher(
            MarkerArray,
            'camera_frustum',
            10
        )
        
        self.pub_camera_path = self.create_publisher(
            Path,
            'camera_path',
            10
        )
        
        self.pub_semantics_in_fov = self.create_publisher(
            MarkerArray,
            'semantics_in_fov',
            10
        )
        
        # NEW: Publisher for camera pose
        self.pub_camera_pose = self.create_publisher(
            PoseStamped,
            'camera_pose',
            10
        )
        
        # Timer for camera frustum updates
        self.timer = self.create_timer(0.1, self.update_camera_frustum)
        
        self.get_logger().info('Filter Predicted Semantics initialized')
        self.get_logger().info(f'Distance threshold: {self.distance_threshold} m')
        self.get_logger().info('Subscribing to:')
        self.get_logger().info('  - observed_semantics')
        self.get_logger().info('  - predicted_semantics')
        self.get_logger().info('  - /panoptic/camera_info')
        self.get_logger().info('Publishing to:')
        self.get_logger().info('  - filtered_predicted_semantics')
        self.get_logger().info('  - filtering_stats')
        self.get_logger().info('  - camera_frustum')
        self.get_logger().info('  - camera_path')
    
    def camera_info_callback(self, msg):
        """Store camera info and compute FOV"""
        self.camera_info = msg
        
        # Compute FOV from camera parameters
        fx = msg.k[0]  # Focal length x
        fy = msg.k[4]  # Focal length y
        width = msg.width
        height = msg.height
        
        # Calculate FOV angles
        h_fov = 2 * math.atan(width / (2 * fx))
        v_fov = 2 * math.atan(height / (2 * fy))
        
        # self.get_logger().info(f'Camera FOV - Horizontal: {math.degrees(h_fov):.2f}°, Vertical: {math.degrees(v_fov):.2f}°')
        # self.get_logger().info(f'Camera frame: {msg.header.frame_id}')
    
    def get_camera_to_odom_transform(self):
        """Get the fixed transform between camera and odom frame using TF"""
        if self.odom_frame_id is None:
            self.get_logger().warn("Odom frame ID not available yet")
            return None
            
        try:
            # Look up the transform from camera frame to odom frame
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame_id,  # target frame (odom)
                'rmf_owl/camera_link/segmentation_camera',  # source frame (camera)
                rclpy.time.Time(),  # Use latest available time
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            return transform
            
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"Could not get camera-to-odom transform: {ex}")
            return None

    def odom_callback(self, msg):
        """Store odometry data and update camera pose"""
        self.odom_pose = msg.pose.pose
        self.odom_frame_id = msg.child_frame_id  # Get the odom frame ID
        self.update_camera_pose()

    def update_camera_pose(self):
        """Update camera pose based on odometry and TF transform"""
        if self.odom_pose is None or self.odom_frame_id is None:
            return
            
        # Get the camera-to-odom transform if we don't have it yet
        if self.camera_to_odom_transform is None:
            self.camera_to_odom_transform = self.get_camera_to_odom_transform()
            if self.camera_to_odom_transform is None:
                return
        
        # Create a pose in the odom frame (camera pose relative to odom)
        camera_pose_odom = Pose()
        camera_pose_odom.position.x = self.camera_to_odom_transform.transform.translation.x
        camera_pose_odom.position.y = self.camera_to_odom_transform.transform.translation.y
        camera_pose_odom.position.z = self.camera_to_odom_transform.transform.translation.z
        camera_pose_odom.orientation = self.camera_to_odom_transform.transform.rotation
        
        # Create a transform from the odom pose (which is already in map frame)
        odom_to_map_transform = TransformStamped()
        odom_to_map_transform.header.frame_id = 'map'
        odom_to_map_transform.header.stamp = self.get_clock().now().to_msg()
        odom_to_map_transform.child_frame_id = self.odom_frame_id
        odom_to_map_transform.transform.translation.x = self.odom_pose.position.x
        odom_to_map_transform.transform.translation.y = self.odom_pose.position.y
        odom_to_map_transform.transform.translation.z = self.odom_pose.position.z
        odom_to_map_transform.transform.rotation = self.odom_pose.orientation
        
        # Transform the camera pose from odom frame to map frame
        camera_pose_map = do_transform_pose(camera_pose_odom, odom_to_map_transform)
        
        # Convert back to PoseStamped for storage
        camera_pose_stamped = PoseStamped()
        camera_pose_stamped.header.frame_id = 'map'
        camera_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        camera_pose_stamped.pose = camera_pose_map
        
        self.camera_pose = camera_pose_stamped
        
        # NEW: Publish camera pose for visualization
        self.pub_camera_pose.publish(camera_pose_stamped)

    def get_camera_pose(self, camera_info_msg):
        """Get camera pose in world frame using odometry data"""
        if self.camera_pose is None:
            self.get_logger().warn("Camera pose not available - no odometry data")
            return None
            
        return self.camera_pose
    
    def create_camera_frustum_markers(self):
        """Create camera frustum visualization markers"""
        if not self.camera_info or not self.camera_pose:
            return MarkerArray()
        
        marker_array = MarkerArray()
        
        # Get camera parameters
        fx = self.camera_info.k[0]
        fy = self.camera_info.k[4]
        cx = self.camera_info.k[2]
        cy = self.camera_info.k[5]
        width = self.camera_info.width
        height = self.camera_info.height
        
        # Calculate FOV
        h_fov = 2 * math.atan(width / (2 * fx))
        v_fov = 2 * math.atan(height / (2 * fy))
        
        # Frustum parameters
        near_distance = 0.1  # Near plane distance
        far_distance = 10.0  # Far plane distance
        
        # Calculate frustum corners
        near_height = 2 * near_distance * math.tan(v_fov / 2)
        near_width = 2 * near_distance * math.tan(h_fov / 2)
        far_height = 2 * far_distance * math.tan(v_fov / 2)
        far_width = 2 * far_distance * math.tan(h_fov / 2)
        
        # Camera position
        cam_pos = self.camera_pose.pose.position
        
        # Create frustum corner points in camera frame
        # Near plane corners
        near_tl = Point(x=near_distance, y=-near_width/2, z=near_height/2)  # Top left
        near_tr = Point(x=near_distance, y=near_width/2, z=near_height/2)   # Top right
        near_bl = Point(x=near_distance, y=-near_width/2, z=-near_height/2) # Bottom left
        near_br = Point(x=near_distance, y=near_width/2, z=-near_height/2)  # Bottom right
        
        # Far plane corners
        far_tl = Point(x=far_distance, y=-far_width/2, z=far_height/2)      # Top left
        far_tr = Point(x=far_distance, y=far_width/2, z=far_height/2)       # Top right
        far_bl = Point(x=far_distance, y=-far_width/2, z=-far_height/2)     # Bottom left
        far_br = Point(x=far_distance, y=far_width/2, z=-far_height/2)      # Bottom right
        
        # Transform points to world frame (assuming camera is looking along +X axis)
        # For now, we'll create a simple frustum visualization
        frustum_points = [
            # Near plane
            Point(x=cam_pos.x + near_tl.x, y=cam_pos.y + near_tl.y, z=cam_pos.z + near_tl.z),
            Point(x=cam_pos.x + near_tr.x, y=cam_pos.y + near_tr.y, z=cam_pos.z + near_tr.z),
            Point(x=cam_pos.x + near_br.x, y=cam_pos.y + near_br.y, z=cam_pos.z + near_br.z),
            Point(x=cam_pos.x + near_bl.x, y=cam_pos.y + near_bl.y, z=cam_pos.z + near_bl.z),
            Point(x=cam_pos.x + near_tl.x, y=cam_pos.y + near_tl.y, z=cam_pos.z + near_tl.z),  # Close the loop
            
            # Far plane
            Point(x=cam_pos.x + far_tl.x, y=cam_pos.y + far_tl.y, z=cam_pos.z + far_tl.z),
            Point(x=cam_pos.x + far_tr.x, y=cam_pos.y + far_tr.y, z=cam_pos.z + far_tr.z),
            Point(x=cam_pos.x + far_br.x, y=cam_pos.y + far_br.y, z=cam_pos.z + far_br.z),
            Point(x=cam_pos.x + far_bl.x, y=cam_pos.y + far_bl.y, z=cam_pos.z + far_bl.z),
            Point(x=cam_pos.x + far_tl.x, y=cam_pos.y + far_tl.y, z=cam_pos.z + far_tl.z),  # Close the loop
        ]
        
        # Create frustum wireframe marker
        frustum_marker = Marker()
        frustum_marker.header.frame_id = 'map'
        frustum_marker.header.stamp = self.get_clock().now().to_msg()
        frustum_marker.ns = 'camera_frustum'
        frustum_marker.id = 0
        frustum_marker.type = Marker.LINE_STRIP
        frustum_marker.action = Marker.ADD
        frustum_marker.points = frustum_points
        frustum_marker.scale.x = 0.02  # Line width
        frustum_marker.color.r = 0.0
        frustum_marker.color.g = 1.0
        frustum_marker.color.b = 0.0
        frustum_marker.color.a = 0.8
        
        marker_array.markers.append(frustum_marker)
        
        # Create camera position marker
        camera_marker = Marker()
        camera_marker.header.frame_id = 'map'
        camera_marker.header.stamp = self.get_clock().now().to_msg()
        camera_marker.ns = 'camera_frustum'
        camera_marker.id = 1
        camera_marker.type = Marker.SPHERE
        camera_marker.action = Marker.ADD
        camera_marker.pose.position = cam_pos
        camera_marker.pose.orientation.w = 1.0
        camera_marker.scale.x = 0.1
        camera_marker.scale.y = 0.1
        camera_marker.scale.z = 0.1
        camera_marker.color.r = 1.0
        camera_marker.color.g = 0.0
        camera_marker.color.b = 0.0
        camera_marker.color.a = 1.0
        
        marker_array.markers.append(camera_marker)
        
        # Create camera direction arrow
        direction_marker = Marker()
        direction_marker.header.frame_id = 'map'
        direction_marker.header.stamp = self.get_clock().now().to_msg()
        direction_marker.ns = 'camera_frustum'
        direction_marker.id = 2
        direction_marker.type = Marker.ARROW
        direction_marker.action = Marker.ADD
        
        # Camera direction (assuming looking along +X axis)
        start_point = Point(x=cam_pos.x, y=cam_pos.y, z=cam_pos.z)
        end_point = Point(x=cam_pos.x + 1.0, y=cam_pos.y, z=cam_pos.z)
        
        direction_marker.points = [start_point, end_point]
        direction_marker.scale.x = 0.05
        direction_marker.scale.y = 0.1
        direction_marker.scale.z = 0.0
        direction_marker.color.r = 1.0
        direction_marker.color.g = 1.0
        direction_marker.color.b = 0.0
        direction_marker.color.a = 1.0
        
        marker_array.markers.append(direction_marker)
        
        return marker_array
    
    def create_camera_path(self):
        """Create camera path for trajectory visualization"""
        if not self.camera_pose:
            return None
        
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        # Add current camera position to path
        pose_stamped = PoseStamped()
        pose_stamped.header = path.header
        pose_stamped.pose = self.camera_pose.pose
        path.poses.append(pose_stamped)
        
        return path
    
    def update_camera_frustum(self):
        """Update camera frustum visualization"""
        # Get current camera pose
        self.get_camera_pose(self.camera_info)
        
        if self.camera_pose:
            # Create and publish frustum markers
            frustum_markers = self.create_camera_frustum_markers()
            self.pub_camera_frustum.publish(frustum_markers)
            
            # Create and publish camera path
            camera_path = self.create_camera_path()
            if camera_path:
                self.pub_camera_path.publish(camera_path)
    
    def observed_callback(self, msg):
        """Store observed semantics data"""
        self.observed_semantics = self.extract_semantic_objects(msg)
        self.get_logger().debug(f'Received {len(self.observed_semantics)} observed semantics')
        self.process_and_publish()
    
    def predicted_callback(self, msg):
        """Store predicted semantics data"""
        self.predicted_semantics = self.extract_semantic_objects(msg)
        self.get_logger().debug(f'Received {len(self.predicted_semantics)} predicted semantics')
        self.process_and_publish()
    
    def vdb_map_callback(self, msg):
        """Store VDB map data"""
        self.vdb_map_data = msg
        self.vdb_map_received = True
        # self.get_logger().info("Received VDB map marker: %s", msg.ns)

    def extract_semantic_objects(self, marker_array):
        """Extract semantic objects from MarkerArray message"""
        objects = []
        
        # Group markers by ID to combine sphere, text, and arrow markers
        sphere_markers = {}
        text_markers = {}
        arrow_markers = {}
        
        for marker in marker_array.markers:
            if marker.type == Marker.SPHERE:
                sphere_markers[marker.id] = marker
            elif marker.type == Marker.TEXT_VIEW_FACING:
                # Text markers have id offset by 1000 from their corresponding sphere markers
                sphere_id = marker.id - 1000
                text_markers[sphere_id] = marker
            elif marker.type == Marker.ARROW and marker.ns == "panoptic_axes":
                # Arrow markers have id offset by 2000 from their corresponding sphere markers
                sphere_id = marker.id - 2000
                arrow_markers[sphere_id] = marker
        
        # Create semantic objects from sphere markers
        for sphere_id, sphere_marker in sphere_markers.items():
            obj = {
                'id': sphere_id,
                'position': sphere_marker.pose.position,
                'text': text_markers.get(sphere_id, Marker()).text if sphere_id in text_markers else f"Object_{sphere_id}",
                'color': sphere_marker.color,
                'principal_axis': self.extract_principal_axis(arrow_markers.get(sphere_id)),
                'frame_id': sphere_marker.header.frame_id  # Store frame ID
            }
            objects.append(obj)
        
        return objects
    
    def extract_principal_axis(self, arrow_marker):
        """Extract principal axis from arrow marker"""
        if arrow_marker is None or len(arrow_marker.points) < 2:
            return Point(x=0.0, y=0.0, z=0.0)
        
        start = arrow_marker.points[0]
        end = arrow_marker.points[1]
        
        # Calculate direction vector
        axis = Point()
        axis.x = end.x - start.x
        axis.y = end.y - start.y
        axis.z = end.z - start.z
        
        # Normalize
        length = math.sqrt(axis.x**2 + axis.y**2 + axis.z**2)
        if length > 1e-6:
            axis.x /= length
            axis.y /= length
            axis.z /= length
        
        return axis
    
    def calculate_distance(self, pos1, pos2):
        """Calculate Euclidean distance between two points"""
        dx = pos1.x - pos2.x
        dy = pos1.y - pos2.y
        dz = pos1.z - pos2.z
        return math.sqrt(dx**2 + dy**2 + dz**2)
    
    def is_point_in_camera_fov(self, point):
        """Check if a point is within the camera's field of view"""
        if self.camera_pose is None or self.camera_info is None:
            return False
            
        # Calculate distance from camera to point
        dx = point.x - self.camera_pose.pose.position.x
        dy = point.y - self.camera_pose.pose.position.y
        dz = point.z - self.camera_pose.pose.position.z
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # Check if within distance threshold
        if distance > self.camera_fov_distance_threshold:
            return False
            
        # Calculate camera FOV (simplified - assuming square FOV)
        # Get camera intrinsic parameters
        fx = self.camera_info.k[0]  # focal length x
        fy = self.camera_info.k[4]  # focal length y
        cx = self.camera_info.k[2]  # principal point x
        cy = self.camera_info.k[5]  # principal point y
        
        # Calculate horizontal and vertical FOV
        image_width = self.camera_info.width
        image_height = self.camera_info.height
        h_fov = 2 * math.atan(image_width / (2 * fx))
        v_fov = 2 * math.atan(image_height / (2 * fy))
        
        # Transform point to camera frame
        point_camera = self.transform_point_to_camera_frame(point)
        # self.get_logger().info(
        #     f"Point camera: {point_camera.x}, {point_camera.y}, {point_camera.z}"
        # )
        # Check if point is in front of camera
        if point_camera.z <= 0:
            return False
            
        # Calculate angles from camera center
        angle_x = math.atan2(point_camera.y, point_camera.x)
        angle_y = math.atan2(point_camera.z, point_camera.x)
        # self.get_logger().info(
        #     f"Angle x: {angle_x/math.pi*180}, Angle y: {angle_y/math.pi*180}"
        # )
        
        # Check if within FOV
        return (abs(angle_x) <= h_fov/2 and abs(angle_y) <= v_fov/2)

    def transform_point_to_camera_frame(self, point):
        """Transform a point from world frame to camera frame using proper inverse transform"""
        if self.camera_pose is None:
            return point

        # Get camera pose in world frame
        cam_pos = self.camera_pose.pose.position
        cam_ori = self.camera_pose.pose.orientation

        # Convert quaternion to rotation matrix
        qx, qy, qz, qw = cam_ori.x, cam_ori.y, cam_ori.z, cam_ori.w
        # Rotation matrix (world to camera)
        R = np.array([
            [1 - 2*qy**2 - 2*qz**2,     2*qx*qy - 2*qz*qw,     2*qx*qz + 2*qy*qw],
            [2*qx*qy + 2*qz*qw,     1 - 2*qx**2 - 2*qz**2,     2*qy*qz - 2*qx*qw],
            [2*qx*qz - 2*qy*qw,     2*qy*qz + 2*qx*qw,     1 - 2*qx**2 - 2*qy**2]
        ])

        # Point in world frame as vector
        p_world = np.array([point.x, point.y, point.z])
        cam_t = np.array([cam_pos.x, cam_pos.y, cam_pos.z])

        # Transform: first translate, then rotate (inverse transform)
        p_rel = p_world - cam_t
        # Inverse rotation: R^T
        p_cam = R.T @ p_rel

        return Point(x=p_cam[0], y=p_cam[1], z=p_cam[2])

    def is_line_of_sight_clear(self, camera_pos, object_pos):
        """Check if line of sight between camera and object is clear (no VDB obstacles)"""
        if not self.vdb_map_received or self.vdb_map_data is None:
            return True  # No VDB data, assume clear
            
        if self.vdb_map_data.type != Marker.CUBE_LIST:
            return True  # Not a VDB map, assume clear
            
        # Get voxel size from the marker
        voxel_size = self.vdb_map_data.scale.x
        
        # Sample points along the line from camera to object
        num_samples = int(math.ceil(self.calculate_distance(camera_pos, object_pos) / (voxel_size / 2.0)))
        if num_samples < 2:
            num_samples = 2
            
        for i in range(num_samples-2):
            t = i / (num_samples - 1) if num_samples > 1 else 0
            sample_point = Point()
            sample_point.x = camera_pos.x + t * (object_pos.x - camera_pos.x)
            sample_point.y = camera_pos.y + t * (object_pos.y - camera_pos.y)
            sample_point.z = camera_pos.z + t * (object_pos.z - camera_pos.z)
            
            # Check if this sample point collides with any VDB voxel
            if self.is_point_in_vdb_collision(sample_point, voxel_size):
                return False
                
        return True

    def is_point_in_vdb_collision(self, point, voxel_size):
        """Check if a point collides with VDB voxels"""
        # Check distance to each voxel in the VDB map
        for voxel_point in self.vdb_map_data.points:
            dx = point.x - voxel_point.x
            dy = point.y - voxel_point.y
            dz = point.z - voxel_point.z
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            # Check if point is within voxel (using half voxel size as threshold)
            if distance < voxel_size / 2.0:
                return True
                
        return False

    def create_semantics_in_fov_marker_array(self, semantics_in_fov):
        """Create MarkerArray for semantics in camera FOV"""
        marker_array = MarkerArray()
        
        for i, semantic_obj in enumerate(semantics_in_fov):
            # Create sphere marker
            sphere_marker = Marker()
            sphere_marker.header.frame_id = 'map'
            sphere_marker.header.stamp = self.get_clock().now().to_msg()
            sphere_marker.ns = 'semantics_in_fov'
            sphere_marker.id = i
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            sphere_marker.pose.position = semantic_obj['position']
            sphere_marker.pose.orientation.w = 1.0
            sphere_marker.scale.x = 0.2
            sphere_marker.scale.y = 0.2
            sphere_marker.scale.z = 0.2
            sphere_marker.color.r = 1.0  # Yellow for FOV semantics
            sphere_marker.color.g = 1.0
            sphere_marker.color.b = 0.0
            sphere_marker.color.a = 0.8
            sphere_marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
            
            marker_array.markers.append(sphere_marker)
            
            # Create text marker
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'semantics_in_fov_text'
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = semantic_obj['position'].x
            text_marker.pose.position.y = semantic_obj['position'].y
            text_marker.pose.position.z = semantic_obj['position'].z + 0.3
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.2
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 0.0
            text_marker.color.a = 1.0
            text_marker.text = semantic_obj.get('text', f'fov_{i}')
            text_marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
            
            marker_array.markers.append(text_marker)
        
        return marker_array

    def filter_predicted_semantics(self, predicted_semantics, observed_semantics):
        """Filter predicted semantics by removing those close to observed ones"""
        filtered_predicted = []
        semantics_in_fov = []  # NEW: Track semantics in FOV
        
        for predicted_obj in predicted_semantics:
            is_too_close = False
            
            # Check distance to all observed objects
            for observed_obj in observed_semantics:
                distance = self.calculate_distance(predicted_obj['position'], observed_obj['position'])
                if distance <= self.distance_threshold:
                    is_too_close = True
                    # self.get_logger().debug(
                    #     "Filtered predicted object %s (distance: %.2f m to observed %s)",
                    #     predicted_obj.get('text', 'unknown'), distance, observed_obj.get('text', 'unknown')
                    # )
                    break
            
            # Check camera FOV and line of sight
            if not is_too_close and self.camera_pose is not None:
                # self.get_logger().info(
                #     f"Checking if object {predicted_obj.get('text', 'unknown')} is in camera FOV"
                # )
                if self.is_point_in_camera_fov(predicted_obj['position']):
                    # Add to FOV semantics list
                    semantics_in_fov.append(predicted_obj)
                    
                    # Check line of sight
                    camera_pos = self.camera_pose.pose.position
                    if self.is_line_of_sight_clear(camera_pos, predicted_obj['position']):
                        # Object is in FOV, within range, and has clear line of sight
                        # Remove it from filtered list (it's already observed by camera)
                        # self.get_logger().debug(
                        #     "Filtered predicted object %s (in camera FOV with clear LOS)",
                        #     predicted_obj.get('text', 'unknown')
                        # )
                        is_too_close = True
            
            if not is_too_close:
                filtered_predicted.append(predicted_obj)
        
        # NEW: Publish semantics in FOV
        if semantics_in_fov:
            fov_markers = self.create_semantics_in_fov_marker_array(semantics_in_fov)
            self.pub_semantics_in_fov.publish(fov_markers)
        
        # self.get_logger().info(
        #     "Filtered %zu predicted objects, %zu remaining, %zu in FOV (distance_threshold: %.2f m, camera_fov_threshold: %.2f m)",
        #     len(predicted_semantics) - len(filtered_predicted), len(filtered_predicted), len(semantics_in_fov),
        #     self.distance_threshold, self.camera_fov_distance_threshold
        # )
        
        return filtered_predicted
    
    def process_and_publish(self):
        """Process data and publish filtered results"""
        if not self.observed_semantics or not self.predicted_semantics:
            return
        
        # Filter predicted semantics
        filtered_predicted = self.filter_predicted_semantics(self.predicted_semantics, self.observed_semantics)
        
        # Create filtered marker array
        filtered_markers = self.create_filtered_marker_array(filtered_predicted)
        
        # Publish filtered results
        self.pub_filtered.publish(filtered_markers)
        
        # Create and publish statistics
        stats_markers = self.create_stats_markers(len(self.predicted_semantics), 
                                                len(filtered_predicted), 
                                                len(self.predicted_semantics) - len(filtered_predicted)) # Removed removed_count
        self.pub_stats.publish(stats_markers)
        
        # self.get_logger().info(
        #     f"Filtered predicted semantics: {len(filtered_predicted)}/{len(self.predicted_semantics)} "
        #     f"objects remain (removed {removed_count} close to observed)"
        # )
    
    def create_filtered_marker_array(self, filtered_objects):
        """Create MarkerArray message for filtered predicted semantics"""
        marker_array = MarkerArray()
        
        for i, obj in enumerate(filtered_objects):
            # Create sphere marker
            sphere_marker = Marker()
            sphere_marker.header.frame_id = obj['frame_id']  # Use original frame ID
            sphere_marker.header.stamp = self.get_clock().now().to_msg()
            sphere_marker.ns = 'panoptic_instances'
            sphere_marker.id = i
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            sphere_marker.pose.position = obj['position']  # Use exact position
            sphere_marker.pose.orientation.w = 1.0
            sphere_marker.scale.x = 0.2
            sphere_marker.scale.y = 0.2
            sphere_marker.scale.z = 0.2
            
            # Use orange color for filtered predicted
            sphere_marker.color.r = 1.0
            sphere_marker.color.g = 0.5
            sphere_marker.color.b = 0.0
            sphere_marker.color.a = 0.8
            
            marker_array.markers.append(sphere_marker)
            
            # Create text marker - Following panoptic_3d_locator.py configuration exactly
            text_marker = Marker()
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.header.frame_id = obj['frame_id']
            text_marker.ns = 'panoptic_labels'
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = obj['position'].x
            text_marker.pose.position.y = obj['position'].y
            text_marker.pose.position.z = obj['position'].z + 0.0  # Same height as sphere
            text_marker.pose.orientation.w = 1.0
            text_marker.scale.z = 0.2
            text_marker.color.r = text_marker.color.g = text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = obj['text'] if obj['text'] else f"Object_{i}"
            
            # Add lifetime like in panoptic_3d_locator.py
            from builtin_interfaces.msg import Duration
            text_marker.lifetime = Duration(sec=2, nanosec=0)
            
            marker_array.markers.append(text_marker)
            
            # Debug logging
            # self.get_logger().info(f"Created text marker for '{obj['text']}' at position ({text_marker.pose.position.x:.2f}, {text_marker.pose.position.y:.2f}, {text_marker.pose.position.z:.2f})")
            
            # Create arrow marker for principal axis
            if (obj['principal_axis'].x != 0.0 or 
                obj['principal_axis'].y != 0.0 or 
                obj['principal_axis'].z != 0.0):
                
                arrow_marker = Marker()
                arrow_marker.header.stamp = self.get_clock().now().to_msg()
                arrow_marker.header.frame_id = obj['frame_id']
                arrow_marker.ns = 'panoptic_axes'
                arrow_marker.id = i + 2000
                arrow_marker.type = Marker.ARROW
                arrow_marker.action = Marker.ADD
                
                # Set arrow start point (exact object position)
                arrow_marker.points = []
                start_point = Point()
                start_point.x = obj['position'].x
                start_point.y = obj['position'].y
                start_point.z = obj['position'].z
                
                # Set arrow end point (object position + principal_axis * 0.4)
                end_point = Point()
                end_point.x = obj['position'].x + obj['principal_axis'].x * 0.4
                end_point.y = obj['position'].y + obj['principal_axis'].y * 0.4
                end_point.z = obj['position'].z + obj['principal_axis'].z * 0.4
                
                arrow_marker.points = [start_point, end_point]
                
                # Set arrow properties like in panoptic_3d_locator.py
                arrow_marker.scale.x = 0.02  # Shaft diameter
                arrow_marker.scale.y = 0.04  # Head diameter
                arrow_marker.scale.z = 0.0   # Not used for arrows
                
                # Use same color as sphere but slightly brighter
                arrow_marker.color.r, arrow_marker.color.g, arrow_marker.color.b = [float(c) for c in [1.0, 0.5, 0.0]]
                arrow_marker.color.a = 1.0
                arrow_marker.lifetime = Duration(sec=2, nanosec=0)
                
                marker_array.markers.append(arrow_marker)
        
        return marker_array
    
    def create_stats_markers(self, total_predicted, filtered_count, removed_count):
        """Create statistics markers for visualization"""
        marker_array = MarkerArray()
        
        # Create text marker with statistics
        stats_marker = Marker()
        stats_marker.header.frame_id = 'map'
        stats_marker.header.stamp = self.get_clock().now().to_msg()
        stats_marker.ns = 'filtering_stats'
        stats_marker.id = 0
        stats_marker.type = Marker.TEXT_VIEW_FACING
        stats_marker.action = Marker.ADD
        stats_marker.pose.position.x = 0.0
        stats_marker.pose.position.y = 0.0
        stats_marker.pose.position.z = 2.0
        stats_marker.pose.orientation.w = 1.0
        stats_marker.scale.z = 0.3
        stats_marker.color.r = 1.0
        stats_marker.color.g = 1.0
        stats_marker.color.b = 1.0
        stats_marker.color.a = 1.0
        
        stats_marker.text = (
            f"Filtering Stats:\n"
            f"Total Predicted: {total_predicted}\n"
            f"Filtered: {filtered_count}\n"
            f"Removed: {removed_count}\n"
            f"Threshold: {self.distance_threshold:.1f}m"
        )
        
        marker_array.markers.append(stats_marker)
        
        return marker_array


def main(args=None):
    rclpy.init(args=args)
    node = FilterPredictedSemantics()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main() 