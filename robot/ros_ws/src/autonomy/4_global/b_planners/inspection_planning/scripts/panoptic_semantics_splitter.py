#!/usr/bin/env python3
"""
Panoptic Semantics Splitter
Subscribes to '/panoptic/markers' topic and splits markers into observed and predicted semantics
Treats first half of markers as observed semantics and second half as predicted semantics
Publishes both as MarkerArray messages to 'observed_semantics' and 'predicted_semantics' topics
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, PointStamped
import copy


class PanopticSemanticsSplitter(Node):
    def __init__(self):
        super().__init__('panoptic_semantics_splitter')
        
        # TF2 for coordinate transformation
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Parameters
        self.declare_parameter('world_frame', 'map')
        self.world_frame = self.get_parameter('world_frame').get_parameter_value().string_value
        
        # Publishers
        self.observed_pub = self.create_publisher(
            MarkerArray, 
            'observed_semantics', 
            10
        )
        self.predicted_pub = self.create_publisher(
            MarkerArray, 
            'predicted_semantics', 
            10
        )
        
        # Subscriber
        self.panoptic_sub = self.create_subscription(
            MarkerArray,
            '/panoptic/markers',
            self.panoptic_markers_callback,
            10
        )
        
        self.get_logger().info('Panoptic Semantics Splitter started')
        self.get_logger().info('Subscribing to: /panoptic/markers')
        self.get_logger().info('Publishing to: observed_semantics, predicted_semantics')
        self.get_logger().info(f'World frame: {self.world_frame}')

    def transform_marker_to_world_frame(self, marker):
        """Transform a marker's pose to world frame"""
        if marker.header.frame_id == self.world_frame:
            # Already in world frame, no transformation needed
            return marker
        
        try:
            # Create a copy of the marker to avoid modifying the original
            transformed_marker = copy.deepcopy(marker)
            
            # Create a PoseStamped from the marker pose
            pose_stamped = PoseStamped()
            pose_stamped.header = marker.header
            pose_stamped.pose = marker.pose
            
            # Transform to world frame
            transformed_pose = self.tf_buffer.transform(pose_stamped, self.world_frame)
            
            # Update marker with transformed pose and frame
            transformed_marker.header.frame_id = self.world_frame
            if marker.type != marker.ARROW:
                transformed_marker.pose = transformed_pose.pose
            
            # Also transform arrow points if this is an arrow marker
            if marker.type == marker.ARROW and len(marker.points) >= 2:
                transformed_points = []
                for point in marker.points:
                    # Create PointStamped for each point (pure position, no orientation)
                    point_stamped = PointStamped()
                    point_stamped.header = marker.header
                    point_stamped.point = point
                    
                    # Transform point
                    transformed_point_stamped = self.tf_buffer.transform(point_stamped, self.world_frame)
                    transformed_points.append(transformed_point_stamped.point)
                
                transformed_marker.points = transformed_points
            
            return transformed_marker
            
        except Exception as e:
            self.get_logger().warn(
                f'Failed to transform marker from {marker.header.frame_id} to {self.world_frame}: {e}'
            )
            # Return original marker if transformation fails
            return marker

    def panoptic_markers_callback(self, msg):
        """
        Process incoming panoptic markers and split into observed/predicted
        Each semantic object has 3 markers: sphere, text, arrow (in that order)
        First half of objects -> observed_semantics
        Second half of objects -> predicted_semantics
        Transform all markers to world frame before publishing
        """
        try:
            markers = msg.markers
            num_markers = len(markers)
            
            if num_markers == 0:
                self.get_logger().debug('Received empty marker array')
                return
            
            # Each semantic object has 3 markers (sphere, text, arrow)
            if num_markers % 3 != 0:
                self.get_logger().warn(f'Expected markers to be multiple of 3, got {num_markers}')
                # Handle gracefully by trimming to nearest multiple of 3
                num_markers = (num_markers // 3) * 3
                markers = markers[:num_markers]
            
            num_objects = num_markers // 3
            
            # Transform all markers to world frame
            transformed_markers = []
            for marker in markers:
                transformed_marker = self.transform_marker_to_world_frame(marker)
                transformed_markers.append(transformed_marker)
            
            # Split by objects (groups of 3 markers), not by individual markers
            split_objects = num_objects // 2
            split_point = split_objects * 3  # Convert back to marker index
            
            # Create observed semantics message (first half of objects)
            observed_msg = MarkerArray()
            observed_msg.markers = transformed_markers[:split_point]
            
            # Update marker IDs to avoid conflicts and set namespace
            # Preserve the relationship: sphere=0, text=1000, arrow=2000 for first object
            #                          sphere=1, text=1001, arrow=2001 for second object, etc.
            for i, marker in enumerate(observed_msg.markers):
                object_idx = i // 3
                marker_type_idx = i % 3
                
                if marker_type_idx == 0:  # Sphere marker
                    marker.ns = 'observed_semantics'
                    marker.id = object_idx
                elif marker_type_idx == 1:  # Text marker  
                    marker.ns = 'observed_semantics'
                    marker.id = object_idx + 1000
                else:  # Arrow marker
                    marker.ns = 'panoptic_axes'  # Keep original namespace for arrows
                    marker.id = object_idx + 2000
                    
                marker.header.stamp = self.get_clock().now().to_msg()
            
            # Create predicted semantics message (second half of objects)
            predicted_msg = MarkerArray()
            predicted_msg.markers = transformed_markers[split_point:]
            
            # Update marker IDs to avoid conflicts and set namespace
            for i, marker in enumerate(predicted_msg.markers):
                object_idx = i // 3
                marker_type_idx = i % 3
                
                if marker_type_idx == 0:  # Sphere marker
                    marker.ns = 'predicted_semantics'
                    marker.id = object_idx
                elif marker_type_idx == 1:  # Text marker
                    marker.ns = 'predicted_semantics'
                    marker.id = object_idx + 1000
                else:  # Arrow marker
                    marker.ns = 'panoptic_axes'  # Keep original namespace for arrows
                    marker.id = object_idx + 2000
                    
                marker.header.stamp = self.get_clock().now().to_msg()
            
            # Publish both messages
            self.observed_pub.publish(observed_msg)
            self.predicted_pub.publish(predicted_msg)
            
            self.get_logger().debug(
                f'Split {num_objects} objects ({num_markers} markers): '
                f'{split_objects} observed, {num_objects - split_objects} predicted '
                f'(transformed to {self.world_frame})'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error processing panoptic markers: {e}')


def main():
    rclpy.init()
    node = PanopticSemanticsSplitter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()