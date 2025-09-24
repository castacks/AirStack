#!/usr/bin/env python3
"""
Filter Predicted Semantics
Subscribes to predicted and observed semantics topics and filters out predicted
semantics that are close to observed ones based on a distance threshold.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import math


class FilterPredictedSemantics(Node):
    def __init__(self):
        super().__init__('filter_predicted_semantics')
        
        # Parameters
        self.distance_threshold = 1.0  # meters
        self.declare_parameter('distance_threshold', self.distance_threshold)
        self.distance_threshold = self.get_parameter('distance_threshold').value
        
        # Data storage
        self.observed_semantics = []
        self.predicted_semantics = []
        
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
        
        self.get_logger().info('Filter Predicted Semantics initialized')
        self.get_logger().info(f'Distance threshold: {self.distance_threshold} m')
        self.get_logger().info('Subscribing to:')
        self.get_logger().info('  - observed_semantics')
        self.get_logger().info('  - predicted_semantics')
        self.get_logger().info('Publishing to:')
        self.get_logger().info('  - filtered_predicted_semantics')
        self.get_logger().info('  - filtering_stats')
    
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
    
    def extract_semantic_objects(self, marker_array):
        """Extract semantic objects from MarkerArray message"""
        objects = []
        
        # Group markers by ID to combine sphere and arrow markers
        sphere_markers = {}
        arrow_markers = {}
        
        for marker in marker_array.markers:
            if marker.type == Marker.SPHERE:
                sphere_markers[marker.id] = marker
            elif marker.type == Marker.ARROW and marker.ns == "panoptic_axes":
                # Arrow markers have id offset by 2000 from their corresponding sphere markers
                sphere_id = marker.id - 2000
                arrow_markers[sphere_id] = marker
        
        # Create semantic objects from sphere markers
        for sphere_id, sphere_marker in sphere_markers.items():
            obj = {
                'id': sphere_id,
                'position': sphere_marker.pose.position,
                'text': sphere_marker.text,
                'color': sphere_marker.color,
                'principal_axis': self.extract_principal_axis(arrow_markers.get(sphere_id))
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
    
    def filter_predicted_semantics(self):
        """Filter predicted semantics by removing those close to observed ones"""
        filtered_predicted = []
        removed_count = 0
        
        for predicted_obj in self.predicted_semantics:
            is_close_to_observed = False
            
            for observed_obj in self.observed_semantics:
                distance = self.calculate_distance(
                    predicted_obj['position'], 
                    observed_obj['position']
                )
                
                if distance <= self.distance_threshold:
                    is_close_to_observed = True
                    removed_count += 1
                    self.get_logger().debug(
                        f"Filtered out predicted object {predicted_obj['text']} "
                        f"(distance: {distance:.2f} m to observed {observed_obj['text']})"
                    )
                    break
            
            if not is_close_to_observed:
                filtered_predicted.append(predicted_obj)
        
        return filtered_predicted, removed_count
    
    def process_and_publish(self):
        """Process data and publish filtered results"""
        if not self.observed_semantics or not self.predicted_semantics:
            return
        
        # Filter predicted semantics
        filtered_predicted, removed_count = self.filter_predicted_semantics()
        
        # Create filtered marker array
        filtered_markers = self.create_filtered_marker_array(filtered_predicted)
        
        # Publish filtered results
        self.pub_filtered.publish(filtered_markers)
        
        # Create and publish statistics
        stats_markers = self.create_stats_markers(len(self.predicted_semantics), 
                                                len(filtered_predicted), 
                                                removed_count)
        self.pub_stats.publish(stats_markers)
        
        self.get_logger().info(
            f"Filtered predicted semantics: {len(filtered_predicted)}/{len(self.predicted_semantics)} "
            f"objects remain (removed {removed_count} close to observed)"
        )
    
    def create_filtered_marker_array(self, filtered_objects):
        """Create MarkerArray message for filtered predicted semantics"""
        marker_array = MarkerArray()
        
        for i, obj in enumerate(filtered_objects):
            # Create sphere marker
            sphere_marker = Marker()
            sphere_marker.header.frame_id = 'map'
            sphere_marker.header.stamp = self.get_clock().now().to_msg()
            sphere_marker.ns = 'filtered_predicted_semantics'
            sphere_marker.id = i
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            sphere_marker.pose.position = obj['position']
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
            
            # Create text marker
            text_marker = Marker()
            text_marker.header = sphere_marker.header
            text_marker.ns = 'filtered_predicted_labels'
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose = sphere_marker.pose
            text_marker.pose.position.z += 0.3
            text_marker.scale.z = 0.2
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = obj['text']
            
            marker_array.markers.append(text_marker)
            
            # Create arrow marker for principal axis
            if (obj['principal_axis'].x != 0.0 or 
                obj['principal_axis'].y != 0.0 or 
                obj['principal_axis'].z != 0.0):
                
                arrow_marker = Marker()
                arrow_marker.header = sphere_marker.header
                arrow_marker.ns = 'filtered_predicted_axes'
                arrow_marker.id = i + 2000
                arrow_marker.type = Marker.ARROW
                arrow_marker.action = Marker.ADD
                
                # Set arrow start point
                start_point = Point()
                start_point.x = obj['position'].x
                start_point.y = obj['position'].y
                start_point.z = obj['position'].z
                
                # Set arrow end point
                end_point = Point()
                end_point.x = obj['position'].x + obj['principal_axis'].x * 0.4
                end_point.y = obj['position'].y + obj['principal_axis'].y * 0.4
                end_point.z = obj['position'].z + obj['principal_axis'].z * 0.4
                
                arrow_marker.points = [start_point, end_point]
                arrow_marker.scale.x = 0.02
                arrow_marker.scale.y = 0.04
                arrow_marker.scale.z = 0.0
                arrow_marker.color = sphere_marker.color
                
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