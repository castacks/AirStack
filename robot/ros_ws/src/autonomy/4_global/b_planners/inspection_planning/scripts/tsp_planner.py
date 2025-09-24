#!/usr/bin/env python3
"""
TSP Planner for Inspection Poses
Subscribes to predicted and observed inspection poses, robot odometry,
and solves a TSP problem to generate an optimal traversal plan.
"""

import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
import math
from typing import List, Tuple, Optional
import time


class TSPPlanner(Node):
    def __init__(self):
        super().__init__('tsp_planner')
        
        # Declare parameters
        self.declare_parameter('update_rate', 1.0)  # Hz
        self.declare_parameter('max_poses', 50)  # Limit for computational efficiency
        self.declare_parameter('use_predicted_poses', True)
        self.declare_parameter('use_observed_poses', True)
        self.declare_parameter('algorithm', 'nearest_neighbor')  # 'nearest_neighbor', 'greedy', 'opt2'
        
        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.max_poses = self.get_parameter('max_poses').value
        self.use_predicted = self.get_parameter('use_predicted_poses').value
        self.use_observed = self.get_parameter('use_observed_poses').value
        self.algorithm = self.get_parameter('algorithm').value
        
        # Initialize data
        self.current_odometry: Optional[Odometry] = None
        self.observed_poses: List[Pose] = []
        self.predicted_poses: List[Pose] = []
        self.last_plan_time = 0.0
        self.plan_valid = False
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            10
        )
        
        self.observed_poses_sub = self.create_subscription(
            PoseArray,
            '/inspection_planner/observed_inspection_poses',
            self.observed_poses_callback,
            10
        )
        
        self.predicted_poses_sub = self.create_subscription(
            PoseArray,
            '/inspection_planner/predicted_inspection_poses',
            self.predicted_poses_callback,
            10
        )
        
        # Publisher
        self.global_plan_pub = self.create_publisher(
            Path,
            '/robot_1/global_plan',
            10
        )
        
        # Timer for periodic planning
        self.timer = self.create_timer(1.0 / self.update_rate, self.plan_callback)
        
        self.get_logger().info('TSP Planner initialized')
        self.get_logger().info(f'Algorithm: {self.algorithm}')
        self.get_logger().info(f'Max poses: {self.max_poses}')
        self.get_logger().info(f'Update rate: {self.update_rate} Hz')

    def odometry_callback(self, msg: Odometry):
        """Process robot odometry."""
        self.current_odometry = msg

    def observed_poses_callback(self, msg: PoseArray):
        """Process observed inspection poses."""
        self.observed_poses = msg.poses
        self.plan_valid = False
        self.get_logger().debug(f'Received {len(self.observed_poses)} observed poses')

    def predicted_poses_callback(self, msg: PoseArray):
        """Process predicted inspection poses."""
        self.predicted_poses = msg.poses
        self.plan_valid = False
        self.get_logger().debug(f'Received {len(self.predicted_poses)} predicted poses')

    def calculate_distance(self, pose1: Pose, pose2: Pose) -> float:
        """Calculate Euclidean distance between two poses."""
        dx = pose1.position.x - pose2.position.x
        dy = pose1.position.y - pose2.position.y
        dz = pose1.position.z - pose2.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def create_distance_matrix(self, poses: List[Pose], start_pose: Pose) -> np.ndarray:
        """Create distance matrix including start position."""
        all_poses = [start_pose] + poses
        n = len(all_poses)
        distance_matrix = np.zeros((n, n))
        
        for i in range(n):
            for j in range(n):
                if i != j:
                    distance_matrix[i][j] = self.calculate_distance(all_poses[i], all_poses[j])
        
        return distance_matrix

    def solve_tsp_nearest_neighbor(self, distance_matrix: np.ndarray) -> Tuple[List[int], float]:
        """Solve TSP using nearest neighbor heuristic."""
        n = distance_matrix.shape[0]
        if n <= 1:
            return [0], 0.0
        
        visited = [False] * n
        tour = [0]  # Start from robot position (index 0)
        visited[0] = True
        total_distance = 0.0
        
        current = 0
        for _ in range(n - 1):
            nearest_dist = float('inf')
            nearest_idx = -1
            
            for j in range(n):
                if not visited[j] and distance_matrix[current][j] < nearest_dist:
                    nearest_dist = distance_matrix[current][j]
                    nearest_idx = j
            
            if nearest_idx != -1:
                tour.append(nearest_idx)
                visited[nearest_idx] = True
                total_distance += nearest_dist
                current = nearest_idx
        
        return tour, total_distance

    def solve_tsp_greedy(self, distance_matrix: np.ndarray) -> Tuple[List[int], float]:
        """Solve TSP using greedy edge selection."""
        n = distance_matrix.shape[0]
        if n <= 1:
            return [0], 0.0
        
        # Create list of all edges with distances
        edges = []
        for i in range(n):
            for j in range(i + 1, n):
                edges.append((distance_matrix[i][j], i, j))
        
        # Sort edges by distance
        edges.sort()
        
        # Build tour using greedy approach starting from robot position
        tour = [0]
        used_edges = set()
        total_distance = 0.0
        
        while len(tour) < n:
            current = tour[-1]
            best_dist = float('inf')
            best_next = -1
            
            for dist, i, j in edges:
                if (i, j) in used_edges or (j, i) in used_edges:
                    continue
                
                if i == current and j not in tour:
                    if dist < best_dist:
                        best_dist = dist
                        best_next = j
                elif j == current and i not in tour:
                    if dist < best_dist:
                        best_dist = dist
                        best_next = i
            
            if best_next != -1:
                tour.append(best_next)
                used_edges.add((current, best_next))
                total_distance += best_dist
            else:
                break
        
        return tour, total_distance

    def solve_tsp_2opt(self, distance_matrix: np.ndarray) -> Tuple[List[int], float]:
        """Solve TSP using 2-opt improvement on nearest neighbor solution."""
        # Start with nearest neighbor solution
        tour, _ = self.solve_tsp_nearest_neighbor(distance_matrix)
        
        def calculate_tour_distance(tour_order):
            total = 0.0
            for i in range(len(tour_order) - 1):
                total += distance_matrix[tour_order[i]][tour_order[i + 1]]
            return total
        
        improved = True
        while improved:
            improved = False
            for i in range(1, len(tour) - 2):
                for j in range(i + 1, len(tour)):
                    if j - i == 1:
                        continue
                    
                    # Create new tour by reversing the segment between i and j
                    new_tour = tour[:i] + tour[i:j][::-1] + tour[j:]
                    new_distance = calculate_tour_distance(new_tour)
                    old_distance = calculate_tour_distance(tour)
                    
                    if new_distance < old_distance:
                        tour = new_tour
                        improved = True
                        break
                
                if improved:
                    break
        
        final_distance = calculate_tour_distance(tour)
        return tour, final_distance

    def solve_tsp(self, poses: List[Pose], start_pose: Pose) -> Tuple[List[int], float]:
        """Solve TSP using the configured algorithm."""
        if not poses:
            return [0], 0.0
        
        distance_matrix = self.create_distance_matrix(poses, start_pose)
        
        if self.algorithm == 'nearest_neighbor':
            return self.solve_tsp_nearest_neighbor(distance_matrix)
        elif self.algorithm == 'greedy':
            return self.solve_tsp_greedy(distance_matrix)
        elif self.algorithm == 'opt2':
            return self.solve_tsp_2opt(distance_matrix)
        else:
            self.get_logger().warn(f'Unknown algorithm: {self.algorithm}, using nearest_neighbor')
            return self.solve_tsp_nearest_neighbor(distance_matrix)

    def create_robot_pose_from_odometry(self, odom: Odometry) -> Pose:
        """Convert odometry to pose."""
        pose = Pose()
        pose.position = odom.pose.pose.position
        pose.orientation = odom.pose.pose.orientation
        return pose

    def create_path_message(self, tour: List[int], poses: List[Pose], start_pose: Pose) -> Path:
        """Create Path message from TSP solution."""
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        all_poses = [start_pose] + poses
        
        for idx in tour:
            if idx < len(all_poses):
                pose_stamped = PoseStamped()
                pose_stamped.header = path.header
                pose_stamped.pose = all_poses[idx]
                path.poses.append(pose_stamped)
        
        return path

    def plan_callback(self):
        """Main planning callback."""
        if self.current_odometry is None:
            self.get_logger().debug('Waiting for odometry...')
            return
        
        # Collect all inspection poses
        all_poses = []
        if self.use_observed:
            all_poses.extend(self.observed_poses)
        if self.use_predicted:
            all_poses.extend(self.predicted_poses)
        
        if not all_poses:
            self.get_logger().debug('No inspection poses available')
            return
        
        # Limit number of poses for computational efficiency
        if len(all_poses) > self.max_poses:
            self.get_logger().warn(f'Too many poses ({len(all_poses)}), limiting to {self.max_poses}')
            all_poses = all_poses[:self.max_poses]
        
        # Check if we need to replan
        current_time = time.time()
        if self.plan_valid and (current_time - self.last_plan_time) < 5.0:
            return
        
        # Solve TSP
        start_time = time.time()
        start_pose = self.create_robot_pose_from_odometry(self.current_odometry)
        
        try:
            tour, total_distance = self.solve_tsp(all_poses, start_pose)
            solve_time = time.time() - start_time
            
            # Create and publish path
            path = self.create_path_message(tour, all_poses, start_pose)
            self.global_plan_pub.publish(path)
            
            self.plan_valid = True
            self.last_plan_time = current_time
            
            self.get_logger().info(
                f'TSP solved: {len(all_poses)} poses, {len(tour)} waypoints, '
                f'distance: {total_distance:.2f}m, solve_time: {solve_time:.3f}s'
            )
            
        except Exception as e:
            self.get_logger().error(f'TSP solving failed: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    tsp_planner = TSPPlanner()
    
    try:
        rclpy.spin(tsp_planner)
    except KeyboardInterrupt:
        tsp_planner.get_logger().info('Shutting down TSP Planner...')
    finally:
        tsp_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()