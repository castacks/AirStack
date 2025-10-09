// Copyright (c) 2024 Carnegie Mellon University
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <disparity_graph_cost_map/disparity_graph_cost_map.hpp>
namespace disparity_graph_cost_map {
DisparityGraphCostMap::DisparityGraphCostMap() : CostMapInterface(), disp_graph() {
    unseen_marker.ns = "unseen";
    unseen_marker.id = 0;
    unseen_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    unseen_marker.action = visualization_msgs::msg::Marker::ADD;
    unseen_marker.scale.x = 0.03;
    unseen_marker.scale.y = 0.03;
    unseen_marker.scale.z = 0.03;
    
    collision_marker = unseen_marker;
    collision_marker.ns = "collision";
    
    free_marker = unseen_marker;
    free_marker.ns = "free";

    green.r = 0;
    green.b = 0;
    green.g = 1;
    green.a = 1.0;
    gray.r = 0.7;
    gray.b = 0.7;
    gray.g = 0.7;
    gray.a = 1.0;
    red.r = 1;
    red.b = 0;
    red.g = 0;
    red.a = 1.0;

    Q_UP.setRPY(0, -M_PI / 2, 0);
    Q_DOWN.setRPY(0, M_PI / 2, 0);
    Q_LEFT.setRPY(0, 0, M_PI / 2);
    Q_RIGHT.setRPY(0, 0, -M_PI / 2);
    Q_DIRECTIONS = {Q_UP, Q_DOWN, Q_LEFT, Q_RIGHT};
}

void DisparityGraphCostMap::initialize(const rclcpp::Node::SharedPtr& node_ptr,
                                       const std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr) {
    RCLCPP_INFO(node_ptr->get_logger(), "DisparityGraphCostMap initialize called");
    CostMapInterface::initialize(node_ptr, tf_buffer_ptr);

    node_ptr->declare_parameter<double>("robot_radius", 1.0);
    node_ptr->get_parameter("robot_radius", this->robot_radius);
    RCLCPP_INFO_STREAM(node_ptr->get_logger(), "robot_radius: " << this->robot_radius);

    node_ptr->declare_parameter<int>("obstacle_check_num_points", 4);
    node_ptr->get_parameter("obstacle_check_num_points", this->obstacle_check_num_points);
    RCLCPP_INFO_STREAM(node_ptr->get_logger(),
                       "obstacle_check_num_points: " << this->obstacle_check_num_points);

    node_ptr->get_parameter("obstacle_check_radius", this->obstacle_check_radius);
    RCLCPP_INFO_STREAM(node_ptr->get_logger(),
                       "obstacle_check_radius: " << this->obstacle_check_radius);

    disp_graph.initialize(node_ptr, tf_buffer_ptr);
}

tf2::Vector3 DisparityGraphCostMap::determine_waypoint_direction(const Trajectory& trajectory,
                                                                 const Waypoint& waypoint,
                                                                 size_t waypoint_index) {
    tf2::Vector3 waypoint_direction;
    if (trajectory.get_num_waypoints() == 1) {
        // edge case: only 1 waypoint, make direction match the waypoint's heading
        waypoint_direction = quaternion_to_unit_vector(waypoint.quaternion());
    } else {
        if (&waypoint == &(trajectory.get_waypoint(0))) {
            // edge case: first waypoint, direction is from waypoint 0 to waypoint 1
            waypoint_direction = trajectory.get_waypoint(1).position() - waypoint.position();
        } else {
            // otherwise direction is from previous waypoint to current waypoint
            waypoint_direction =
                waypoint.position() - trajectory.get_waypoint(waypoint_index - 1).position();
        }
    }
    return waypoint_direction;
}

/**
 * @brief for each trajectory, calculate each waypoint's cost to be inversely proportional to the
 * distance to the closest non-free space. Algorithm: For each waypoint, check in 4 directions (up,
 * down, left, right) along the obstacle_check_radius for non-free zones. Non-free means either
 * OCCUPIED or UNOBSERVED.
 *
 * If the non-free zone is within the robot radius, then:
 *  if the non-free zone is OCCUPIED, then the cost is infinite
 *  if the non-free zone is UNOBSERVED, then the cost is NAN
 *
 * @param trajectories
 * @return std::vector<std::vector<double>> for each trajectory, the cost of each waypoint
 */
std::vector<std::vector<double> > DisparityGraphCostMap::get_trajectory_costs_per_waypoint(
    const std::vector<Trajectory>& trajectories) {
    marker_array.markers.clear();
    unseen_marker.points.clear();
    unseen_marker.colors.clear();
    collision_marker.points.clear();
    collision_marker.colors.clear();
    free_marker.points.clear();
    free_marker.colors.clear();

    // TODO trajectories are not guaranteed to all be in the same frame
    if(!trajectories.empty()){
      unseen_marker.header.frame_id = trajectories[0].get_frame_id();
      free_marker.header.frame_id = trajectories[0].get_frame_id();
      collision_marker.header.frame_id = trajectories[0].get_frame_id();
    }
    
    disp_graph.virtual_cylinder_viz();

    // initialize cost_values to 0
    std::vector<std::vector<double> > cost_values(trajectories.size());
    for (size_t i = 0; i < trajectories.size(); i++) {
        cost_values[i] = std::vector<double>(trajectories[i].get_num_waypoints(), 0.0);
    }

    size_t trajectory_index = -1;
    //RCLCPP_INFO_STREAM(node_ptr->get_logger(), "BEGIN---------------------------------------------------");
    for (const Trajectory& trajectory : trajectories) {
        ++trajectory_index;
        //points_marker.header.frame_id = trajectory.get_frame_id();

        size_t waypoint_index = -1;
        for (const Waypoint& waypoint : trajectory.get_waypoints()) {
            ++waypoint_index;
            // find the direction of the current trajectory segment between waypoints
            // this direction will be used to check for obstacles in a perpendicular direction.
            // i.e. left, right, up, down
            tf2::Vector3 waypoint_direction =
                determine_waypoint_direction(trajectory, waypoint, waypoint_index);

            auto [side, up] = create_side_and_up_vectors(waypoint_direction);

            std::set<tf2::Vector3> direction_vectors = {up, side, -up, -side};

            // for this waypoint, check around the waypoint for obstacles
            // the cost is inversely proportional to how close the nearest non-free space is
            // if the nearest occupied space is within the robot radius, the cost is infinite
            // if the nearest unknown space is within the robot radius, the cost is NAN
            bool has_checked_center = false;
            double closest_distance_to_unsafe = obstacle_check_radius;
	    // if the obstacle check radius is 1m, for example, then if the point at 1m was in collision,
	    // then closest distance to unsafe would also be 1m
	    // if everything is free, the closest distance to unsafe would also be 1m, which isn't good since its more free
	    // adding this extra increment makes the case where everything is free score slightly better to account for this
	    if(obstacle_check_num_points > 0)
	      closest_distance_to_unsafe += obstacle_check_radius / obstacle_check_num_points;
            bool is_non_free_within_robot_radius = false;
	    //RCLCPP_INFO_STREAM(node_ptr->get_logger(), "start");
            for (auto direction : direction_vectors) {
                if (is_non_free_within_robot_radius) {
                    break;
                }
                for (int k = 0; k < obstacle_check_num_points + 1; k++) {
                    bool is_center = (k == 0);
                    if (is_center && has_checked_center) {
                        continue;
                    }
                    has_checked_center = true;
                    double dist = k * obstacle_check_radius / obstacle_check_num_points;

                    // Get pose to check:
                    auto pose_to_check =
                        get_pose_at_direction_and_distance(trajectory, waypoint, dist, direction);
                    // Check the pose:
                    auto [is_seen, is_free, occupancy] =
                        disp_graph.is_pose_seen_and_free(pose_to_check, 0.9);
		    
                    // Add marker:
                    is_seen = true;
                    is_free = true;
                    add_check_marker(is_seen, is_free, dist, pose_to_check);

                    // Calculate the cost:

                    // if the point is within the robot radius, and is collision, the cost is
                    // infinite
                    if (/*dist <= this->robot_radius*/is_center && !is_free) {
                        RCLCPP_DEBUG_STREAM(node_ptr->get_logger(),
                                           "Trajectory " << trajectory_index << " waypoint "
                                                         << waypoint_index << " is non-free within "
                                                         << this->robot_radius << "m"
                                                         << " at distance " << dist
                                                         );
                        cost_values[trajectory_index][waypoint_index] =
                            std::numeric_limits<double>::infinity();
                        is_non_free_within_robot_radius = true;
                        break;
                    }
                    // if the point is within the robot radius, and is unseen, the cost is NAN
                    if (/*dist <= this->robot_radius*/is_center && !is_seen) {
                        RCLCPP_DEBUG_STREAM(
                            node_ptr->get_logger(),
                            "Trajectory " << trajectory_index << " waypoint " << waypoint_index
                                          << " is unseen within " << this->robot_radius << "m");
                        cost_values[trajectory_index][waypoint_index] =
                            std::numeric_limits<double>::quiet_NaN();
                        is_non_free_within_robot_radius = true;
                        break;
                    }
                    // otherwise if the point is beyond the robot radius, yet it is not free or not
                    // seen, say the cost is the distance
                    if (!is_seen || !is_free) {
                        closest_distance_to_unsafe = std::min(dist, closest_distance_to_unsafe);
			//RCLCPP_INFO_STREAM(node_ptr->get_logger(), "\tk: " << k << " " << dist << " " << closest_distance_to_unsafe);
                        // we can break here because we know the closest distance for this
                        // direction, don't have to check farther distances break;
                    }
                }
            }
            if (!is_non_free_within_robot_radius) {
  	        double cost = -closest_distance_to_unsafe;//1 / (1 + closest_distance_to_unsafe);
                cost_values[trajectory_index][waypoint_index] = cost;
		//RCLCPP_INFO_STREAM(node_ptr->get_logger(), "cost: " << cost << " " << closest_distance_to_unsafe);
            }
            // continue to next waypoint
        }
    }

    unseen_marker.header.stamp = node_ptr->now();
    collision_marker.header.stamp = node_ptr->now();
    free_marker.header.stamp = node_ptr->now();
    marker_array.markers.push_back(unseen_marker);
    marker_array.markers.push_back(collision_marker);
    marker_array.markers.push_back(free_marker);

    return cost_values;
}

void DisparityGraphCostMap::clear(){
  disp_graph.clear_graph();
}
  
void DisparityGraphCostMap::add_check_marker(bool is_seen, bool is_free, double distance,
                                             const geometry_msgs::msg::PoseStamped& pose_to_check) {
    std_msgs::msg::ColorRGBA color;
    if (is_seen && is_free) {
        color = green;
    } else {
        if (!is_seen) {
            color = gray;
        } else if (!is_free) {
            color = red;
        }
    }

    double alpha = (distance <= this->robot_radius) ? 1.0 : 0.1;
    color.a = alpha;

    if(!is_seen){
      unseen_marker.points.push_back(pose_to_check.pose.position);
      unseen_marker.colors.push_back(color);
    }
    else if(!is_free){
      collision_marker.points.push_back(pose_to_check.pose.position);
      collision_marker.colors.push_back(color);
    }
    else{
      free_marker.points.push_back(pose_to_check.pose.position);
      free_marker.colors.push_back(color);
    }
}

geometry_msgs::msg::PoseStamped DisparityGraphCostMap::get_pose_at_direction_and_distance(
    const Trajectory& trajectory, const Waypoint& waypoint, double dist,
    const tf2::Vector3 direction) {
    tf2::Vector3 target_to_check = waypoint.position() + dist * direction;
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id = trajectory.get_frame_id();
    target_pose.header.stamp = trajectory.get_stamp();
    target_pose.pose.position.x = target_to_check.x();
    target_pose.pose.position.y = target_to_check.y();
    target_pose.pose.position.z = target_to_check.z();
    target_pose.pose.orientation.w = 1.0;
    return target_pose;
}

const visualization_msgs::msg::MarkerArray& DisparityGraphCostMap::get_debug_markerarray() const {
    return marker_array;
}

}  // namespace disparity_graph_cost_map
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(disparity_graph_cost_map::DisparityGraphCostMap,
                       cost_map_interface::CostMapInterface) 
