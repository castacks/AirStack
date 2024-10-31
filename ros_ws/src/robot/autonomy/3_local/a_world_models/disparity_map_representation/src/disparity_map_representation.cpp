#include <disparity_map_representation/disparity_map_representation.hpp>
namespace disparity_map_representation {
DisparityMapRepresentation::DisparityMapRepresentation() : MapRepresentation(), disp_graph() {
    points_marker.ns = "obstacles";
    points_marker.id = 0;
    points_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    points_marker.action = visualization_msgs::msg::Marker::ADD;
    points_marker.scale.x = 0.03;
    points_marker.scale.y = 0.03;
    points_marker.scale.z = 0.03;

    green.r = 0;
    green.b = 0;
    green.g = 1;
    green.a = 0.8;
    gray.r = 0.5;
    gray.b = 0.5;
    gray.g = 0.5;
    gray.a = 0.5;
    red.r = 1;
    red.b = 0;
    red.g = 0;
    red.a = 0.5;

    Q_UP.setRPY(0, -M_PI / 2, 0);
    Q_DOWN.setRPY(0, M_PI / 2, 0);
    Q_LEFT.setRPY(0, 0, M_PI / 2);
    Q_RIGHT.setRPY(0, 0, -M_PI / 2);
    Q_DIRECTIONS = {Q_UP, Q_DOWN, Q_LEFT, Q_RIGHT};
}

void DisparityMapRepresentation::initialize(const rclcpp::Node::SharedPtr& node_ptr,
                                            const std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr) {
    RCLCPP_INFO(node_ptr->get_logger(), "DisparityMapRepresentation initialize called");
    MapRepresentation::initialize(node_ptr, tf_buffer_ptr);
    node_ptr->declare_parameter<int>("obstacle_check_num_points", 69);
    node_ptr->get_parameter("obstacle_check_num_points", this->obstacle_check_num_points);
    RCLCPP_INFO_STREAM(node_ptr->get_logger(),
                       "obstacle_check_num_points: " << this->obstacle_check_num_points);
    node_ptr->get_parameter("obstacle_check_radius", this->obstacle_check_radius);
    RCLCPP_INFO_STREAM(node_ptr->get_logger(),
                       "obstacle_check_radius: " << this->obstacle_check_radius);
    disp_graph.initialize(node_ptr, tf_buffer_ptr);
}

void DisparityMapRepresentation::check_pose_and_add_marker(const Trajectory& trajectory,
                                                           const Waypoint& waypoint, double dist,
                                                           const tf2::Vector3 direction,
                                                           double& closest_obstacle_distance) {
    tf2::Vector3 point_to_check = waypoint.position() + dist * direction;
    geometry_msgs::msg::PoseStamped pose_to_check;
    pose_to_check.header.frame_id = trajectory.get_frame_id();
    pose_to_check.header.stamp = trajectory.get_stamp();
    pose_to_check.pose.position.x = point_to_check.x();
    pose_to_check.pose.position.y = point_to_check.y();
    pose_to_check.pose.position.z = point_to_check.z();
    pose_to_check.pose.orientation.w = 1.0;
    // RCLCPP_INFO_STREAM(node_ptr->get_logger(),
    //                    "check_pose: " << check_pose.pose.position.x << " "
    //                                   << check_pose.pose.position.y << " "
    //                                   << check_pose.pose.position.z);

    std_msgs::msg::ColorRGBA color;

    auto [is_seen, is_free, occupancy] = disp_graph.is_pose_seen_and_free(pose_to_check, 0.9);
    if (is_seen && is_free) {
        // RCLCPP_INFO_STREAM(node_ptr->get_logger(), "no collision");
        color = green;
    } else {
        // RCLCPP_INFO_STREAM(node_ptr->get_logger(), "collision");
        closest_obstacle_distance = std::min(dist, closest_obstacle_distance);
        if (!is_seen)
            color = gray;
        else if (!is_free)
            color = red;
    }

    points_marker.points.push_back(pose_to_check.pose.position);
    points_marker.colors.push_back(color);
}

tf2::Vector3 DisparityMapRepresentation::determine_waypoint_direction(const Trajectory& trajectory,
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

std::vector<std::vector<double> > DisparityMapRepresentation::get_values(
    std::vector<Trajectory> trajectories) {
    marker_array.markers.clear();
    points_marker.points.clear();
    points_marker.colors.clear();

    // initialize values to 0
    std::vector<std::vector<double> > values(trajectories.size());
    for (size_t i = 0; i < trajectories.size(); i++) {
        values[i] = std::vector<double>(trajectories[i].get_num_waypoints(), 0.0);
    }

    size_t trajectory_index = -1;
    for (const Trajectory& trajectory : trajectories) {
        ++trajectory_index;
        points_marker.header.frame_id = trajectory.get_frame_id();

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

            double closest_obstacle_distance = obstacle_check_radius;
            for (auto direction : direction_vectors) {
                for (int k = 1; k < obstacle_check_num_points + 1; k++) {
                    double dist = k * obstacle_check_radius / (obstacle_check_num_points + 1);

                    check_pose_and_add_marker(trajectory, waypoint, dist, direction,
                                              closest_obstacle_distance);
                }
            }

            // check the waypoint itself
            check_pose_and_add_marker(trajectory, waypoint, 0, waypoint_direction,
                                      closest_obstacle_distance);

            values[trajectory_index][waypoint_index] = closest_obstacle_distance;
        }
    }

    points_marker.header.stamp = node_ptr->now();
    marker_array.markers.push_back(points_marker);

    return values;
}

double DisparityMapRepresentation::distance_to_obstacle(geometry_msgs::msg::PoseStamped pose,
                                                        tf2::Vector3 direction) {
    points_marker.header.frame_id = pose.header.frame_id;
    std_msgs::msg::ColorRGBA green;
    green.r = 0;
    green.b = 0;
    green.g = 1;
    green.a = 1;
    std_msgs::msg::ColorRGBA red;
    red.r = 1;
    red.b = 0;
    red.g = 0;
    red.a = 1;

    tf2::Quaternion q_up, q_down, q_left, q_right;
    q_up.setRPY(0, -M_PI / 2, 0);
    q_down.setRPY(0, M_PI / 2, 0);
    q_left.setRPY(0, 0, M_PI / 2);
    q_right.setRPY(0, 0, -M_PI / 2);
    std::vector<tf2::Quaternion> directions;
    directions.push_back(q_up);
    directions.push_back(q_down);
    directions.push_back(q_left);
    directions.push_back(q_right);

    tf2::Vector3 position;
    tf2::fromMsg(pose.pose.position, position);
    tf2::Quaternion q;
    tf2::fromMsg(pose.pose.orientation, q);
    tf2::Vector3 unit(1, 0, 0);
    double closest_obstacle_distance = obstacle_check_radius;

    direction.normalize();
    tf2::Vector3 up = tf2::Transform(q_up * q) * unit;
    up.normalize();

    tf2::Vector3 side = up.cross(direction);
    side.normalize();

    up = side.cross(direction);
    up.normalize();

    std::vector<tf2::Vector3> direction_vectors;
    direction_vectors.push_back(up);
    direction_vectors.push_back(side);
    direction_vectors.push_back(-up);
    direction_vectors.push_back(-side);

    for (size_t m = 0; m < directions.size(); m++) {
        // tf2::Quaternion q_curr = directions[m];
        for (int k = 1; k < obstacle_check_num_points + 1; k++) {
            double dist =
                (double)k * obstacle_check_radius / (double)(obstacle_check_num_points + 1);

            tf2::Vector3 point =
                position + dist * direction_vectors[m];  // tf::Transform(q_curr)*(dist*direction);
            geometry_msgs::msg::PoseStamped check_pose;
            check_pose.header = pose.header;
            check_pose.pose.position.x = point.x();
            check_pose.pose.position.y = point.y();
            check_pose.pose.position.z = point.z();
            check_pose.pose.orientation.w = 1.0;

            auto [is_seen, is_free, occupancy] = disp_graph.is_pose_seen_and_free(pose, 0.9);

            std_msgs::msg::ColorRGBA c;
            if (!is_seen || !is_free) {
                closest_obstacle_distance = std::min(dist, closest_obstacle_distance);
                c = red;
            } else
                c = green;

            points_marker.points.push_back(check_pose.pose.position);
            points_marker.colors.push_back(c);
        }
    }

    points_marker.points.push_back(pose.pose.position);
    points_marker.colors.push_back(green);

    auto [is_seen, is_free, occupancy] = disp_graph.is_pose_seen_and_free(pose, 0.9);
    if (!is_seen || !is_free) return 0.f;

    return closest_obstacle_distance;
}

const visualization_msgs::msg::MarkerArray& DisparityMapRepresentation::get_debug_markerarray()
    const {
    return marker_array;
}

}  // namespace disparity_map_representation
#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(disparity_map_representation::DisparityMapRepresentation,
                       map_representation_interface::MapRepresentation)
