
#include "../include/random_walk_node.hpp"

#include "../include/random_walk_logic.hpp"

RandomWalkNode::RandomWalkNode() : Node("random_walk_node") {
    std::optional<init_params> params_opt = RandomWalkNode::readParameters();
    // Initialize the random walk planner
    if (params_opt.has_value()) {
        this->params = params_opt.value();
    } else
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize random walk planner");

    this->create_subscription<visualization_msgs::msg::Marker>(
        sub_vdb_map_topic_, 10,
        std::bind(&RandomWalkNode::vdbmapCallback, this, std::placeholders::_1));
    this->create_subscription<nav_msgs::msg::Odometry>(
        sub_odometry_topic_, 10,
        std::bind(&RandomWalkNode::odometryCallback, this, std::placeholders::_1));

    this->pub_global_path =
        this->create_publisher<airstack_msgs::msg::TrajectoryXYZVYaw>(pub_global_path_topic_, 10);
}

void RandomWalkNode::vdbmapCallback(const visualization_msgs::msg::Marker &msg) {
    // updating the local voxel points and generating a path only if the path is not executing
    if (!this->received_first_map) {
        this->received_first_map = true;
        this->params.voxel_size_m= std::tuple<float, float, float>(msg.scale.x, msg.scale.y, msg.scale.z);
        this->random_walk_planner = RandomWalkPlanner(this->params);
    }
    
    if (!this->is_path_executing && this->received_first_odometry) {
        this->voxel_points.clear();
        for (int i = 0; i < msg.points.size(); i++) {
            this->random_walk_planner.voxel_points.push_back(
                std::tuple<float, float, float>(msg.points[i].x, msg.points[i].y, msg.points[i].z));
        }
        std::optional<Path> generated_path =
            this->random_walk_planner.generate_straight_rand_path(this->current_location);
        this->current_goal_location = std::tuple<float, float, float, float>(
            std::get<0>(generated_path.value().back()), std::get<1>(generated_path.value().back()),
            std::get<2>(generated_path.value().back()), std::get<3>(generated_path.value().back()));
        if (generated_path.has_value() && generated_path.value().size() > 0) {
            // publish the path
            airstack_msgs::msg::TrajectoryXYZVYaw path_msg;
            path_msg.header.stamp = this->now();
            path_msg.header.frame_id = world_frame_id_;
            for (auto point : generated_path.value()) {
                airstack_msgs::msg::WaypointXYZVYaw point_msg;
                point_msg.position.x = std::get<0>(point);
                point_msg.position.y = std::get<1>(point);
                point_msg.position.z = std::get<2>(point);
                point_msg.yaw = std::get<3>(point);
                path_msg.waypoints.push_back(point_msg);
            }
            pub_global_path->publish(path_msg);
            this->is_path_executing = true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to generate path, size was 0");
        }
    } else if (this->is_path_executing) {
        std::tuple<float, float, float> curr_loc_wo_yaw(std::get<0>(this->current_location),
                                                        std::get<1>(this->current_location),
                                                        std::get<2>(this->current_location));
        std::tuple<float, float, float> goal_loc_wo_yaw(std::get<0>(this->current_goal_location),
                                                        std::get<1>(this->current_goal_location),
                                                        std::get<2>(this->current_goal_location));
        if (get_point_distance(curr_loc_wo_yaw, goal_loc_wo_yaw) <
            this->random_walk_planner.path_end_threshold_m) {
            this->is_path_executing = false;
        }
    }
}

void RandomWalkNode::odometryCallback(const nav_msgs::msg::Odometry &msg) {
    // get the current location
    this->current_location = std::tuple<float, float, float, float>(
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z,
        msg.pose.pose.orientation.z);
    received_first_odometry = true;
}

std::optional<init_params> RandomWalkNode::readParameters() {
    // Read in parameters based off the default yaml file
    init_params params;
    if (!this->get_parameter("world_frame_id_", world_frame_id_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: world_frame_id_");
        return std::optional<init_params>{};
    } else {
        this->world_frame_id_ = world_frame_id_;
    }
    if (!this->get_parameter("pub_global_path_topic", pub_global_path_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: pub_global_path_topic");
        return std::optional<init_params>{};
    } else {
        this->pub_global_path_topic_ = pub_global_path_topic_;
    }
    if (!this->get_parameter("sub_vdb_map_topic", sub_vdb_map_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: sub_vdb_map_topic");
        return std::optional<init_params>{};
    } else {
        this->sub_vdb_map_topic_ = sub_vdb_map_topic_;
    }
    if (!this->get_parameter("sub_odometry_topic", sub_odometry_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: sub_odometry_topic");
        return std::optional<init_params>{};
    } else {
        this->sub_odometry_topic_ = sub_odometry_topic_;
    }
    float temp;
    if (!this->get_parameter("max_start_to_goal_dist_m", temp)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: max_start_to_goal_dist_m");
        return std::optional<init_params>{};
    } else {
        params.max_start_to_goal_dist_m = temp;
    }
    if (!this->get_parameter("max_angle_change_deg", temp)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: max_angle_change_deg");
        return std::optional<init_params>{};
    } else {
        params.max_angle_change_deg = temp;
    }
    if (!this->get_parameter("checking_point_cnt", temp)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: checking_point_cnt");
        return std::optional<init_params>{};
    } else {
        params.checking_point_cnt = temp;
    }
    if (!this->get_parameter("waypoint_dist_m", temp)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: waypoint_dist_m");
        return std::optional<init_params>{};
    } else {
        params.waypoint_dist_m = temp;
    }
    if (!this->get_parameter("max_z_m", temp)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: max_z_m");
        return std::optional<init_params>{};
    } else {
        params.max_z_m = temp;
    }
    if (!this->get_parameter("collision_padding_m", temp)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: collision_padding_m");
        return std::optional<init_params>{};
    } else {
        params.collision_padding_m = temp;
    }
    if (!this->get_parameter("path_end_threshold_m", temp)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: path_end_threshold_m");
        return std::optional<init_params>{};
    } else {
        params.path_end_threshold_m = temp;
    }
    return params;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RandomWalkNode>());
    rclcpp::shutdown();
    return 0;
}