
#include "../include/random_walk_node.hpp"

#include "../include/random_walk_logic.hpp"

RandomWalkNode::RandomWalkNode() : Node("random_walk_node") {
    std::optional<init_params> params_opt = RandomWalkNode::readParameters();
    // Initialize the random walk planner
    if (params_opt.has_value()) {
        this->params = params_opt.value();
    } else
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize random walk planner");

    this->sub_vdb_map = this->create_subscription<visualization_msgs::msg::Marker>(
        sub_vdb_map_topic_, 10,
        std::bind(&RandomWalkNode::vdbmapCallback, this, std::placeholders::_1));
    this->sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
        sub_odometry_topic_, 10,
        std::bind(&RandomWalkNode::odometryCallback, this, std::placeholders::_1));

    this->pub_global_path =
        this->create_publisher<airstack_msgs::msg::TrajectoryXYZVYaw>(pub_global_path_topic_, 10);
    this->pub_goal_point =
        this->create_publisher<visualization_msgs::msg::Marker>(pub_goal_point_topic_, 10);
    this->pub_trajectory_lines =
        this->create_publisher<visualization_msgs::msg::Marker>(pub_trajectory_lines_topic_, 10);
    this->timer = this->create_wall_timer(std::chrono::seconds(1),
                                          std::bind(&RandomWalkNode::timerCallback, this));
}

void RandomWalkNode::vdbmapCallback(const visualization_msgs::msg::Marker::SharedPtr msg) {
    // updating the local voxel points and generating a path only if the path is not executing
    if (!this->received_first_map) {
        this->received_first_map = true;
        RCLCPP_INFO(this->get_logger(), "Received first map");
        this->params.voxel_size_m =
            std::tuple<float, float, float>(msg->scale.x, msg->scale.y, msg->scale.z);
        this->random_walk_planner = RandomWalkPlanner(this->params);
        RCLCPP_INFO(this->get_logger(), "Initialized random walk planner");
    }
    // if path is not executing and the odometry has been received
    if (!this->is_path_executing && this->received_first_odometry) {
        RCLCPP_INFO(this->get_logger(), "Setting Voxel Points from message");
        this->voxel_points.clear();
        for (int i = 0; i < msg->points.size(); i++) {
            this->random_walk_planner.voxel_points.push_back(std::tuple<float, float, float>(
                msg->points[i].x, msg->points[i].y, msg->points[i].z));
        }
        RCLCPP_INFO(this->get_logger(), "Generating Path...");
        std::optional<Path> gen_path_opt =
            this->random_walk_planner.generate_straight_rand_path(this->current_location);
        if (gen_path_opt.has_value() && gen_path_opt.value().size() > 0) {
            RCLCPP_INFO(this->get_logger(), "Generated path with size %d",
                        gen_path_opt.value().size());

            this->current_goal_location = std::tuple<float, float, float, float>(
                std::get<0>(gen_path_opt.value().back()), std::get<1>(gen_path_opt.value().back()),
                std::get<2>(gen_path_opt.value().back()), std::get<3>(gen_path_opt.value().back()));
            // publish the path
            this->generated_path = airstack_msgs::msg::TrajectoryXYZVYaw();
            this->generated_path.header.stamp = this->now();
            this->generated_path.header.frame_id = world_frame_id_;
            for (auto point : gen_path_opt.value()) {
                airstack_msgs::msg::WaypointXYZVYaw point_msg;
                point_msg.position.x = std::get<0>(point);
                point_msg.position.y = std::get<1>(point);
                point_msg.position.z = std::get<2>(point);
                point_msg.yaw = std::get<3>(point);
                this->generated_path.waypoints.push_back(point_msg);
            }
            if (this->publish_visualizations) {
                this->pub_global_path->publish(this->generated_path);
                this->pub_goal_point->publish(createGoalPointMarker());
                this->pub_trajectory_lines->publish(createTrajectoryLineMarker());
            }
            RCLCPP_INFO(this->get_logger(), "Published path and goal point at %f, %f, %f",
                        this->generated_path.waypoints.back().position.x,
                        this->generated_path.waypoints.back().position.y,
                        this->generated_path.waypoints.back().position.z);
            this->is_path_executing = true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to generate path, size was 0");
        }
    }
    // check if the path is executing
    else if (this->is_path_executing) {
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

void RandomWalkNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // get the current location
    this->current_location = std::tuple<float, float, float, float>(
        msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z,
        msg->pose.pose.orientation.z);
    if (!this->received_first_odometry) {
        this->received_first_odometry = true;
        RCLCPP_INFO(this->get_logger(), "Received first odometry");
    }
}

void RandomWalkNode::timerCallback() {
    if (this->is_path_executing && this->publish_visualizations) {
        RCLCPP_INFO(this->get_logger(), "Publishing visualizations...");
        visualization_msgs::msg::Marker goal_point_msg = createGoalPointMarker();
        this->pub_goal_point->publish(goal_point_msg);

        visualization_msgs::msg::Marker trajectory_line_msg = createTrajectoryLineMarker();
        this->pub_trajectory_lines->publish(trajectory_line_msg);
    }
}

std::optional<init_params> RandomWalkNode::readParameters() {
    // Read in parameters based off the default yaml file
    init_params params;
    this->declare_parameter<std::string>("world_frame_id");
    if (!this->get_parameter("world_frame_id", this->world_frame_id_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: world_frame_id");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("pub_global_path_topic");
    if (!this->get_parameter("pub_global_path_topic", this->pub_global_path_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: pub_global_path_topic");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("pub_goal_point_topic");
    if (!this->get_parameter("pub_goal_point_topic", this->pub_goal_point_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: pub_goal_point_topic");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("pub_trajectory_lines_topic");
    if (!this->get_parameter("pub_trajectory_lines_topic", this->pub_trajectory_lines_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: pub_trajectory_lines_topic");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("sub_vdb_map_topic");
    if (!this->get_parameter("sub_vdb_map_topic", this->sub_vdb_map_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: sub_vdb_map_topic");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("sub_odometry_topic");
    if (!this->get_parameter("sub_odometry_topic", this->sub_odometry_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: sub_odometry_topic");
        return std::optional<init_params>{};
    }
    this->declare_parameter<bool>("publish_visualizations");
    if (!this->get_parameter("publish_visualizations", this->publish_visualizations)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: publish_visualizations");
        return std::optional<init_params>{};
    }
    this->declare_parameter<float>("max_start_to_goal_dist_m");
    if (!this->get_parameter("max_start_to_goal_dist_m", params.max_start_to_goal_dist_m)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: max_start_to_goal_dist_m");
        return std::optional<init_params>{};
    }
    this->declare_parameter<float>("max_angle_change_deg");
    if (!this->get_parameter("max_angle_change_deg", params.max_angle_change_deg)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: max_angle_change_deg");
        return std::optional<init_params>{};
    }
    this->declare_parameter<int>("checking_point_cnt");
    if (!this->get_parameter("checking_point_cnt", params.checking_point_cnt)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: checking_point_cnt");
        return std::optional<init_params>{};
    }
    this->declare_parameter<float>("waypoint_dist_m");
    if (!this->get_parameter("waypoint_dist_m", params.waypoint_dist_m)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: waypoint_dist_m");
        return std::optional<init_params>{};
    }
    this->declare_parameter<float>("max_z_m");
    if (!this->get_parameter("max_z_m", params.max_z_m)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: max_z_m");
        return std::optional<init_params>{};
    }
    this->declare_parameter<float>("collision_padding_m");
    if (!this->get_parameter("collision_padding_m", params.collision_padding_m)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: collision_padding_m");
        return std::optional<init_params>{};
    }
    this->declare_parameter<float>("path_end_threshold_m");
    if (!this->get_parameter("path_end_threshold_m", params.path_end_threshold_m)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: path_end_threshold_m");
        return std::optional<init_params>{};
    }
    return params;
}

visualization_msgs::msg::Marker RandomWalkNode::createTrajectoryLineMarker() {
    visualization_msgs::msg::Marker trajectory_line_msg;
    trajectory_line_msg.header.frame_id = this->world_frame_id_;
    trajectory_line_msg.header.stamp = this->now();
    trajectory_line_msg.ns = "trajectory_line";
    trajectory_line_msg.id = 0;
    trajectory_line_msg.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trajectory_line_msg.action = visualization_msgs::msg::Marker::ADD;
    trajectory_line_msg.pose.orientation.w = 1.0;
    trajectory_line_msg.scale.x = 0.1;
    trajectory_line_msg.color.a = 1.0;
    trajectory_line_msg.color.r = 1.0;
    trajectory_line_msg.color.g = 0.0;
    trajectory_line_msg.color.b = 0.0;
    if (this->generated_path.waypoints.size() > 0) {
        // RCLCPP_INFO(this->get_logger(), "Creating trajectory line with %d points...",
        //             this->generated_path.waypoints.size());
        for (auto point : this->generated_path.waypoints) {
            geometry_msgs::msg::Point p;
            p.x = point.position.x;
            p.y = point.position.y;
            p.z = point.position.z;
            trajectory_line_msg.points.push_back(p);
        }
    } else {
        RCLCPP_ERROR(this->get_logger(), "No points in the path");
    }
    return trajectory_line_msg;
}

visualization_msgs::msg::Marker RandomWalkNode::createGoalPointMarker() {
    visualization_msgs::msg::Marker goal_point_msg;
    goal_point_msg.header.frame_id = this->world_frame_id_;
    goal_point_msg.header.stamp = this->now();
    goal_point_msg.ns = "goal_point";
    goal_point_msg.id = 0;
    goal_point_msg.type = visualization_msgs::msg::Marker::SPHERE;
    goal_point_msg.action = visualization_msgs::msg::Marker::ADD;
    geometry_msgs::msg::Point goal_point = geometry_msgs::msg::Point();
    goal_point.x = std::get<0>(this->current_goal_location);
    goal_point.y = std::get<1>(this->current_goal_location);
    goal_point.z = std::get<2>(this->current_goal_location);
    goal_point_msg.pose.position = goal_point;
    goal_point_msg.scale.x = 0.1;
    goal_point_msg.scale.y = 0.1;
    goal_point_msg.scale.z = 0.1;
    goal_point_msg.color.a = 1.0;
    goal_point_msg.color.r = 0.0;
    goal_point_msg.color.g = 1.0;
    goal_point_msg.color.b = 0.0;
    return goal_point_msg;
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RandomWalkNode>());
    rclcpp::shutdown();
    return 0;
}