
#include "../include/random_walk_node.hpp"

#include "../include/random_walk_logic.hpp"

RandomWalkNode::RandomWalkNode() : Node("random_walk_node") {
    // Initialize the random walk planner
    std::optional<init_params> params_opt = RandomWalkNode::readParameters();
    if (params_opt.has_value()) {
        this->params = params_opt.value();
    } else
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize random walk planner");

    this->sub_map = this->create_subscription<visualization_msgs::msg::Marker>(
        sub_map_topic_, 10, std::bind(&RandomWalkNode::mapCallback, this, std::placeholders::_1));
    this->sub_robot_tf = this->create_subscription<tf2_msgs::msg::TFMessage>(
        sub_robot_tf_topic_, 10,
        std::bind(&RandomWalkNode::tfCallback, this, std::placeholders::_1));

    this->pub_goal_point =
        this->create_publisher<visualization_msgs::msg::Marker>(pub_goal_point_viz_topic_, 10);
    this->pub_trajectory_lines =
        this->create_publisher<visualization_msgs::msg::Marker>(pub_trajectory_viz_topic_, 10);

    // Set up the timer
    this->timer = this->create_wall_timer(std::chrono::seconds(1),
                                          std::bind(&RandomWalkNode::timerCallback, this));
    // Set up the service
    this->srv_random_walk_toggle = this->create_service<std_srvs::srv::Trigger>(
        srv_random_walk_toggle_topic_,
        std::bind(&RandomWalkNode::randomWalkToggleCallback, this, std::placeholders::_1,
                  std::placeholders::_2));
}

void RandomWalkNode::randomWalkToggleCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
    if (this->enable_random_walk == false) {
        this->enable_random_walk = true;
        response->success = true;
        response->message = "Random walk enabled";
        RCLCPP_INFO(this->get_logger(), "Random walk enabled");
    } else {
        this->enable_random_walk = false;
        response->success = true;
        response->message = "Random walk disabled";
        RCLCPP_INFO(this->get_logger(), "Random walk disabled");
    }
}

void RandomWalkNode::mapCallback(const visualization_msgs::msg::Marker::SharedPtr msg) {
    // updating the local voxel points and generating a path only if the path is not executing
    if (!this->received_first_map) {
        this->received_first_map = true;
        RCLCPP_INFO(this->get_logger(), "Received first map");
        this->params.voxel_size_m =
            std::tuple<float, float, float>(msg->scale.x, msg->scale.y, msg->scale.z);
        this->random_walk_planner = RandomWalkPlanner(this->params);
        RCLCPP_INFO(this->get_logger(), "Initialized random walk planner");
    }
    this->random_walk_planner.voxel_points.clear();
    for (int i = 0; i < msg->points.size(); i++) {
        this->random_walk_planner.voxel_points.push_back(
            std::tuple<float, float, float>(msg->points[i].x, msg->points[i].y, msg->points[i].z));
    }
}

void RandomWalkNode::tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
    // get the current location
    for (int i = 0; i < msg->transforms.size(); i++) {
        if (msg->transforms[i].child_frame_id.c_str() == this->robot_frame_id_) {
            this->current_location = msg->transforms[i].transform;
            if (!this->received_first_robot_tf) {
                this->received_first_robot_tf = true;
                RCLCPP_INFO(this->get_logger(), "Received first robot_tf");
            }
        }
    }
}

void RandomWalkNode::generate_plan() {
    RCLCPP_INFO(this->get_logger(), "Starting to generate plan...");
    geometry_msgs::msg::Quaternion curr_orientation;
    curr_orientation = this->current_location.rotation;

    tf2::Quaternion curr_orientation_quat(curr_orientation.x, curr_orientation.y,
                                          curr_orientation.z, curr_orientation.w);
    tf2::Matrix3x3 m(curr_orientation_quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    std::tuple<float, float, float, float> start_loc;
    if (this->generated_paths.size() == 0) {
        start_loc = std::make_tuple(this->current_location.translation.x,
                                                         this->current_location.translation.y,
                                                         this->current_location.translation.z, yaw);
    } else {
        start_loc = std::make_tuple(
            this->generated_paths.back().poses.back().pose.position.x,
            this->generated_paths.back().poses.back().pose.position.y,
            this->generated_paths.back().poses.back().pose.position.z,
            this->generated_paths.back().poses.back().pose.orientation.z);
    }

    float timeout_duration = 5.0;
    std::optional<Path> gen_path_opt =
        this->random_walk_planner.generate_straight_rand_path(start_loc, timeout_duration);
    if (gen_path_opt.has_value() && gen_path_opt.value().size() > 0) {
        RCLCPP_INFO(this->get_logger(), "Generated path with %ld points",
                    gen_path_opt.value().size());

        // set the current goal location
        this->current_goal_location = geometry_msgs::msg::Transform();
        this->current_goal_location.translation.x = std::get<0>(gen_path_opt.value().back());
        this->current_goal_location.translation.y = std::get<1>(gen_path_opt.value().back());
        this->current_goal_location.translation.z = std::get<2>(gen_path_opt.value().back());
        this->current_goal_location.rotation.z = std::get<3>(gen_path_opt.value().back());

        // publish the path
        nav_msgs::msg::Path generated_single_path;
        generated_single_path = nav_msgs::msg::Path();
        generated_single_path.header.stamp = this->now();
        generated_single_path.header.frame_id = world_frame_id_;
        for (auto point : gen_path_opt.value()) {
            geometry_msgs::msg::PoseStamped point_msg;
            point_msg.pose.position.x = std::get<0>(point);
            point_msg.pose.position.y = std::get<1>(point);
            point_msg.pose.position.z = std::get<2>(point);
            // convert yaw rotation to quaternion
            tf2::Quaternion q;
            q.setRPY(0, 0, std::get<3>(point));  // Roll = 0, Pitch = 0, Yaw = yaw
            q.normalize();
            point_msg.pose.orientation.x = q.x();
            point_msg.pose.orientation.y = q.y();
            point_msg.pose.orientation.z = q.z();
            point_msg.pose.orientation.w = q.w();
            point_msg.header.stamp = this->now();
            generated_single_path.poses.push_back(point_msg);
        }
        // this->pub_global_path->publish(this->generated_path);
        if (this->publish_visualizations) {
            this->pub_goal_point->publish(createGoalPointMarker());
            this->pub_trajectory_lines->publish(createTrajectoryLineMarker());
        }
        // RCLCPP_INFO(this->get_logger(), "Published path and goal point at %f, %f, %f",
        //             this->generated_path.poses.back().pose.position.x,
        //             this->generated_path.poses.back().pose.position.y,
        //             this->generated_path.poses.back().pose.position.z);
        this->generated_paths.push_back(generated_single_path);

    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to generate path, size was 0");
    }
}

void RandomWalkNode::timerCallback() {
    if (this->enable_random_walk) {
        if (this->received_first_map && this->received_first_robot_tf) {
            this->generate_plan();
        } else {
            RCLCPP_INFO(this->get_logger(), "Waiting for map and robot tf to be received...");
        }
    }
    // if (this->publish_visualizations && this->generated_path.poses.size() > 0) {
    //     visualization_msgs::msg::Marker goal_point_msg = createGoalPointMarker();
    //     this->pub_goal_point->publish(goal_point_msg);
    //     visualization_msgs::msg::Marker trajectory_line_msg = createTrajectoryLineMarker();
    //     this->pub_trajectory_lines->publish(trajectory_line_msg);
    //     RCLCPP_INFO(this->get_logger(), "Published goal point and trajectory line");
    // }
}

std::optional<init_params> RandomWalkNode::readParameters() {
    // Read in parameters based off the default yaml file
    init_params params;
    this->declare_parameter<std::string>("world_frame_id");
    if (!this->get_parameter("world_frame_id", this->world_frame_id_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: world_frame_id");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("robot_frame_id");
    if (!this->get_parameter("robot_frame_id", this->robot_frame_id_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: robot_frame_id");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("pub_global_trajectory_topic");
    if (!this->get_parameter("pub_global_trajectory_topic", this->pub_global_trajectory_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: pub_global_trajectory_topic");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("pub_goal_point_viz_topic");
    if (!this->get_parameter("pub_goal_point_viz_topic", this->pub_goal_point_viz_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: pub_goal_point_viz_topic");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("pub_trajectory_viz_topic");
    if (!this->get_parameter("pub_trajectory_viz_topic", this->pub_trajectory_viz_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: pub_trajectory_viz_topic");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("sub_map_topic");
    if (!this->get_parameter("sub_map_topic", this->sub_map_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: sub_map_topic");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("sub_robot_tf_topic");
    if (!this->get_parameter("sub_robot_tf_topic", this->sub_robot_tf_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: sub_robot_tf_topic");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("srv_random_walk_toggle_topic");
    if (!this->get_parameter("srv_random_walk_toggle_topic", this->srv_random_walk_toggle_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: srv_random_walk_toggle_topic");
        return std::optional<init_params>{};
    }
    this->declare_parameter<bool>("publish_visualizations");
    if (!this->get_parameter("publish_visualizations", this->publish_visualizations)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: publish_visualizations");
        return std::optional<init_params>{};
    }
    this->declare_parameter<int>("num_paths_to_generate");
    if (!this->get_parameter("num_paths_to_generate", this->num_paths_to_generate_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: num_paths_to_generate");
        return std::optional<init_params>{};
    }
    this->declare_parameter<float>("max_start_to_goal_dist_m");
    if (!this->get_parameter("max_start_to_goal_dist_m", params.max_start_to_goal_dist_m)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: max_start_to_goal_dist_m");
        return std::optional<init_params>{};
    }
    this->declare_parameter<int>("checking_point_cnt");
    if (!this->get_parameter("checking_point_cnt", params.checking_point_cnt)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: checking_point_cnt");
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

    for (auto path : this->generated_paths) {
        for (auto point : path.poses) {
            geometry_msgs::msg::Point p;
            p.x = point.pose.position.x;
            p.y = point.pose.position.y;
            p.z = point.pose.position.z;
            trajectory_line_msg.points.push_back(p);
        }
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
    goal_point.x = this->current_goal_location.translation.x;
    goal_point.y = this->current_goal_location.translation.y;
    goal_point.z = this->current_goal_location.translation.z;
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