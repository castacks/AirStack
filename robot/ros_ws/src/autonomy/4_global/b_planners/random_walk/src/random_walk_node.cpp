
#include "../include/random_walk_node.hpp"

#include "../include/random_walk_logic.hpp"

std::optional<init_params> RandomWalkNode::readParameters() {
    // Read in parameters based off the default yaml file
    init_params params;
    this->declare_parameter<std::string>("robot_frame_id");
    if (!this->get_parameter("robot_frame_id", this->robot_frame_id_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: robot_frame_id");
        return std::optional<init_params>{};
    }
    this->declare_parameter<std::string>("pub_global_plan_topic");
    if (!this->get_parameter("pub_global_plan_topic", this->pub_global_plan_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: pub_global_plan_topic");
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
    this->declare_parameter<float>("max_z_change_m");
    if (!this->get_parameter("max_z_change_m", params.max_z_change_m)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: max_z_change_m");
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
    this->declare_parameter<float>("max_z_angle_change_rad");
    if (!this->get_parameter("max_z_angle_change_rad", params.max_z_angle_change_rad)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: max_z_angle_change_rad");
        return std::optional<init_params>{};
    }
    return params;
}

RandomWalkNode::RandomWalkNode() : Node("random_walk_node") {
    // Initialize the random walk planner
    std::optional<init_params> params_opt = RandomWalkNode::readParameters();
    if (params_opt.has_value()) {
        this->params = params_opt.value();
    } else
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize random walk planner");

    this->sub_map = this->create_subscription<visualization_msgs::msg::Marker>(
        sub_map_topic_, 10, std::bind(&RandomWalkNode::mapCallback, this, std::placeholders::_1));

    //TF buffer and listener
    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    this->pub_global_plan = this->create_publisher<nav_msgs::msg::Path>(pub_global_plan_topic_, 10);
    this->pub_goal_point =
        this->create_publisher<visualization_msgs::msg::Marker>(pub_goal_point_viz_topic_, 10);
    this->pub_trajectory_lines =
        this->create_publisher<visualization_msgs::msg::Marker>(pub_trajectory_viz_topic_, 10);

    // Set up the timer
    this->timer = this->create_wall_timer(std::chrono::seconds(1),
                                          std::bind(&RandomWalkNode::timerCallback, this));
    // Set up the service
    this->srv_random_walk_toggle = this->create_service<std_srvs::srv::Trigger>(
        srv_random_walk_toggle_topic_, std::bind(&RandomWalkNode::randomWalkToggleCallback, this,
                                                 std::placeholders::_1, std::placeholders::_2));
    RCLCPP_INFO(this->get_logger(), "Random walk node initialized");
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
        this->world_frame_id_ = msg->header.frame_id;
        RCLCPP_INFO(this->get_logger(), "Received first map");
        this->params.voxel_size_m =
            std::tuple<float, float, float>(msg->scale.x, msg->scale.y, msg->scale.z);
        // this->random_walk_planner = RandomWalkPlanner(this->params);
        this->random_walk_planner = std::make_unique<RandomWalkPlanner>(this->params);

        RCLCPP_INFO(this->get_logger(), "Initialized random walk planner logic");
    }
    this->random_walk_planner->voxel_points.clear();
    for (int i = 0; i < msg->points.size(); i++) {
        this->random_walk_planner->voxel_points.push_back(
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

    std::tuple<float, float, float, float> start_loc;
    if (this->generated_paths.size() == 0) {
        geometry_msgs::msg::Quaternion orientation = this->current_location.rotation;
        tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
        q.normalize();
        double roll, pitch, yaw;
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        start_loc = std::make_tuple(this->current_location.translation.x,
                                    this->current_location.translation.y,
                                    this->current_location.translation.z, yaw);
    } else {
        geometry_msgs::msg::Quaternion orientation =
            this->generated_paths.back().poses.back().pose.orientation;
        tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
        q.normalize();
        double roll, pitch, yaw;
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);
        start_loc = std::make_tuple(this->generated_paths.back().poses.back().pose.position.x,
                                    this->generated_paths.back().poses.back().pose.position.y,
                                    this->generated_paths.back().poses.back().pose.position.z, yaw);
    }

    float timeout_duration = 5.0;
    std::optional<Path> gen_path_opt =
        this->random_walk_planner->generate_straight_rand_path(start_loc, timeout_duration);
    if (gen_path_opt.has_value() && gen_path_opt.value().size() > 0) {
        RCLCPP_INFO(this->get_logger(), "Generated path with %ld points",
                    gen_path_opt.value().size());

        // set the current goal location
        this->current_goal_location = geometry_msgs::msg::Transform();
        this->current_goal_location.translation.x = std::get<0>(gen_path_opt.value().back());
        this->current_goal_location.translation.y = std::get<1>(gen_path_opt.value().back());
        this->current_goal_location.translation.z = std::get<2>(gen_path_opt.value().back());
        float z_rot = std::get<3>(gen_path_opt.value().back());
        tf2::Quaternion q;
        q.setRPY(0, 0, z_rot);  // Roll = 0, Pitch = 0, Yaw = yaw
        q.normalize();
        this->current_goal_location.rotation.x = q.x();
        this->current_goal_location.rotation.y = q.y();
        this->current_goal_location.rotation.z = q.z();
        this->current_goal_location.rotation.w = q.w();

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
        geometry_msgs::msg::PoseStamped last_goal_loc = generated_single_path.poses.back();
        this->current_goal_location.translation.x = last_goal_loc.pose.position.x;
        this->current_goal_location.translation.y = last_goal_loc.pose.position.y;
        this->current_goal_location.translation.z = last_goal_loc.pose.position.z;
        this->current_goal_location.rotation.z = last_goal_loc.pose.orientation.z;
        this->generated_paths.push_back(generated_single_path);

    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to generate path, size was 0");
    }
}

void RandomWalkNode::publish_plan() {
    nav_msgs::msg::Path full_path;
    for (auto path : this->generated_paths) {
        for (auto point : path.poses) {
            full_path.poses.push_back(point);
        }
    }
    full_path.header.stamp = this->now();
    full_path.header.frame_id = this->world_frame_id_;
    this->pub_global_plan->publish(full_path);
    RCLCPP_INFO(this->get_logger(), "Published full path");
}
void RandomWalkNode::timerCallback() {
    // get current TF to world
    try {
        geometry_msgs::msg::TransformStamped transform_stamped =
            this->tf_buffer->lookupTransform(this->world_frame_id_, this->robot_frame_id_,
                                             rclcpp::Time(0));
        this->current_location = transform_stamped.transform;
        if (!this->received_first_robot_tf) {
            this->received_first_robot_tf = true;
            RCLCPP_INFO(this->get_logger(), "Received first robot_tf");
        }
    } catch (tf2::TransformException & ex) {
        RCLCPP_ERROR_ONCE(this->get_logger(), "Robot tf not yet received: %s", ex.what());
    }

    if (this->enable_random_walk && !this->is_path_executing) {
        if (this->received_first_map && this->received_first_robot_tf) {
            for (int i = 0; i < this->num_paths_to_generate_; i++) {
                this->generate_plan();
            }
            this->publish_plan();
            this->is_path_executing = true;
        } else {
            RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for map and robot tf to be received...");
        }
    } else if (this->enable_random_walk && this->is_path_executing) {
        std::tuple<float, float, float> current_point = std::make_tuple(
            this->current_location.translation.x, this->current_location.translation.y,
            this->current_location.translation.z);
        std::tuple<float, float, float> goal_point = std::make_tuple(
            this->current_goal_location.translation.x, this->current_goal_location.translation.y,
            this->current_goal_location.translation.z);
        if (get_point_distance(current_point, goal_point) <
            this->random_walk_planner->path_end_threshold_m) {
            this->is_path_executing = false;
            this->generated_paths.clear();
            RCLCPP_INFO(this->get_logger(), "Reached goal point");
        }
    }
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RandomWalkNode>());
    rclcpp::shutdown();
    return 0;
}