#include "../include/frontier_node.hpp"

#include "../include/frontier_logic.hpp"

std::optional<init_params> FrontierNode::readParameters(){
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
    this->declare_parameter<std::string>("srv_frontier_toggle_topic");
    if (!this->get_parameter("srv_frontier_toggle_topic", this->srv_frontier_toggle_topic_)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: srv_frontier_toggle_topic");
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
    return params;
}

FrontierNode::FrontierNode() : Node("frontier_node") {
    //initialize frontier planner
    std::optional<init_params> params_opt = FrontierNode::readParameters();
    if (params_opt.has_value()){
        this->params = params_opt.value();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize Frontier planner");
    }
    
    this->sub_map = this->create_subscription<visualization_msgs::msg::Marker>(
        sub_map_topic_, 10, std::bind(&FrontierNode::mapCallback, this, std::placeholders::_1));

    tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    this->pub_global_plan = this->create_publisher<nav_msgs::msg::Path>(pub_global_plan_topic_,10);
    this->pub_goal_point = this->create_publisher<visualization_msgs::msg::Marker>(pub_goal_point_viz_topic_, 10);
    this->pub_trajectory_lines =
        this->create_publisher<visualization_msgs::msg::Marker>(pub_trajectory_viz_topic_, 10);

    // set up the timer
    this->timer = this->create_wall_timer(std::chrono::seconds(1),std::bind(&FrontierNode::timerCallback,this));

    // Set up the service
    this->srv_frontier_toggle = this->create_service<std_srvs::srv::Trigger>(
        srv_frontier_toggle_topic_, std::bind(&FrontierNode::frontierToggleCallback, this,
                                                 std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Frontier node initialized");
}

void FrontierNode::frontierToggleCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr request,
    std_srvs::srv::Trigger::Response::SharedPtr response) {
    if (this->enable_frontier == false) {
        this->enable_frontier = true;
        response->success = true;
        response->message = "Frontier enabled";
        RCLCPP_INFO(this->get_logger(), "Frontier enabled");
    } else {
        this->enable_frontier = false;
        response->success = true;
        response->message = "Frontier disabled";
        RCLCPP_INFO(this->get_logger(), "Frontier disabled");
    }
}

void FrontierNode::mapCallback(const visualization_msgs::msg::Marker::SharedPtr msg){
    return;
}

void FrontierNode::timerCallback(){
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

    if (this->enable_frontier && !this->is_path_executing) {
        if (this->received_first_map && this->received_first_robot_tf) {
            for (int i = 0; i < this->num_paths_to_generate_; i++) {
                this->generate_plan();
            }
            this->publish_plan();
            this->is_path_executing = true;
        } else {
            RCLCPP_INFO_ONCE(this->get_logger(), "Waiting for map and robot tf to be received...");
        }
    } else if (this->enable_frontier && this->is_path_executing) {
        std::tuple<float, float, float> current_point = std::make_tuple(
            this->current_location.translation.x, this->current_location.translation.y,
            this->current_location.translation.z);
        std::tuple<float, float, float> goal_point = std::make_tuple(
            this->current_goal_location.translation.x, this->current_goal_location.translation.y,
            this->current_goal_location.translation.z);
        if (get_point_distance(current_point, goal_point) < 3) {
            this->is_path_executing = false;
            this->generated_paths.clear();
            RCLCPP_INFO(this->get_logger(), "Reached goal point");
        }
    }
}

void FrontierNode::tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg) {
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

void FrontierNode::generate_plan(){
    RCLCPP_INFO(this->get_logger(), "Starting to generate plan...");
}

void FrontierNode::publish_plan() {
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

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierNode>());
    rclcpp::shutdown();
    return 0;
}