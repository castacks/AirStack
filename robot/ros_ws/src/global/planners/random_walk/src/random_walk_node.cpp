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

#include "../include/random_walk_node.hpp"

#include "../include/random_walk_logic.hpp"

std::optional<init_params> RandomWalkNode::readParameters() {
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
    this->declare_parameter<float>("max_yaw_change_degrees");
    if (!this->get_parameter("max_yaw_change_degrees", params.max_yaw_change_degrees)) {
        RCLCPP_ERROR(this->get_logger(), "Cannot read parameter: max_yaw_change_degrees");
        return std::optional<init_params>{};
    }
    return params;
}

RandomWalkNode::RandomWalkNode() : Node("random_walk_node") {
    std::optional<init_params> params_opt = RandomWalkNode::readParameters();
    if (params_opt.has_value()) {
        this->params = params_opt.value();
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize random walk planner");
    }

    this->sub_map = this->create_subscription<visualization_msgs::msg::Marker>(
        sub_map_topic_, 10, std::bind(&RandomWalkNode::mapCallback, this, std::placeholders::_1));

    this->sub_odometry = this->create_subscription<nav_msgs::msg::Odometry>(
        sub_odometry_topic_, 10,
        std::bind(&RandomWalkNode::odometryCallback, this, std::placeholders::_1));

    this->pub_global_plan = this->create_publisher<nav_msgs::msg::Path>(pub_global_plan_topic_, 10);
    this->pub_goal_point =
        this->create_publisher<visualization_msgs::msg::Marker>(pub_goal_point_viz_topic_, 10);
    this->pub_trajectory_lines =
        this->create_publisher<visualization_msgs::msg::Marker>(pub_trajectory_viz_topic_, 10);

    this->navigate_client_ = rclcpp_action::create_client<NavigateTask>(this, "navigate_task");

    this->action_server_ = rclcpp_action::create_server<ExplorationTask>(
        this, "~/exploration_task",
        std::bind(&RandomWalkNode::handle_goal, this, std::placeholders::_1,
                  std::placeholders::_2),
        std::bind(&RandomWalkNode::handle_cancel, this, std::placeholders::_1),
        std::bind(&RandomWalkNode::handle_accepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Random walk node initialized, waiting for ExplorationTask goals");
}

// ---------------------------------------------------------------------------
// Subscriber callbacks
// ---------------------------------------------------------------------------

void RandomWalkNode::mapCallback(const visualization_msgs::msg::Marker::SharedPtr msg) {
    if (!this->received_first_map) {
        this->received_first_map = true;
        this->world_frame_id_ = msg->header.frame_id;
        this->params.voxel_size_m =
            std::tuple<float, float, float>(msg->scale.x, msg->scale.y, msg->scale.z);
        this->random_walk_planner = std::make_unique<RandomWalkPlanner>(this->params);
        RCLCPP_INFO(this->get_logger(), "Received first map, initialized planner");
    }
    this->random_walk_planner->voxel_points.clear();
    for (size_t i = 0; i < msg->points.size(); i++) {
        this->random_walk_planner->voxel_points.push_back(
            std::tuple<float, float, float>(msg->points[i].x, msg->points[i].y, msg->points[i].z));
    }
}

void RandomWalkNode::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    this->current_location = msg->pose.pose;
    if (!this->received_first_robot_tf) {
        this->received_first_robot_tf = true;
        RCLCPP_INFO(this->get_logger(), "Received first odometry");
    }
}

// ---------------------------------------------------------------------------
// Action server callbacks
// ---------------------------------------------------------------------------

rclcpp_action::GoalResponse RandomWalkNode::handle_goal(
    const rclcpp_action::GoalUUID&,
    std::shared_ptr<const ExplorationTask::Goal> goal) {
    if (task_active_) {
        RCLCPP_WARN(this->get_logger(), "Rejecting goal: a task is already active");
        return rclcpp_action::GoalResponse::REJECT;
    }
    task_active_ = true;
    RCLCPP_INFO(this->get_logger(),
                "Received ExplorationTask goal: alt=[%.1f,%.1f]m AGL, "
                "speed=[%.1f,%.1f]m/s, time_limit=%.1fs",
                goal->min_altitude_agl, goal->max_altitude_agl, goal->min_flight_speed,
                goal->max_flight_speed, goal->time_limit_sec);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RandomWalkNode::handle_cancel(
    std::shared_ptr<GoalHandle> /*goal_handle*/) {
    RCLCPP_INFO(this->get_logger(), "Received cancel request for ExplorationTask");
    cancel_requested_ = true;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void RandomWalkNode::handle_accepted(std::shared_ptr<GoalHandle> goal_handle) {
    std::thread{std::bind(&RandomWalkNode::execute, this, std::placeholders::_1), goal_handle}
        .detach();
}

// ---------------------------------------------------------------------------
// Task execution (runs in its own thread)
// ---------------------------------------------------------------------------

void RandomWalkNode::execute(std::shared_ptr<GoalHandle> goal_handle) {
    const auto goal = goal_handle->get_goal();

    // TODO: pass min/max altitude AGL and flight speed bounds into RandomWalkPlanner
    // Currently the planner uses fixed bounds from config; these goal params are logged
    // and will be wired in when the planner supports them.
    RCLCPP_INFO(this->get_logger(),
                "ExplorationTask executing: alt=[%.1f,%.1f]m AGL, "
                "speed=[%.1f,%.1f]m/s, time_limit=%.1fs",
                goal->min_altitude_agl, goal->max_altitude_agl, goal->min_flight_speed,
                goal->max_flight_speed, goal->time_limit_sec);

    task_start_time_ = this->now();
    task_time_limit_sec_ = goal->time_limit_sec;
    cancel_requested_ = false;

    // Reset planning state for this task
    is_path_executing = false;
    generated_paths.clear();

    rclcpp::Rate rate(1.0);  // 1 Hz planning loop

    while (rclcpp::ok()) {
        // --- Check for cancellation ---
        if (cancel_requested_) {
            // Cancel any active NavigateTask goal
            if (navigate_goal_handle_ && !navigate_goal_done_)
                navigate_client_->async_cancel_goal(navigate_goal_handle_);
            auto result = std::make_shared<ExplorationTask::Result>();
            result->success = false;
            result->message = "Task cancelled";
            task_active_ = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "ExplorationTask cancelled");
            return;
        }

        // --- Check time limit ---
        if (task_time_limit_sec_ > 0.0f) {
            double elapsed = (this->now() - task_start_time_).seconds();
            if (elapsed >= static_cast<double>(task_time_limit_sec_)) {
                if (navigate_goal_handle_ && !navigate_goal_done_)
                    navigate_client_->async_cancel_goal(navigate_goal_handle_);
                auto result = std::make_shared<ExplorationTask::Result>();
                result->success = true;
                result->message = "Time limit reached";
                task_active_ = false;
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "ExplorationTask succeeded (time limit reached)");
                return;
            }
        }

        // --- Publish feedback ---
        auto feedback = std::make_shared<ExplorationTask::Feedback>();
        feedback->current_position.x = current_location.position.x;
        feedback->current_position.y = current_location.position.y;
        feedback->current_position.z = current_location.position.z;
        if (task_time_limit_sec_ > 0.0f) {
            double elapsed = (this->now() - task_start_time_).seconds();
            feedback->progress = static_cast<float>(elapsed / task_time_limit_sec_);
        } else {
            feedback->progress = 0.0f;
        }
        feedback->status = is_path_executing ? "navigating" : "planning";
        goal_handle->publish_feedback(feedback);

        // --- Planning / navigation loop ---
        if (!is_path_executing) {
            if (received_first_map && received_first_robot_tf) {
                generated_paths.clear();
                for (int i = 0; i < num_paths_to_generate_; i++) {
                    generate_plan();
                }
                send_navigate_goal();
                is_path_executing = true;
            } else {
                RCLCPP_INFO_ONCE(this->get_logger(),
                                 "Waiting for map and odometry before planning...");
            }
        } else if (navigate_goal_done_) {
            // NavigateTask completed (success or failure) — generate next segment
            is_path_executing = false;
            if (!navigate_goal_succeeded_) {
                RCLCPP_WARN(this->get_logger(),
                            "NavigateTask did not succeed, replanning");
            } else {
                RCLCPP_INFO(this->get_logger(), "Waypoint reached, planning next segment");
            }
        }

        rate.sleep();
    }

    // Node is shutting down
    if (navigate_goal_handle_ && !navigate_goal_done_)
        navigate_client_->async_cancel_goal(navigate_goal_handle_);
    auto result = std::make_shared<ExplorationTask::Result>();
    result->success = false;
    result->message = "Node shutting down";
    task_active_ = false;
    goal_handle->abort(result);
}

// ---------------------------------------------------------------------------
// Planning helpers (unchanged from original)
// ---------------------------------------------------------------------------

void RandomWalkNode::generate_plan() {
    RCLCPP_INFO(this->get_logger(), "Generating plan...");

    std::tuple<float, float, float, float> start_loc;
    if (this->generated_paths.empty()) {
        geometry_msgs::msg::Quaternion orientation = this->current_location.orientation;
        tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
        q.normalize();
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        start_loc = std::make_tuple(this->current_location.position.x,
                                    this->current_location.position.y,
                                    this->current_location.position.z, static_cast<float>(yaw));
    } else {
        geometry_msgs::msg::Quaternion orientation =
            this->generated_paths.back().poses.back().pose.orientation;
        tf2::Quaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
        q.normalize();
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        start_loc = std::make_tuple(this->generated_paths.back().poses.back().pose.position.x,
                                    this->generated_paths.back().poses.back().pose.position.y,
                                    this->generated_paths.back().poses.back().pose.position.z,
                                    static_cast<float>(yaw));
    }

    float timeout_duration = 5.0;
    std::optional<Path> gen_path_opt =
        this->random_walk_planner->generate_straight_rand_path(start_loc, timeout_duration);
    if (gen_path_opt.has_value() && !gen_path_opt.value().empty()) {
        RCLCPP_INFO(this->get_logger(), "Generated path with %ld points",
                    gen_path_opt.value().size());

        nav_msgs::msg::Path generated_single_path;
        generated_single_path.header.stamp = this->now();
        generated_single_path.header.frame_id = world_frame_id_;
        for (auto point : gen_path_opt.value()) {
            geometry_msgs::msg::PoseStamped point_msg;
            point_msg.pose.position.x = std::get<0>(point);
            point_msg.pose.position.y = std::get<1>(point);
            point_msg.pose.position.z = std::get<2>(point);
            tf2::Quaternion q;
            q.setRPY(0, 0, std::get<3>(point));
            q.normalize();
            point_msg.pose.orientation.x = q.x();
            point_msg.pose.orientation.y = q.y();
            point_msg.pose.orientation.z = q.z();
            point_msg.pose.orientation.w = q.w();
            point_msg.header.stamp = this->now();
            generated_single_path.poses.push_back(point_msg);
        }

        this->generated_paths.push_back(generated_single_path);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to generate path");
    }
}

void RandomWalkNode::send_navigate_goal() {
    if (!navigate_client_->wait_for_action_server(std::chrono::seconds(2))) {
        RCLCPP_WARN(this->get_logger(), "NavigateTask action server not available");
        is_path_executing = false;
        return;
    }

    // Concatenate generated path segments
    nav_msgs::msg::Path full_path;
    full_path.header.stamp = this->now();
    full_path.header.frame_id = this->world_frame_id_;
    for (auto& path : this->generated_paths) {
        for (auto& pose : path.poses) {
            full_path.poses.push_back(pose);
        }
    }

    auto nav_goal = NavigateTask::Goal();
    nav_goal.global_plan = full_path;
    nav_goal.goal_tolerance_m = static_cast<float>(random_walk_planner->path_end_threshold_m);

    navigate_goal_done_ = false;
    navigate_goal_succeeded_ = false;

    auto goal_options = rclcpp_action::Client<NavigateTask>::SendGoalOptions();
    goal_options.goal_response_callback =
        [this](const rclcpp_action::ClientGoalHandle<NavigateTask>::SharedPtr& gh) {
            navigate_goal_handle_ = gh;
            if (!gh) {
                RCLCPP_WARN(this->get_logger(), "NavigateTask goal rejected by server");
                navigate_goal_done_ = true;
            }
        };
    goal_options.result_callback =
        [this](const rclcpp_action::ClientGoalHandle<NavigateTask>::WrappedResult& result) {
            navigate_goal_succeeded_ =
                (result.code == rclcpp_action::ResultCode::SUCCEEDED) && result.result->success;
            navigate_goal_done_ = true;
        };

    navigate_client_->async_send_goal(nav_goal, goal_options);

    RCLCPP_INFO(this->get_logger(), "Sent NavigateTask goal (%zu waypoints)",
                full_path.poses.size());
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RandomWalkNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
