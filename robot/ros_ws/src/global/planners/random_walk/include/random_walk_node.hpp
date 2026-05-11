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

#pragma once

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <atomic>
#include <cmath>
#include <mutex>
#include <utility>
#include <vector>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <optional>
#include <string>
#include <thread>
#include <tuple>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "random_walk_logic.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "task_msgs/action/exploration_task.hpp"
#include "task_msgs/action/navigate_task.hpp"

class RandomWalkNode : public rclcpp::Node {
   public:
    using ExplorationTask = task_msgs::action::ExplorationTask;
    using GoalHandle = rclcpp_action::ServerGoalHandle<ExplorationTask>;
    using NavigateTask = task_msgs::action::NavigateTask;

    RandomWalkNode();
    ~RandomWalkNode() = default;

    // ROS subscribers
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr sub_map;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odometry;

    // ROS publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_global_plan;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_goal_point;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_trajectory_lines;

    // ROS action server (ExplorationTask)
    rclcpp_action::Server<ExplorationTask>::SharedPtr action_server_;

    // ROS action client (NavigateTask → local planner)
    rclcpp_action::Client<NavigateTask>::SharedPtr navigate_client_;
    rclcpp_action::ClientGoalHandle<NavigateTask>::SharedPtr navigate_goal_handle_;
    std::atomic<bool> navigate_goal_done_{true};
    std::atomic<bool> navigate_goal_succeeded_{false};

   private:
    // Planner
    std::unique_ptr<RandomWalkPlanner> random_walk_planner;

    // Topic name parameters
    std::string world_frame_id_;
    std::string robot_frame_id_;
    std::string pub_global_plan_topic_;
    std::string pub_goal_point_viz_topic_;
    std::string pub_trajectory_viz_topic_;
    std::string sub_map_topic_;
    std::string sub_odometry_topic_;

    // Configuration parameters
    init_params params;
    int num_paths_to_generate_;
    bool publish_visualizations = false;

    // Planning state
    std::vector<nav_msgs::msg::Path> generated_paths;
    bool received_first_map = false;
    bool received_first_robot_tf = false;
    bool is_path_executing = false;

    geometry_msgs::msg::Pose current_location;

    // Active task state
    std::atomic<bool> task_active_{false};
    std::atomic<bool> cancel_requested_{false};
    rclcpp::Time task_start_time_;
    float task_time_limit_sec_ = 0.0f;

    // Latest requested search bounds (XY polygon in robot-local map). Stored
    // here so we can apply them either when execute() begins or, if the
    // planner doesn't exist yet, the moment it's constructed in mapCallback.
    std::vector<std::pair<float, float>> pending_bounds_;
    std::mutex pending_bounds_mutex_;

    // Subscriber callbacks
    void mapCallback(const visualization_msgs::msg::Marker::SharedPtr msg);
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);

    // Action server callbacks
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid,
                                            std::shared_ptr<const ExplorationTask::Goal> goal);
    rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle> goal_handle);
    void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);
    void execute(std::shared_ptr<GoalHandle> goal_handle);

    // Planning helpers
    void generate_plan();
    void send_navigate_goal();

    std::optional<init_params> readParameters();
};
