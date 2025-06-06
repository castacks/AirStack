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
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <array>
#include <cmath>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <optional>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tuple>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>

#include "random_walk_logic.hpp"
#include "rclcpp/rclcpp.hpp"

class RandomWalkNode : public rclcpp::Node {
   private:
    // Planner
    // RandomWalkPlanner random_walk_planner;
    std::unique_ptr<RandomWalkPlanner> random_walk_planner;

    // String constants
    std::string world_frame_id_;
    std::string robot_frame_id_;
    std::string pub_global_plan_topic_;
    std::string pub_goal_point_viz_topic_;
    std::string pub_trajectory_viz_topic_;
    std::string sub_map_topic_;
    std::string sub_robot_tf_topic_;
    std::string srv_random_walk_toggle_topic_;

    // Variables
    init_params params;
    // nav_msgs::msg::Path generated_path;
    int num_paths_to_generate_;
    std::vector<nav_msgs::msg::Path> generated_paths;
    bool publish_visualizations = false;
    bool received_first_map = false;
    bool received_first_robot_tf = false;
    bool enable_random_walk = false;
    bool is_path_executing = false;

    geometry_msgs::msg::Transform current_location;       // x, y, z, yaw
    geometry_msgs::msg::Transform current_goal_location;  // x, y, z, yaw
    geometry_msgs::msg::Transform last_location;         // Last recorded position
    rclcpp::Time last_position_change;                  // Time of last position change
    double position_change_threshold = 0.1;       // Minimum distance (meters) to consider as movement
    double stall_timeout_seconds = 5.0;          // Time without movement before clearing plan

    // Callbacks
    void mapCallback(const visualization_msgs::msg::Marker::SharedPtr msg);

    void tfCallback(const tf2_msgs::msg::TFMessage::SharedPtr msg);

    void randomWalkToggleCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                  std_srvs::srv::Trigger::Response::SharedPtr response);

    void timerCallback();

    void generate_plan();

    void publish_plan();

    // Other functions
    std::optional<init_params> readParameters();

   public:
    // explicit RandomWalkNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    RandomWalkNode();
    ~RandomWalkNode() = default;

    // TF buffer
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener;

    // ROS subscribers
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr sub_map;
    // rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_robot_tf;

    // ROS publishers
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_global_plan;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_goal_point;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_trajectory_lines;

    // ROS services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_random_walk_toggle;

    // ROS timers
    rclcpp::TimerBase::SharedPtr timer;
};
