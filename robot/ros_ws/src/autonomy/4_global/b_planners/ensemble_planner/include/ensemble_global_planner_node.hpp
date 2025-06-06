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

#include "rclcpp/rclcpp.hpp"

class EnsembleGlobalPlannerNode : public rclcpp::Node {
   private:
    // String constants
    std::string srv_global_plan_toggle_topic_;

    void globalPlannnerToggleCallback(const std_srvs::srv::Trigger::Request::SharedPtr request,
                                      std_srvs::srv::Trigger::Response::SharedPtr response);

    // Other functions
    void readParameters();

    bool enable_global_planner = false;

   public:
    // explicit RandomWalkNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    EnsembleGlobalPlannerNode();
    ~EnsembleGlobalPlannerNode() = default;

    // ROS subscribers
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr sub_map;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_robot_tf;

    // ROS publishers
    // rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_global_path;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_goal_point;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_trajectory_lines;

    // ROS services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_global_planner_toggle;

    // ROS timers
    rclcpp::TimerBase::SharedPtr timer;
}; 
