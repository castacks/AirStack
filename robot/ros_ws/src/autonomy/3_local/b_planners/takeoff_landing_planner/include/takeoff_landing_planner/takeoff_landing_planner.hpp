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

#include <airstack_common/tflib.hpp>
#include <airstack_msgs/msg/odometry.hpp>
#include <airstack_msgs/msg/trajectory_xyzv_yaw.hpp>
#include <airstack_msgs/srv/takeoff_landing_command.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>
#include <trajectory_library/trajectory_library.hpp>

class TakeoffLandingPlanner : public rclcpp::Node
{
private:
  // parameters
  float takeoff_height, high_takeoff_height, takeoff_velocity, landing_velocity;
  float takeoff_acceptance_distance, takeoff_acceptance_time;
  float landing_stationary_distance, landing_acceptance_time;
  float landing_tracking_point_ahead_time;
  float takeoff_path_roll, takeoff_path_pitch;
  bool takeoff_path_relative_to_orientation;

  // variables
  bool got_completion_percentage, is_tracking_point_received, got_robot_odom;
  nav_msgs::msg::Odometry robot_odom;
  airstack_msgs::msg::Odometry tracking_point_odom;
  float completion_percentage;
  // airstack_msgs::srv::TrajectoryMode track_mode_srv;
  uint8_t current_command;

  // takeoff variables
  bool takeoff_is_newly_active;
  bool takeoff_distance_check;
  bool ekf_active;
  rclcpp::Time takeoff_acceptance_start;
  bool high_takeoff;
  TakeoffTrajectory * takeoff_traj_gen;
  TakeoffTrajectory * high_takeoff_traj_gen;

  // land variables
  bool land_is_newly_active;
  std::list<nav_msgs::msg::Odometry> robot_odoms;
  TakeoffTrajectory * landing_traj_gen;

  // subscribers
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr completion_percentage_sub;
  rclcpp::Subscription<airstack_msgs::msg::Odometry>::SharedPtr tracking_point_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_odom_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ekf_active_sub;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr high_takeoff_sub;
  tf2_ros::Buffer * tf_buffer;
  tf2_ros::TransformListener * tf_listener;

  // publishers
  rclcpp::Publisher<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr traj_override_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr takeoff_state_pub;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr landing_state_pub;

  // services
  rclcpp::CallbackGroup::SharedPtr service_callback_group;
  rclcpp::Service<airstack_msgs::srv::TakeoffLandingCommand>::SharedPtr command_server;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr ardupilot_takeoff_server;
  rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client;

  // timers
  rclcpp::TimerBase::SharedPtr timer;

  // callbacks
  void completion_percentage_callback(std_msgs::msg::Float32::SharedPtr msg);
  void tracking_point_callback(airstack_msgs::msg::Odometry::SharedPtr msg);
  void robot_odom_callback(nav_msgs::msg::Odometry::SharedPtr msg);
  void ekf_active_callback(std_msgs::msg::Bool::SharedPtr msg);
  void high_takeoff_callback(std_msgs::msg::Bool::SharedPtr msg);

  void set_takeoff_landing_command(
    const airstack_msgs::srv::TakeoffLandingCommand::Request::SharedPtr request,
    airstack_msgs::srv::TakeoffLandingCommand::Response::SharedPtr response);
  void ardupilot_takeoff(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

public:
  TakeoffLandingPlanner();
  void timer_callback();
}; 
