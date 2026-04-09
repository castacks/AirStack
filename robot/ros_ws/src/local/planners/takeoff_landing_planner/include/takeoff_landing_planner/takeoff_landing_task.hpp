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

#include <atomic>
#include <deque>
#include <mutex>
#include <string>

#include <airstack_common/tflib.hpp>
#include <airstack_msgs/msg/odometry.hpp>
#include <airstack_msgs/msg/trajectory_xyzv_yaw.hpp>
#include <airstack_msgs/srv/trajectory_mode.hpp>
#include <mavros_msgs/msg/extended_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <trajectory_library/trajectory_library.hpp>

#include "task_msgs/action/land_task.hpp"
#include "task_msgs/action/takeoff_task.hpp"

class TakeoffLandingTaskNode : public rclcpp::Node
{
public:
  using TakeoffTask = task_msgs::action::TakeoffTask;
  using TakeoffGoalHandle = rclcpp_action::ServerGoalHandle<TakeoffTask>;
  using LandTask = task_msgs::action::LandTask;
  using LandGoalHandle = rclcpp_action::ServerGoalHandle<LandTask>;

  TakeoffLandingTaskNode();

private:
  // parameters
  double default_takeoff_velocity_;
  double default_landing_velocity_;
  double takeoff_acceptance_distance_;
  double takeoff_acceptance_time_;
  double landing_stationary_distance_;
  double landing_acceptance_time_;
  double landing_tracking_point_ahead_time_;
  double takeoff_path_roll_;
  double takeoff_path_pitch_;
  bool takeoff_path_relative_to_orientation_;

  // shared state (protected by mutex)
  std::mutex odom_mutex_;
  nav_msgs::msg::Odometry robot_odom_;
  bool got_robot_odom_{false};

  std::mutex tracking_point_mutex_;
  airstack_msgs::msg::Odometry tracking_point_odom_;
  bool got_tracking_point_{false};

  std::mutex completion_mutex_;
  float completion_percentage_{0.0f};
  bool got_completion_percentage_{false};

  // precondition state (atomic for lock-free reads in handle_goal)
  std::atomic<bool> is_armed_{false};
  std::atomic<bool> has_control_{false};
  std::atomic<bool> state_estimate_timed_out_{false};

  // landed state from mavros
  std::atomic<uint8_t> landed_state_{0};  // mavros_msgs::msg::ExtendedState::LANDED_STATE_UNDEFINED

  // task exclusion
  std::atomic<bool> task_active_{false};
  std::atomic<bool> cancel_requested_{false};

  // subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr robot_odom_sub_;
  rclcpp::Subscription<airstack_msgs::msg::Odometry>::SharedPtr tracking_point_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr is_armed_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr has_control_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr state_estimate_timed_out_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr completion_percentage_sub_;
  rclcpp::Subscription<mavros_msgs::msg::ExtendedState>::SharedPtr extended_state_sub_;

  // publishers
  rclcpp::Publisher<airstack_msgs::msg::TrajectoryXYZVYaw>::SharedPtr traj_override_pub_;

  // service client
  rclcpp::Client<airstack_msgs::srv::TrajectoryMode>::SharedPtr traj_mode_client_;

  // action servers
  rclcpp_action::Server<TakeoffTask>::SharedPtr takeoff_server_;
  rclcpp_action::Server<LandTask>::SharedPtr land_server_;

  // subscription callbacks
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void tracking_point_callback(const airstack_msgs::msg::Odometry::SharedPtr msg);
  void completion_percentage_callback(const std_msgs::msg::Float32::SharedPtr msg);

  // helpers
  bool set_trajectory_mode(int32_t mode);

  // TakeoffTask action server callbacks
  rclcpp_action::GoalResponse takeoff_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const TakeoffTask::Goal> goal);
  rclcpp_action::CancelResponse takeoff_handle_cancel(
    std::shared_ptr<TakeoffGoalHandle> goal_handle);
  void takeoff_handle_accepted(std::shared_ptr<TakeoffGoalHandle> goal_handle);
  void takeoff_execute(std::shared_ptr<TakeoffGoalHandle> goal_handle);

  // LandTask action server callbacks
  rclcpp_action::GoalResponse land_handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const LandTask::Goal> goal);
  rclcpp_action::CancelResponse land_handle_cancel(
    std::shared_ptr<LandGoalHandle> goal_handle);
  void land_handle_accepted(std::shared_ptr<LandGoalHandle> goal_handle);
  void land_execute(std::shared_ptr<LandGoalHandle> goal_handle);
};
