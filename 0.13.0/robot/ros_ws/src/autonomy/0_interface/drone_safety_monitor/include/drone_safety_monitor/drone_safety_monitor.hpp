#pragma once

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <airstack_common/ros2_helper.hpp>
#include <limits>

// =====================================================================================
// ---------------------------------- TimeChecker --------------------------------------
// =====================================================================================

class TimeChecker{
private:
  rclcpp::Time time;
  bool time_initialized;

public:
  TimeChecker();
  void update(rclcpp::Time time);
  double elapsed_since_last_update(rclcpp::Time time);
};
