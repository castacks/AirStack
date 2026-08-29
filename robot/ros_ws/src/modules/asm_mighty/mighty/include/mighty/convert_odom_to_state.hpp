#pragma once

#include <dynus_interfaces/msg/state.hpp>

#include "mighty/mighty.hpp"

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"

/** @brief ROS 2 node that converts nav_msgs/Odometry to dynus_interfaces/State.
 *
 *  Subscribes to an odometry topic and republishes it as a State message
 *  suitable for consumption by the MIGHTY planner.
 */
class OdometryToStateNode : public rclcpp::Node {
 public:
  /** @brief Construct the node, set up subscriber and publisher. */
  OdometryToStateNode();

 private:
  void callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
  rclcpp::Publisher<dynus_interfaces::msg::State>::SharedPtr state_publisher_;
};