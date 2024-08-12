#ifndef _TFLIB_H_
#define _TFLIB_H_

/*
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
*/
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace tflib {

// ==========================================================================
// -------------------------------- Conversion ------------------------------
// ==========================================================================

tf2::Quaternion to_tf(geometry_msgs::msg::Quaternion q);
tf2::Vector3 to_tf(geometry_msgs::msg::Point p);
tf2::Vector3 to_tf(geometry_msgs::msg::Vector3 v);
geometry_msgs::msg::TransformStamped to_tf(nav_msgs::msg::Odometry odom, std::string frame);

// ==========================================================================
// --------------------------------- Utils ----------------------------------
// ==========================================================================

tf2::Quaternion get_stabilized(tf2::Quaternion q);
tf2::Transform get_stabilized(tf2::Transform transform);
geometry_msgs::msg::TransformStamped get_stabilized(geometry_msgs::msg::TransformStamped transform);
bool transform_odometry(tf2_ros::Buffer* tf_buffer, nav_msgs::msg::Odometry odom,
                        std::string new_frame_id, std::string new_child_frame_id,
                        nav_msgs::msg::Odometry* out_odom);
// This can throw any exception that lookupTransform throws
nav_msgs::msg::Odometry transform_odometry(tf2_ros::Buffer* tf_buffer, nav_msgs::msg::Odometry odom,
                                           std::string new_frame_id, std::string new_child_frame_id,
                                           rclcpp::Duration polling_sleep_duration);

// ==========================================================================
// ------------------------------ Transforms --------------------------------
// ==========================================================================

bool to_frame(tf2_ros::Buffer* tf_buffer, tf2::Vector3 vec, std::string source, std::string target,
              rclcpp::Time stamp, tf2::Vector3* out,
              rclcpp::Duration duration = rclcpp::Duration(0, 100000000));  // 0.1 second;
}  // namespace tflib

#endif
