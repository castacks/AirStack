#ifndef _TFLIB_H_
#define _TFLIB_H_

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <airstack_msgs/msg/odometry.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/exceptions.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace tflib{
  
  // ==========================================================================
  // -------------------------------- Conversion ------------------------------
  // ==========================================================================
  
  tf2::Quaternion to_tf(geometry_msgs::msg::Quaternion q);
  tf2::Vector3 to_tf(geometry_msgs::msg::Point p);
  tf2::Vector3 to_tf(geometry_msgs::msg::Vector3 v);
  tf2::Stamped<tf2::Transform> to_tf(nav_msgs::msg::Odometry odom);//, std::string frame);
  tf2::Stamped<tf2::Transform> to_tf(airstack_msgs::msg::Odometry odom);//, std::string frame);
  geometry_msgs::msg::Point from_tf(tf2::Vector3 v);
  geometry_msgs::msg::Quaternion from_tf(tf2::Quaternion q);
  
  // ==========================================================================
  // --------------------------------- Utils ----------------------------------
  // ==========================================================================
  
  tf2::Quaternion get_stabilized(tf2::Quaternion q);
  tf2::Transform get_stabilized(tf2::Transform transform);
  tf2::Stamped<tf2::Transform> get_stabilized(tf2::Stamped<tf2::Transform> transform);
  bool transform_odometry(tf2_ros::Buffer* tf_buffer, nav_msgs::msg::Odometry odom,
			  std::string new_frame_id, std::string new_child_frame_id, nav_msgs::msg::Odometry* out_odom,
			  rclcpp::Duration polling_sleep_duration=rclcpp::Duration::from_seconds(0.1));
  // This can throw any exception that lookupTransform throws
  nav_msgs::msg::Odometry transform_odometry(tf2_ros::Buffer* tf_buffer, nav_msgs::msg::Odometry odom,
					     std::string new_frame_id, std::string new_child_frame_id,
					     rclcpp::Duration polling_sleep_duration=rclcpp::Duration::from_seconds(0.1));
  bool transform_odometry(tf2_ros::Buffer* tf_buffer, airstack_msgs::msg::Odometry odom,
			  std::string new_frame_id, std::string new_child_frame_id, airstack_msgs::msg::Odometry* out_odom,
			  rclcpp::Duration polling_sleep_duration=rclcpp::Duration::from_seconds(0.1));
  // This can throw any exception that lookupTransform throws
  airstack_msgs::msg::Odometry transform_odometry(tf2_ros::Buffer* tf_buffer, airstack_msgs::msg::Odometry odom,
						  std::string new_frame_id, std::string new_child_frame_id,
						  rclcpp::Duration polling_sleep_duration=rclcpp::Duration::from_seconds(0.1));
  
  // ==========================================================================
  // ------------------------------ Transforms --------------------------------
  // ==========================================================================
  
  bool to_frame(tf2_ros::Buffer* tf_buffer,
		tf2::Vector3 vec, std::string source, std::string target, rclcpp::Time stamp,
		tf2::Vector3* out, rclcpp::Duration polling_sleep_duration=rclcpp::Duration::from_seconds(0.1));
}

#endif
