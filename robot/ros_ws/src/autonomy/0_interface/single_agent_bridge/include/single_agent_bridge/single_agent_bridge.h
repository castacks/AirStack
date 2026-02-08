#pragma once

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/static_transform_broadcaster.h"
#include <list>
#include <string>
#include <vector>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "rclcpp/qos.hpp"
#include <iostream>
#include "std_msgs/msg/string.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Eigenvalues"
class SingleAgentBridge : public rclcpp::Node{

private:
// ros-parameter
std::string local_frame_id_; // Map Frame_ID for Each Agent
std::string global_frame_id_; // Global Map Frame_ID 
std::string local_mavros_frame_id_; // "mavros_enu" for AirStack
// parameter
std::string robot_name_; // Prefix of Topics
std::string ego_odom_topic_; // Odometry from Isaac Sim (Ground Truth)

rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr ego_pose_global_pub_; // Publish Global Ego-Odometry
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odometry_sub_; // Subscribe Ego Odometry (w.r.t Ego Map Frame)	
rclcpp::TimerBase::SharedPtr timer_tf_; // Publish Tf
bool is_ego_global_pose_received_{false}; // Check Whether the Ego Pose Is Received

std::unique_ptr<tf2_ros::Buffer> tf_buffer_; // TF 
tf2_ros::TransformListener *tf_listener_ptr_; // TF
tf2_ros::TransformBroadcaster *tf_broadcaster_ptr_; //TF

void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
  // Get Map frame's pose with respect to world frame
  geometry_msgs::msg::TransformStamped pose_from_tf_global;
  pose_from_tf_global.header = msg->header;
  pose_from_tf_global.header.frame_id = global_frame_id_;
  pose_from_tf_global.child_frame_id = msg->child_frame_id;
  try{
    pose_from_tf_global = tf_buffer_->lookupTransform(pose_from_tf_global.header.frame_id, msg->child_frame_id, msg->header.stamp);
  }
  catch (const tf2::TransformException &ex){
    RCLCPP_ERROR_STREAM(get_logger(), ex.what());
    RCLCPP_ERROR(get_logger(), "[ERROR] No transform between map and ouster.");
    return;
  }
  is_ego_global_pose_received_ = true;
  ego_pose_global_pub_->publish(pose_from_tf_global);
};
void timer_tf_callback(){ // MAVROS_ENU <-> MAP
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = local_mavros_frame_id_;
  t.child_frame_id = local_frame_id_;
  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;
  Eigen::AngleAxisf yaw_minus_90(-90.0 * M_PI / 180.0, Eigen::Vector3f::UnitZ());
  Eigen::Quaternionf q(yaw_minus_90);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();
  tf_broadcaster_ptr_->sendTransform(t);
}

public:
SingleAgentBridge(): Node("single_agent_bridge"){
  RCLCPP_INFO(this->get_logger(), "Single_Agent Bridge Node Initialized");
  // Initialize parameters, subscriptions, and publishers here
  initialize();
}
void initialize(){
  // Parameters
  this->declare_parameter("robot_name", "robot_0");
  this->get_parameter("robot_name", robot_name_);
  this->declare_parameter("local_map_frame_id", "map_l");
  this->get_parameter("local_map_frame_id", local_frame_id_);
  this->declare_parameter("local_mavros_frame_id", "map_m");
  this->get_parameter("local_mavros_frame_id", local_mavros_frame_id_);
  this->declare_parameter("global_map_frame_id", "map_g");
  this->get_parameter("global_map_frame_id", global_frame_id_);

  // Timer Callback
  timer_tf_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&SingleAgentBridge::timer_tf_callback, this));
  // Publisher
  ego_pose_global_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("global_pose", 1);
  // Subscriber
  ego_odom_topic_ = "/"+robot_name_ + "/odometry_conversion/odometry";
  ego_odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(ego_odom_topic_, 1, std::bind(&SingleAgentBridge::odom_callback, this, std::placeholders::_1));
  // TF
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ptr_ = new tf2_ros::TransformListener(*tf_buffer_);
  tf_broadcaster_ptr_ = new tf2_ros::TransformBroadcaster(*this);
};
};
