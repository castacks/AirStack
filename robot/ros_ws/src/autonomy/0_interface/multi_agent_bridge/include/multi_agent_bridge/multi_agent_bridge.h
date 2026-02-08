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
class MultiAgentBridge : public rclcpp::Node{
struct PclPoint{
  double x;
  double y;
  double z;
};
struct BboxSize{
  double x;
  double y;
  double z;
};
private:
rclcpp::TimerBase::SharedPtr timer_planner_; // Publish Neighbor Odometrys
rclcpp::TimerBase::SharedPtr timer_tf_; // Publish Tf
std::string robot_name_; // Prefix of Topics
int robot_id_; // Robot Id
int num_agent_; // Number of Agents
bool neighbor_mask_; // True: Remove PCL near Agents, False: Remain PCL 

std::string local_frame_id_; // Map Frame_ID for Each Agent
std::string global_frame_id_; // Global Map Frame_ID 
std::string local_mavros_frame_id_; // "mavros_enu" for AirStack
std::string ego_odom_topic_; // Odometry from Isaac Sim (Ground Truth)
std::string pointcloud_topic_; // Point Cloud Topic
BboxSize bbox_size_; // Mask Size
Eigen::Transform<float, 3, Eigen::Affine> current_odom_local_mat_; // Ego-Odometry
Eigen::Transform<float, 3, Eigen::Affine> transform_mat_; // Convert Data to Local Map Frame
	
rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr neighbor_array_odom_pub_; // Publish Neighbor Odometry
rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr ego_pose_global_pub_; // Publish Global Ego-Odometry
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_pointcloud_pub_; // Filtered PointCloud
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr raw_pointcloud_sub_; // Read PCL from Sensors
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr ego_odometry_sub_; // Subscribe Ego Odometry (w.r.t Ego Map Frame)
	
std::vector<rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr> neighbor_global_pose_sub_; // Subscribe Neighbor's Global Pose
bool is_ego_global_pose_received_{false}; // Check Whether the Ego Pose Is Received
std::vector<bool> neighbor_global_pose_recieved_array_;  // Check Whether the Neighbor Poses are Received
std::vector<int> neighbor_id_array_; // Neighbor_ID
std::vector<Eigen::Transform<float, 3, Eigen::Affine>> neighbor_odom_local_mat_array_; // Neighbor Odom Local Pose
std::unique_ptr<tf2_ros::Buffer> tf_buffer_; // TF 
tf2_ros::TransformListener *tf_listener_ptr_; // TF
tf2_ros::TransformBroadcaster *tf_broadcaster_ptr_; //TF

void neighbor_callback(const geometry_msgs::msg::TransformStamped::SharedPtr msg, const int &neighbor_index){
  if (not is_ego_global_pose_received_)
    return;
  neighbor_global_pose_recieved_array_[neighbor_index] = true;
  Eigen::Transform<float, 3, Eigen::Affine> neighbor_odom;
  neighbor_odom.setIdentity();
  neighbor_odom.translate(Eigen::Vector3f(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z));
  neighbor_odom.rotate(Eigen::Quaternionf(msg->transform.rotation.w, msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z));
  Eigen::Transform<float, 3, Eigen::Affine> neighbor_odom_local_map = transform_mat_ * neighbor_odom;
  neighbor_odom_local_mat_array_[neighbor_index] = neighbor_odom_local_map;
}
void publish_neighbor_odom_local(){
  bool all_neighbor_odom_received = std::all_of(neighbor_global_pose_recieved_array_.begin(), neighbor_global_pose_recieved_array_.end(), [](bool v)
													  { return v; });
  if (all_neighbor_odom_received){ // all_neighbor_odom_received
    geometry_msgs::msg::PoseArray neighbor_odom_local_array_msg;
    neighbor_odom_local_array_msg.header.frame_id = local_frame_id_;
    geometry_msgs::msg::Pose pose;
    for (const auto &odom_mat : neighbor_odom_local_mat_array_){
      pose.position.x = odom_mat.translation().x();
      pose.position.y = odom_mat.translation().y();
      pose.position.z = odom_mat.translation().z();
      Eigen::Quaternionf q(odom_mat.rotation());
      pose.orientation.w = q.w();
      pose.orientation.x = q.x();
      pose.orientation.y = q.y();
      pose.orientation.z = q.z();

      neighbor_odom_local_array_msg.poses.push_back(pose);
    }
    neighbor_array_odom_pub_->publish(neighbor_odom_local_array_msg);
  }
};
void pcl_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg){
  if (not neighbor_mask_){
    filtered_pointcloud_pub_->publish(*cloud_msg);
    //return;
  }
    
  bool all_neighbor_odom_received = std::all_of(neighbor_global_pose_recieved_array_.begin(), neighbor_global_pose_recieved_array_.end(), [](bool v)
                                                                                                          { return v; });
  if (not (all_neighbor_odom_received and is_ego_global_pose_received_))
    return;
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
  std::vector<PclPoint> original_cloud;
  std::vector<PclPoint> filtered_cloud_global;
  pcl::fromROSMsg(*cloud_msg,*input_cloud);
  Eigen::Vector3f pcl_point_local_map;
  Eigen::Vector3f pcl_point_lidar;
  rclcpp::Time current_sensor_time = cloud_msg->header.stamp;
  geometry_msgs::msg::TransformStamped lidar_pose_from_tf;
  try{
    lidar_pose_from_tf = tf_buffer_->lookupTransform(local_frame_id_,cloud_msg->header.frame_id,current_sensor_time);
  }
  catch(const tf2::TransformException &ex){
    RCLCPP_ERROR_STREAM(get_logger(),ex.what());
    RCLCPP_ERROR(get_logger(),"[ERROR] No transform between map and ouster.");
  }
  Eigen::Transform<float,3,Eigen::Affine>poseMat;
  poseMat.setIdentity();
  poseMat.translate(Eigen::Vector3f((float)lidar_pose_from_tf.transform.translation.x,(float)lidar_pose_from_tf.transform.translation.y,(float)lidar_pose_from_tf.transform.translation.z));
  poseMat.rotate(Eigen::Quaternionf((float)lidar_pose_from_tf.transform.rotation.w,(float)lidar_pose_from_tf.transform.rotation.x,(float)lidar_pose_from_tf.transform.rotation.y,(float)lidar_pose_from_tf.transform.rotation.z));
  Eigen::Transform<float,3,Eigen::Affine>poseMatInverse = poseMat.inverse();
  bool mask_flag = false;
  for (const auto &point:input_cloud->points){
    pcl_point_local_map = poseMat*Eigen::Vector3f(point.x,point.y,point.z);
    mask_flag = false;
    for(int i =0;i<neighbor_odom_local_mat_array_.size();i++){
      if(std::abs(neighbor_odom_local_mat_array_[i].translation().x()-pcl_point_local_map[0])<bbox_size_.x and std::abs(neighbor_odom_local_mat_array_[i].translation().y()-pcl_point_local_map[1])<bbox_size_.y and std::abs(neighbor_odom_local_mat_array_[i].translation().z()-pcl_point_local_map[2])<bbox_size_.z){
        mask_flag = true;
      	break;
      }
    }
    if (!mask_flag)
      filtered_cloud->points.push_back(point);
  }
  filtered_cloud->width = filtered_cloud->points.size();
  filtered_cloud->height = 1;
  filtered_cloud->is_dense = true;
  // Convert back to ROS2 msg;
  sensor_msgs::msg::PointCloud2 output_msg;
  pcl::toROSMsg(*filtered_cloud, output_msg);
  output_msg.header = cloud_msg->header;
  filtered_pointcloud_pub_->publish(output_msg);
}
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
  // Get Ego odom with respect to map frame
  current_odom_local_mat_.setIdentity();
  current_odom_local_mat_.translate(Eigen::Vector3f(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z));
  current_odom_local_mat_.rotate(Eigen::Quaternionf(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z));

  Eigen::Transform<float, 3, Eigen::Affine> current_odom_global_mat;
  current_odom_global_mat.setIdentity();
  current_odom_global_mat.translate(Eigen::Vector3f(pose_from_tf_global.transform.translation.x, pose_from_tf_global.transform.translation.y, pose_from_tf_global.transform.translation.z));
  current_odom_global_mat.rotate(Eigen::Quaternionf(pose_from_tf_global.transform.rotation.w, pose_from_tf_global.transform.rotation.x, pose_from_tf_global.transform.rotation.y, pose_from_tf_global.transform.rotation.z));
  transform_mat_ = current_odom_local_mat_ * current_odom_global_mat.inverse();
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
void timer_execution_callback(){
  publish_neighbor_odom_local();
};

public:
MultiAgentBridge(): Node("multi_agent_bridge"){
  RCLCPP_INFO(this->get_logger(), "Multi_agent Bridge Node Initialized");
  // Initialize parameters, subscriptions, and publishers here
  initialize();
}
void initialize(){
  // Parameters
  this->declare_parameter("num_robot", 1);
  this->get_parameter("num_robot", num_agent_);
  this->declare_parameter("robot_name", "robot_0");
  this->get_parameter("robot_name", robot_name_);
  this->declare_parameter("local_map_frame_id", "map_l");
  this->get_parameter("local_map_frame_id", local_frame_id_);
  this->declare_parameter("local_mavros_frame_id", "map_m");
  this->get_parameter("local_mavros_frame_id", local_mavros_frame_id_);
  this->declare_parameter("global_map_frame_id", "map_g");
  this->get_parameter("global_map_frame_id", global_frame_id_);
  this->declare_parameter("neighbor_mask", false);
  this->get_parameter("neighbor_mask", neighbor_mask_);
  this->declare_parameter("x_size",0.0);
  this->get_parameter("x_size",bbox_size_.x);
  this->declare_parameter("y_size",0.0);
  this->get_parameter("y_size",bbox_size_.y);
  this->declare_parameter("z_size",0.0);
  this->get_parameter("z_size",bbox_size_.z);
  for (int i = 0; i < num_agent_; i++){
    if (robot_name_ == "robot_" + std::to_string(i + 1))
      robot_id_ = i + 1;
    else
      neighbor_id_array_.push_back(i + 1);
  }
  neighbor_global_pose_recieved_array_.resize(neighbor_id_array_.size());
  neighbor_odom_local_mat_array_.resize(neighbor_id_array_.size());
  for (size_t i = 0; i < neighbor_global_pose_recieved_array_.size(); ++i){
    neighbor_global_pose_recieved_array_[i] = false;
  }
  for (int i = 0; i < neighbor_id_array_.size(); i++){
    std::string neighbor_global_pose_topic = "/robot_" + std::to_string(neighbor_id_array_[i]) + "/global_pose";
    auto sub = this->create_subscription<geometry_msgs::msg::TransformStamped>(neighbor_global_pose_topic, 1, [this, i](const geometry_msgs::msg::TransformStamped::SharedPtr msg)
																					   { this->neighbor_callback(msg, i); });
    neighbor_global_pose_sub_.push_back(sub);
  }
  // Timer Callback
  timer_planner_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&MultiAgentBridge::timer_execution_callback, this));
  timer_tf_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&MultiAgentBridge::timer_tf_callback, this));
  // Publisher
  neighbor_array_odom_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("neighbor_array_odom", 1);
  ego_pose_global_pub_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("global_pose", 1);
  filtered_pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("sensors/filtered_pointcloud",1);
  // Subscriber
  ego_odom_topic_ = "/robot_" + std::to_string(robot_id_) + "/odometry_conversion/odometry";
  pointcloud_topic_ ="/robot_"+std::to_string(robot_id_)+"/sensors/ouster/point_cloud";
  ego_odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(ego_odom_topic_, 1, std::bind(&MultiAgentBridge::odom_callback, this, std::placeholders::_1));
  raw_pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(pointcloud_topic_,1,std::bind(&MultiAgentBridge::pcl_callback,this,std::placeholders::_1));
  // TF
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ptr_ = new tf2_ros::TransformListener(*tf_buffer_);
  tf_broadcaster_ptr_ = new tf2_ros::TransformBroadcaster(*this);
};
};
