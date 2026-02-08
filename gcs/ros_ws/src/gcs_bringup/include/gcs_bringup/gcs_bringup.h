#pragma once

#include <list>
#include <string>
#include <vector>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <memory>
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Eigenvalues"
#include "std_msgs/msg/string.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "visualization_msgs/msg/marker.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "std_msgs/msg/color_rgba.hpp"
typedef message_filters::sync_policies::ApproximateTime<nav_msgs::msg::Odometry,
                                                        geometry_msgs::msg::TransformStamped> SyncOdomData;
class GcsBringup: public rclcpp::Node{
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
  rclcpp::TimerBase::SharedPtr timer_publisher_; // Timer
  rclcpp::Time latest_time_; // Time Stamp for Publishing Data
  int num_agent_; // Number of Agents
  BboxSize mask_size_; // Masking size (to prevent agent to be obstacles) 
  std::vector<message_filters::Subscriber<nav_msgs::msg::Odometry>*> local_odom_sub_; // Listen Odometry Data w.r.t Each Map Frame  
  std::vector<message_filters::Subscriber<geometry_msgs::msg::TransformStamped>*> global_odom_sub_; // Listen Odoemtry Data w.r.t World Frame
  std::vector<message_filters::Synchronizer<SyncOdomData>*> subscription_synchronizer_; // Synchronizer for Listening Local and Global Odometry
  std::vector<rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr> pcl_sub_; // Listen Poiintcloud 
  std::vector<bool> agent_odom_received_flag_; // For Checking Whether Data Received
  std::vector<Eigen::Transform<float,3,Eigen::Affine>> transform_mat_; // Converting from Data (w.r.t Each Map Fame) to Data (w.r.t World Frame)
  std::vector<Eigen::Transform<float,3,Eigen::Affine>> global_pose_mat_; // Visualize Odometry w.r.t World Frame
  std::vector<std::vector<PclPoint>> pcl_; // Point Cloud Callback Function
  std::vector<std::vector<std_msgs::msg::ColorRGBA>> pcl_color_; // Point cloud color from vdb_mapping
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr global_team_pose_pub_; // Publish Odometry w.r.t World Frame
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pcl_pub_; // Listen vdb_mapping results
  Eigen::Vector3f pcl_scale_; // vdb_mapping Voxel Size
  void SyncCallback(const int & index, const nav_msgs::msg::Odometry::ConstSharedPtr & local_odom_msg, const geometry_msgs::msg::TransformStamped::ConstSharedPtr global_odom_msg){ // Callback for Listening Odometrys (both Local and Global Odometry)
    agent_odom_received_flag_[index] = true;
    global_pose_mat_[index].setIdentity();
    global_pose_mat_[index].translate(Eigen::Vector3f(global_odom_msg->transform.translation.x,global_odom_msg->transform.translation.y,global_odom_msg->transform.translation.z));
    global_pose_mat_[index].rotate(Eigen::Quaternionf(global_odom_msg->transform.rotation.w,global_odom_msg->transform.rotation.x,global_odom_msg->transform.rotation.y,global_odom_msg->transform.rotation.z));
    Eigen::Transform<float,3,Eigen::Affine> pose_local_mat;
    pose_local_mat.setIdentity();
    pose_local_mat.translate(Eigen::Vector3f(local_odom_msg->pose.pose.position.x,local_odom_msg->pose.pose.position.y,local_odom_msg->pose.pose.position.z));
    pose_local_mat.rotate(Eigen::Quaternionf(local_odom_msg->pose.pose.orientation.w, local_odom_msg->pose.pose.orientation.x, local_odom_msg->pose.pose.orientation.y,local_odom_msg->pose.pose.orientation.z));
    transform_mat_[index] = global_pose_mat_[index]*pose_local_mat.inverse();
    latest_time_ = rclcpp::Time(local_odom_msg->header.stamp);
  };
  void PclCallback(const visualization_msgs::msg::Marker::SharedPtr msg, const int &index){ // Point Cloud Callback Function
    bool all_agent_odom_received = std::all_of(agent_odom_received_flag_.begin(),agent_odom_received_flag_.end(),[](bool v){return v;});
    if (not all_agent_odom_received)
      return;
    pcl_[index].clear();
    pcl_color_[index].clear();
    Eigen::Vector3f pcl_point_global;
    PclPoint pcl_point_global_store;
    Eigen::Vector3f pcl_point_local;
    bool store_flag;
    for(int i =0;i<msg->points.size();i++){
      pcl_point_local[0] = msg->points[i].x;
      pcl_point_local[1] = msg->points[i].y;
      pcl_point_local[2] = msg->points[i].z;
      pcl_point_global = transform_mat_[index]*pcl_point_local;
      pcl_point_global_store.x = pcl_point_global[0];
      pcl_point_global_store.y = pcl_point_global[1];
      pcl_point_global_store.z = pcl_point_global[2];
      store_flag = true;
      for(int j=0;j<num_agent_;j++){
        if(std::abs(global_pose_mat_[j].translation().x()-pcl_point_global_store.x)<mask_size_.x and std::abs(global_pose_mat_[j].translation().y()-pcl_point_global_store.y)<mask_size_.y and std::abs(global_pose_mat_[j].translation().z()-pcl_point_global_store.z)<mask_size_.z)
	  store_flag = false;
      }
      if(not store_flag)
        continue;
      pcl_[index].push_back(pcl_point_global_store);
      pcl_color_[index].push_back(msg->colors[i]);
    }
    pcl_scale_[0] = msg->scale.x;
    pcl_scale_[1] = msg->scale.y;
    pcl_scale_[2] = msg->scale.z;
  };
  void timer_execution_callback(){// Publish Datas: Whole PCL, POSES
    bool all_agent_odom_received = std::all_of(agent_odom_received_flag_.begin(),agent_odom_received_flag_.end(),[](bool v){return v;});
    if (not all_agent_odom_received)
      return;
    geometry_msgs::msg::Pose agent_pose;
    geometry_msgs::msg::PoseArray team_pose;
    team_pose.header.frame_id="map";
    team_pose.header.stamp = latest_time_;
	
    for(int i=0;i<num_agent_;i++){
      agent_pose.position.x = global_pose_mat_[i].translation().x();
      agent_pose.position.y = global_pose_mat_[i].translation().y();
      agent_pose.position.z = global_pose_mat_[i].translation().z();
      Eigen::Quaternionf q(global_pose_mat_[i].rotation());      
      agent_pose.orientation.w = q.w();
      agent_pose.orientation.x = q.x();
      agent_pose.orientation.y = q.y();
      agent_pose.orientation.z = q.z();
      team_pose.poses.push_back(agent_pose);
    }
    global_team_pose_pub_->publish(team_pose);
    
    visualization_msgs::msg::Marker pcl_integrated;
    pcl_integrated.type = visualization_msgs::msg::Marker::CUBE_LIST;
    pcl_integrated.header.frame_id="map";
    pcl_integrated.header.stamp = latest_time_;
    pcl_integrated.ns="PCL";
    pcl_integrated.id =0;
    pcl_integrated.action = visualization_msgs::msg::Marker::ADD;
    pcl_integrated.scale.x = pcl_scale_[0];
    pcl_integrated.scale.y = pcl_scale_[1];
    pcl_integrated.scale.z = pcl_scale_[2];
    
    //pcl_integrated.color.a = 1.0;
    //pcl_integrated.color.r = 0.5;
    //pcl_integrated.color.g = 0.5;
    //pcl_integrated.color.b = 0.5;
    geometry_msgs::msg::Point pt;
    for(int i=0;i<pcl_.size();i++){
      for(int j=0;j<pcl_[i].size();j++){
        pt.x = pcl_[i][j].x, pt.y = pcl_[i][j].y, pt.z = pcl_[i][j].z;
        pcl_integrated.points.push_back(pt);
        pcl_integrated.colors.push_back(pcl_color_[i][j]);
      }
    }
    pcl_pub_->publish(pcl_integrated);
  };

	
public:
  GcsBringup(): Node("gcs_bringup"){
    RCLCPP_INFO(this->get_logger(), "GCS Bringup Node Initialized");
    initialize();
  }
  void initialize(){
    this->declare_parameter("num_robot", 2); // Number of Agentss
    this->get_parameter("num_robot", num_agent_); 
    this->declare_parameter("mask_size_x",0.1); // Mask_Size (x [m])
    this->get_parameter("mask_size_x",mask_size_.x);
    this->declare_parameter("mask_size_y",0.1); // Mask_Size (y [m])
    this->get_parameter("mask_size_y",mask_size_.y);
    this->declare_parameter("mask_size_z",0.1); // Mask_Size (z [m])
    this->get_parameter("mask_size_z",mask_size_.z);
    local_odom_sub_.resize(num_agent_);
    global_odom_sub_.resize(num_agent_);
    pcl_.resize(num_agent_);
    pcl_color_.resize(num_agent_);
    agent_odom_received_flag_.resize(num_agent_);
    transform_mat_.resize(num_agent_);
    global_pose_mat_.resize(num_agent_);
    subscription_synchronizer_.resize(num_agent_);
    for(int i=0;i<num_agent_;i++){
      agent_odom_received_flag_[i] = false;
      transform_mat_[i].setIdentity();
      global_pose_mat_[i].setIdentity();
    }
    
    std::string local_odom_topic, global_odom_topic, pcl_topic, pcl_sub_topic, global_pose_topic, pcl_pub_topic; // Message Topics
    pcl_sub_topic = "/vdb_mapping/vdb_map_visualization";
    global_pose_topic = "/robot_team/pose";
    pcl_pub_topic="/robot_team/point_cloud";
    for(int i =0;i<num_agent_;i++){ // Odometry Synchronization
      //topic
      local_odom_topic="/robot_"+std::to_string(i+1)+"/odometry_conversion/odometry";
      global_odom_topic="/robot_"+std::to_string(i+1)+"/global_pose";
      pcl_topic="/robot_"+std::to_string(i+1)+pcl_sub_topic;
      // odom subscriber
      local_odom_sub_[i] = new message_filters::Subscriber<nav_msgs::msg::Odometry>(this,local_odom_topic);
      global_odom_sub_[i] = new message_filters::Subscriber<geometry_msgs::msg::TransformStamped>(this,global_odom_topic);
      subscription_synchronizer_[i] = new message_filters::Synchronizer<SyncOdomData>(SyncOdomData(10),*local_odom_sub_[i],*global_odom_sub_[i]);
      subscription_synchronizer_[i]->registerCallback(std::bind(&GcsBringup::SyncCallback,this,i,std::placeholders::_1,std::placeholders::_2));
      // pcl subscriber
      auto sub = this->create_subscription<visualization_msgs::msg::Marker>(pcl_topic,1,[this,i](const visualization_msgs::msg::Marker::SharedPtr msg){this->PclCallback(msg,i);});
      pcl_sub_.push_back(sub);
      // pose publisher
    }    
    global_team_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(global_pose_topic,1);
    pcl_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(pcl_pub_topic,1);
    timer_publisher_ = this->create_wall_timer(std::chrono::milliseconds(33), std::bind(&GcsBringup::timer_execution_callback, this));
  };
};