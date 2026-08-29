// Copyright 2017 Massachusetts Institute of Technology
#pragma once

// C++ Standard Library
#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// Eigen
#include <Eigen/Dense>

// PCL
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>      // pcl::PointCloud<T>
#include <pcl/point_types.h>      // pcl::PointXYZ, pcl::PointXYZI, etc.
#include <pcl_conversions/pcl_conversions.h>  // between sensor_msgs and PCL
#include <pcl/filters/voxel_grid.h>

// ROS 2 core
#include <rclcpp/rclcpp.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

// ROS 2 message types
#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <dynus_interfaces/msg/state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// ROS 2 utilities
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <message_filters/subscriber.h>

// Project headers
#include "fla_utils/param_utils.h"
#include "fla_utils/process_status.h"
#include "global_mapper/global_mapper.h"
#include "global_mapper/params.h"
#include "global_mapper/planning_grids.h"
#include "distance_grid_2d/distance_grid_2d.h"
#include "occupancy_grid_2d/occupancy_grid_2d.h"
#include "global_mapper_ros/obstacle_tracker.h"

#include <dynus_interfaces/msg/dyn_traj.hpp>

namespace global_mapper_ros
{
class GlobalMapperRos : public rclcpp::Node
{
public:
  GlobalMapperRos();
  void Run();
  double start_time_; 

private:
  void GetParams();
  void InitSubscribers();
  void InitPublishers();
  void PopulateUnknownPointCloudMsg(const voxel_grid::VoxelGrid<float>& occupancy_grid,
                                    sensor_msgs::msg::PointCloud2* pointcloud);
  void PopulateOccupancyPointCloudMsg(const voxel_grid::VoxelGrid<float>& occupancy_grid,
                                      sensor_msgs::msg::PointCloud2* pointcloud);
  void PopulateDistancePointCloudMsg(const voxel_grid::VoxelGrid<int>& distance_grid,
                                     sensor_msgs::msg::PointCloud2* pointcloud);
  void PopulateCostPointCloudMsg(const voxel_grid::VoxelGrid<int>& cost_grid, sensor_msgs::msg::PointCloud2* pointcloud);
  void PopulatePathMsg(const std::vector<std::array<double, 3>>& path, nav_msgs::msg::Path* path_msg);
  void PopulateEsdf2DMsg(const distance_grid_2d::DistanceGrid2D& grid,
                         nav_msgs::msg::OccupancyGrid* msg);
  void PopulateOcc2DMsg(const occupancy_grid_2d::OccupancyGrid2D& grid,
                        nav_msgs::msg::OccupancyGrid* msg);
  void PopulateDynamicPointCloudMsg(const voxel_grid::VoxelGrid<float>& occupancy_grid,
                                    const voxel_grid::VoxelGrid<std::array<double, 7>>& temporal_grid,
                                    float occ_threshold, double dynamic_persistence, double temporal_timestamp,
                                    const std::vector<TrackedObstacle>& tracked_obstacles,
                                    sensor_msgs::msg::PointCloud2* dynamic_pointcloud,
                                    sensor_msgs::msg::PointCloud2* static_pointcloud,
                                    pcl::PointCloud<pcl::PointXYZ>::Ptr dynamic_cloud_out = nullptr);
  void Publish();

  // callbacks
  void PointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & cloud_msg);
  void PoseCallback(const dynus_interfaces::msg::State::SharedPtr pose_ptr);
  void PoseStampedCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose_ptr);
  void GoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal_ptr);
  void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_ptr);

  // health and status
  enum ProcessArgs
  {
    NOMINAL = 0,
    NO_POSE = 1,
    NO_GOAL = 2,
    NO_DEPTH_IMAGE = 3
  };

  // name of the drone
  std::string name_drone;
  std::string lidar_frame_;
  std::string drone_frame_id_;

  // publishers
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr occ_grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr unknown_grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frontier_grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dist_grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cost_grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr dynamic_grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr static_grid_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr downsampled_occ_grid_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr esdf_2d_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_2d_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr sparse_path_pub_;

  // obstacle tracker publishers
  rclcpp::Publisher<dynus_interfaces::msg::DynTraj>::SharedPtr predicted_traj_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracker_bbox_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr tracker_prediction_pub_;

  // time (in secs) of the last point cloud fused in this map
  rclcpp::Time tstampLastPclFused_;
  rclcpp::TimerBase::SharedPtr grid_pub_timer_;

  // subscribers
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Subscription<dynus_interfaces::msg::State>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_stamped_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;

  // params
  global_mapper::Params params_;
  bool publish_occupancy_grid_;
  bool publish_unknown_grid_;
  bool publish_distance_grid_;
  bool publish_cost_grid_;
  bool publish_path_;
  bool publish_dynamic_grid_;
  bool publish_obstacle_tracking_;
  bool publish_esdf_2d_ = false;
  double esdf_2d_publish_rate_{5.0};
  rclcpp::Time last_esdf_2d_pub_time_{0, 0, RCL_ROS_TIME};
  double clear_unknown_distance_;
  double occupancy_publish_rate_{5.0};
  double downsampled_occupancy_publish_rate_{5.0};
  double unknown_publish_rate_{5.0};
  int min_occupied_neighbors_pub_{0};  // drop occupied voxels with fewer than N occupied 26-neighbors when publishing (0 = disabled)
  rclcpp::Time last_occ_pub_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_ds_occ_pub_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_unknown_pub_time_{0, 0, RCL_ROS_TIME};
  double target_altitude_;

  // obstacle tracker
  ObstacleTrackerParams obstacle_tracker_params_;
  std::unique_ptr<ObstacleTracker> obstacle_tracker_;

  // i/o flags
  bool got_goal_;
  bool got_pose_;
  bool got_depth_image_;

  std::shared_ptr<fla_utils::ProcessStatus> process_status_;

  std::shared_ptr<global_mapper::GlobalMapper> global_mapper_ptr_;
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_ptr_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_ptr_;
  std::shared_ptr<image_transport::ImageTransport> it_ptr_;


  // Ros pointcloud message pointer 
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;

};
}  // namespace global_mapper_ros
