// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2022 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------
//----------------------------------------------------------------------
/*!\file
 *
 * \author  Marvin Große Besselmann grosse@fzi.de
 * \date    2022-05-07
 *
 */
//----------------------------------------------------------------------
#ifndef VDB_MAPPING_ROS2_VDBMAPPINGROS2_HPP_INCLUDED
#define VDB_MAPPING_ROS2_VDBMAPPINGROS2_HPP_INCLUDED

#include <rclcpp/rclcpp.hpp>

#include <tf2/exceptions.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <vdb_mapping/OccupancyVDBMapping.h>
#include <vdb_mapping/VDBMapping.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vdb_mapping_interfaces/srv/add_points_to_grid.hpp>
#include <vdb_mapping_interfaces/srv/get_map_section.hpp>
#include <vdb_mapping_interfaces/srv/get_occ_grid.hpp>
#include <vdb_mapping_interfaces/srv/load_map.hpp>
#include <vdb_mapping_interfaces/srv/load_map_from_pcd.hpp>
#include <vdb_mapping_interfaces/srv/raytrace.hpp>
#include <vdb_mapping_interfaces/srv/remove_points_from_grid.hpp>
#include <vdb_mapping_interfaces/srv/trigger_map_section_update.hpp>
#include <visualization_msgs/msg/marker.hpp>

#define BOOST_BIND_NO_PLACEHOLDERS
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vdb_mapping_ros2/VDBMappingTools.hpp>

struct RemoteSource
{
  rclcpp::Subscription<vdb_mapping_interfaces::msg::UpdateGrid>::SharedPtr map_update_sub;
  rclcpp::Subscription<vdb_mapping_interfaces::msg::UpdateGrid>::SharedPtr map_overwrite_sub;
  rclcpp::Subscription<vdb_mapping_interfaces::msg::UpdateGrid>::SharedPtr map_section_sub;
  rclcpp::Client<vdb_mapping_interfaces::srv::GetMapSection>::SharedPtr get_map_section_client;
  bool apply_remote_updates;
  bool apply_remote_overwrites;
  bool apply_remote_sections;
};

struct SensorSource
{
  std::string topic;
  std::string sensor_origin_frame;
  double max_range;
};

template <typename VDBMappingT>
class VDBMappingROS2 : public rclcpp::Node
{
public:
  /*!
   * \brief Creates a new VDBMappingROS instance
   */
  explicit VDBMappingROS2(const rclcpp::NodeOptions& options);
  virtual ~VDBMappingROS2(){};

  /*!
   * \brief Resets the current map
   */
  void resetMap();
  /*!
   * \brief Saves the current map
   */
  bool saveMap(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
               const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  /*!
   * \brief Saves the active values of the current map as PCD file
   */
  bool saveMapToPCD(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                    const std::shared_ptr<std_srvs::srv::Trigger::Response> res);
  /*!
   * \brief Load stored map
   */
  bool loadMap(const std::shared_ptr<vdb_mapping_interfaces::srv::LoadMap::Request> req,
               const std::shared_ptr<vdb_mapping_interfaces::srv::LoadMap::Response> res);

  /*!
   * \brief Load stored map
   */
  bool
  loadMapFromPCD(const std::shared_ptr<vdb_mapping_interfaces::srv::LoadMapFromPCD::Request> req,
                 const std::shared_ptr<vdb_mapping_interfaces::srv::LoadMapFromPCD::Response> res);
  /*!
   * \brief Sensor callback for Pointclouds
   *
   * If the sensor_origin_frame is not empty it will be used instead of the frame id
   * of the input cloud as origin of the raycasting
   *
   * \param cloud_msg PointCloud message
   * \param sensor_source Sensor source corresponding to the Pointcloud
   */
  void cloudCallback(const std::shared_ptr<sensor_msgs::msg::PointCloud2> cloud_msg,
                     const SensorSource& sensor_source);
  /*!
   * \brief Integrating the transformed pointcloud and sensor origins into the core mapping library
   *
   *
   * \param cloud Point cloud transformed into map coordinates
   * \param tf Sensor transform in map coordinates
   */
  void insertPointCloud(const typename VDBMappingT::PointCloudT::Ptr cloud,
                        const geometry_msgs::msg::TransformStamped transform);

  void publishUpdates(typename VDBMappingT::UpdateGridT::Ptr update,
                      typename VDBMappingT::UpdateGridT::Ptr overwrite,
                      rclcpp::Time stamp);
  /*!
   * \brief Publishes a marker array and pointcloud representation of the map
   */
  void publishMap() const;
  /*!
   * \brief Listens to map updates and creats a map from these
   *
   * \param update_msg Single map update from a remote mapping instance
   */
  void mapUpdateCallback(const std::shared_ptr<vdb_mapping_interfaces::msg::UpdateGrid> update_msg);

  /*!
   * \brief Listens to map overwrites and creates a map from these
   *
   * \param update_msg Single map overwrite from a remote mapping instance
   */
  void
  mapOverwriteCallback(const std::shared_ptr<vdb_mapping_interfaces::msg::UpdateGrid> update_msg);

  void
  mapSectionCallback(const std::shared_ptr<vdb_mapping_interfaces::msg::UpdateGrid> update_msg);

  /*!
   * \brief Get the map frame name
   *
   * \returns Map frame name
   */
  const std::string& getMapFrame() const;

  /*!
   * \brief Returns the map
   *
   * \returns VDB map
   */
  std::shared_ptr<VDBMappingT> getMap();

  /*!
   * \brief Returns the map
   *
   * \returns VDB map
   */
  const std::shared_ptr<VDBMappingT> getMap() const;

  /*!
   * \brief Callback for map reset service call
   *
   * \param res result of the map reset
   * \returns result of map reset
   */
  bool resetMapCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> req,
                        const std::shared_ptr<std_srvs::srv::Trigger::Response> res);

  /*!
   * \brief Callback for requesting parts of the map
   *
   * \param req Coordinates and reference of the map section
   * \param res Result of section request, which includes the returned map
   *
   * \returns Result of section request
   */
  bool getMapSectionCallback(
    const std::shared_ptr<vdb_mapping_interfaces::srv::GetMapSection::Request> req,
    const std::shared_ptr<vdb_mapping_interfaces::srv::GetMapSection::Response> res);

  /*!
   * \brief Callback for triggering a map section request on a remote source
   *
   * \param req Coordinates, reference frame and remote source identifier of the map section
   * \param res Result of triggering section request
   *
   * \returns Result of triggering section request
   */
  bool triggerMapSectionUpdateCallback(
    const std::shared_ptr<vdb_mapping_interfaces::srv::TriggerMapSectionUpdate::Request> req,
    const std::shared_ptr<vdb_mapping_interfaces::srv::TriggerMapSectionUpdate::Response> res);

  /*!
   * \brief Callback for adding points directly into the grid
   *
   * \param req Pointcloud which should be added into the grid
   * \param res Result of adding points request
   *
   * \returns Result of adding points request
   */
  bool addPointsToGridCallback(
    const std::shared_ptr<vdb_mapping_interfaces::srv::AddPointsToGrid::Request> req,
    const std::shared_ptr<vdb_mapping_interfaces::srv::AddPointsToGrid::Response> res);

  /*!
   * \brief Callback for removing points directly from the grid
   *
   * \param req Pointcloud which should be removed from the grid
   * \param res Result of removing points request
   *
   * \returns Result of removing points request
   */
  bool removePointsFromGridCallback(
    const std::shared_ptr<vdb_mapping_interfaces::srv::RemovePointsFromGrid::Request> req,
    const std::shared_ptr<vdb_mapping_interfaces::srv::RemovePointsFromGrid::Response> res);

  /*!
   * \brief Callback for occupancy grid service call
   *
   * \param req Trigger request
   * \param res current occupancy grid
   * \returns current occupancy grid
   */
  bool
  occGridGenCallback(const std::shared_ptr<vdb_mapping_interfaces::srv::GetOccGrid::Request> req,
                     const std::shared_ptr<vdb_mapping_interfaces::srv::GetOccGrid::Response> res);

  /*!
   * \brief Callback for raytrace service call
   *
   * \param req Origin and direction for raytracing
   * \param res Resulting point of the raytrace
   *
   * \returns result of raytrace service
   */
  bool raytraceCallback(const std::shared_ptr<vdb_mapping_interfaces::srv::Raytrace::Request> req,
                        const std::shared_ptr<vdb_mapping_interfaces::srv::Raytrace::Response> res);

  void visualizationTimerCallback();
  void accumulationUpdateTimerCallback();
  void sectionTimerCallback();

private:
  std::vector<rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr> m_cloud_subs;
  /*!
   * \brief Subscriber for raw pointclouds
   */
  // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_sensor_cloud_sub;
  /*!
   * \brief Subscriber for scan aligned pointclouds
   */
  // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_aligned_cloud_sub;
  /*!
   * \brief Publisher for the marker array
   */
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_visualization_marker_pub;
  /*!
   * \brief Publisher for the point cloud
   */
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointcloud_pub;
  /*!
   * \brief Publisher for map updates
   */
  rclcpp::Publisher<vdb_mapping_interfaces::msg::UpdateGrid>::SharedPtr m_map_update_pub;
  /*!
   * \brief Publisher for map overwrites
   */
  rclcpp::Publisher<vdb_mapping_interfaces::msg::UpdateGrid>::SharedPtr m_map_overwrite_pub;
  /*!
   * \brief Publisher for map sections
   */
  rclcpp::Publisher<vdb_mapping_interfaces::msg::UpdateGrid>::SharedPtr m_map_section_pub;
  /*!
   * \brief Saves map in specified path from parameter server
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_save_map_service;
  /*!
   * \brief Saves the active values of the map as PCD file in the specified path from the paramter
   * server
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_save_map_to_pcd_service;
  /*!
   * \brief Loads a map from specified path from service
   */
  rclcpp::Service<vdb_mapping_interfaces::srv::LoadMap>::SharedPtr m_load_map_service;
  /*!
   * \brief Generates a map from a PCD file specified by the path from service
   */
  rclcpp::Service<vdb_mapping_interfaces::srv::LoadMapFromPCD>::SharedPtr
    m_load_map_from_pcd_service;
  /*!
   * \brief Service for reset map
   */
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr m_reset_map_service;
  /*!
   * \brief Service for dynamic reconfigure of parameters
   */
  // TODO
  /*!
   * \brief Service to request an occupancy grid based on the current VDB map
   */
  rclcpp::Service<vdb_mapping_interfaces::srv::GetOccGrid>::SharedPtr m_occupancy_grid_service;
  /*!
   * \brief Service for raytracing
   */
  rclcpp::Service<vdb_mapping_interfaces::srv::Raytrace>::SharedPtr m_raytrace_service;
  /*!
   * \brief Service for map section requests
   */
  rclcpp::Service<vdb_mapping_interfaces::srv::GetMapSection>::SharedPtr m_get_map_section_service;
  /*!
   * \brief Service for triggering the map section request on a remote source
   */
  rclcpp::Service<vdb_mapping_interfaces::srv::TriggerMapSectionUpdate>::SharedPtr
    m_trigger_map_section_update_service;
  /*!
   * \brief Service for adding points directly into the grid.
   */
  rclcpp::Service<vdb_mapping_interfaces::srv::AddPointsToGrid>::SharedPtr
    m_add_points_to_grid_service;
  /*!
   * \brief Service for removing points directly from the grid.
   */
  rclcpp::Service<vdb_mapping_interfaces::srv::RemovePointsFromGrid>::SharedPtr
    m_remove_points_from_grid_service;
  /*!
   * \brief Transformation buffer
   */
  std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;
  /*!
   * \brief Transformation listener
   */
  std::shared_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};
  /*!
   * \brief Grid cell resolution
   */
  double m_resolution;
  /*!
   * \brief Map Frame
   */
  std::string m_map_frame;
  /*!
   * \brief Robot Frame
   */
  std::string m_robot_frame;
  /*!
   * \brief Map pointer
   */
  std::shared_ptr<VDBMappingT> m_vdb_map;
  /*!
   * \brief Map configuration
   */
  vdb_mapping::Config m_config;
  /*!
   * \brief Specifies whether a pointcloud should be published or not
   */
  bool m_publish_pointcloud;
  /*!
   * \brief Specifies whether the map should be published as markers or not
   */
  bool m_publish_vis_marker;
  /*!
   * \brief Specifies whether the mapping publishes map updates for remote use
   */
  bool m_publish_updates;
  /*!
   * \brief Specifies whether the mapping publishes map overwrites for remote use
   */
  bool m_publish_overwrites;
  /*!
   * \brief Specifies whether the mapping publishes map sections for remote use
   */
  bool m_publish_sections;
  /*!
   * \brief Specifies whether the mapping applies raw sensor data
   */
  bool m_apply_raw_sensor_data;
  /*!
   * \brief Map of remote mapping source connections
   */
  std::map<std::string, RemoteSource> m_remote_sources;
  /*!
   * \brief Timer for map visualization
   */
  rclcpp::TimerBase::SharedPtr m_visualization_timer;
  /*!
   * \brief Specifies whether the sensor data is accumulated before updating the map
   */
  bool m_accumulate_updates;
  /*!
   * \brief Timer for integrating accumulated data
   */
  rclcpp::TimerBase::SharedPtr m_accumulation_update_timer;
  /*!
   * \brief Timer for publishing map sections
   */
  rclcpp::TimerBase::SharedPtr m_section_timer;
  /*!
   * \brief Min Coordinate of the section update bounding box
   */
  Eigen::Matrix<double, 3, 1> m_section_min_coord;
  /*!
   * \brief Max Coordinate of the section update bounding box
   */
  Eigen::Matrix<double, 3, 1> m_section_max_coord;
  /*!
   * \brief Reference Frame for the section update
   */
  std::string m_section_update_frame;
  /*!
   * \brief Specifies the number of voxels which count as occupied for the occupancy grid
   */
  int m_two_dim_projection_threshold;

  /*!
   * \brief Compression level used for creating the byte array message.
   */
  unsigned int m_compression_level = 1;
  std::shared_ptr<rclcpp::ParameterEventHandler> m_param_sub;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> m_z_min_param_handle;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> m_z_max_param_handle;
  double m_lower_visualization_z_limit;
  double m_upper_visualization_z_limit;
};


#include "VDBMappingROS2_impl.hpp"
#endif /* VDB_MAPPING_ROS22_VDBMAPPINGROS2_HPP_INCLUDED */
