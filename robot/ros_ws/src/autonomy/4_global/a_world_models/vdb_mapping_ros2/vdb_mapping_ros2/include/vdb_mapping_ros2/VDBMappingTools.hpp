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
 * \date    2022-05-09
 *
 */
//----------------------------------------------------------------------
#ifndef VDB_MAPPING_ROS2_VDBMAPPINGTOOLS_H_INCLUDED
#define VDB_MAPPING_ROS2_VDBMAPPINGTOOLS_H_INCLUDED
#include <geometry_msgs/msg/point.hpp>
#include <openvdb/openvdb.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>

/*!
 * \brief Collection of VDBMapping helper functions and tools
 */
template <typename VDBMappingT>
class VDBMappingTools
{
public:
  VDBMappingTools(){};
  virtual ~VDBMappingTools(){};
  /*!
   * \brief Creates output msgs for pointcloud and marker arrays
   *
   * \param grid Map grid
   * \param resolution Resolution of the grid
   * \param frame_id Frame ID of the grid
   * \param marker_msg Output Marker message
   * \param cloud_msg Output Pointcloud message
   * \param create_marker Flag specifying to create a marker message
   * \param create_pointcloud Flag specifying to create a pointcloud message
   */
  static void createMappingOutput(const typename VDBMappingT::GridT::Ptr gri,
                                  const std::string& frame_id,
                                  visualization_msgs::msg::Marker& marker_msg,
                                  sensor_msgs::msg::PointCloud2& cloud_msg,
                                  bool create_marker,
                                  bool create_pointcloud,
                                  double lower_z_limit = 0.0,
                                  double upper_z_limit = 0.0);
  /*!
   * \brief Calculates a height correlating color coding using HSV color space
   *
   * \param height Gridcell height relativ to the min and max height of the complete grid. Parameter
   * can take values between 0 and 1
   *
   * \returns RGBA color of the grid cell
   */
  static std_msgs::msg::ColorRGBA heightColorCoding(const double height);
};
#include "VDBMappingTools_impl.hpp"
#endif /* VDB_MAPPING_ROS2_VDBMAPPINGTOOLS_H_INCLUDED */
