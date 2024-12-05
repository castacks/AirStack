// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2021 FZI Forschungszentrum Informatik
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
 * \author  Lennart Puck puck@fzi.de
 * \date    2021-04-29
 *
 */
//----------------------------------------------------------------------
#ifndef VDB_MAPPING_OCCUPANCY_VDB_MAPPING_H_INCLUDED
#define VDB_MAPPING_OCCUPANCY_VDB_MAPPING_H_INCLUDED

#include "vdb_mapping/VDBMapping.h"

namespace vdb_mapping {

/*!
 * \brief Accumulation of configuration parameters
 */
struct Config : BaseConfig
{
  double prob_hit;
  double prob_miss;
  double prob_thres_min;
  double prob_thres_max;
};

class OccupancyVDBMapping : public VDBMapping<float, Config>
{
public:
  OccupancyVDBMapping(const double resolution)
    : VDBMapping<float, Config>(resolution)
  {
  }

  /*!
   * \brief Handles changing the mapping config
   *
   * \param config Configuration structure
   */
  inline void setConfig(const Config& config) override;

protected:
  inline bool updateFreeNode(float& voxel_value, bool& active) override;
  inline bool updateOccupiedNode(float& voxel_value, bool& active) override;
  inline bool setNodeToFree(float& voxel_value, bool& active) override;
  inline bool setNodeToOccupied(float& voxel_value, bool& active) override;

  inline void createMapFromPointCloud(const PointCloudT::Ptr& cloud,
                                      const bool set_background,
                                      const bool clear_map) override;

  /*!
   * \brief Probability update value for passing an obstacle
   */
  float m_logodds_hit;
  /*!
   * \brief Probability update value for passing free space
   */
  float m_logodds_miss;
  /*!
   * \brief Upper occupancy probability threshold
   */
  float m_logodds_thres_min;
  /*!
   * \brief Lower occupancy probability threshold
   */
  float m_logodds_thres_max;
  /*!
   * \brief Maximum clamping point for logodds
   */
  float m_max_logodds;
  /*!
   * \brief Minimum clamping point for logodds
   */
  float m_min_logodds;
};


} // namespace vdb_mapping

#include "OccupancyVDBMapping.hpp"
#endif /* VDB_MAPPING_OCCUPANCY_VDB_MAPPING_H_INCLUDED */
