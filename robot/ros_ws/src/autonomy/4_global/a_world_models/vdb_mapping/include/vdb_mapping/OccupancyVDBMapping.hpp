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


#include "vdb_mapping/OccupancyVDBMapping.h"

namespace vdb_mapping {

bool OccupancyVDBMapping::updateFreeNode(float& voxel_value, bool& active)
{
  voxel_value += m_logodds_miss;
  if (voxel_value < m_logodds_thres_min)
  {
    active = false;
    if (voxel_value < m_min_logodds)
    {
      voxel_value = m_min_logodds;
    }
  }
  return true;
}

bool OccupancyVDBMapping::updateOccupiedNode(float& voxel_value, bool& active)
{
  voxel_value += m_logodds_hit;
  if (voxel_value > m_logodds_thres_max)
  {
    active = true;
    if (voxel_value > m_max_logodds)
    {
      voxel_value = m_max_logodds;
    }
  }
  return true;
}

bool OccupancyVDBMapping::setNodeToFree(float& voxel_value, bool& active)
{
  voxel_value = m_min_logodds;
  active      = false;
  return true;
}

bool OccupancyVDBMapping::setNodeToOccupied(float& voxel_value, bool& active)
{
  voxel_value = m_max_logodds;
  active      = true;
  return true;
}

void OccupancyVDBMapping::createMapFromPointCloud(const PointCloudT::Ptr& cloud,
                                                  const bool set_background,
                                                  const bool clear_map)
{
  if (clear_map)
  {
    m_vdb_grid->clear();
  }

  typename GridT::Accessor acc = m_vdb_grid->getAccessor();

  for (auto point : cloud->points)
  {
    openvdb::Vec3d index_coord =
      m_vdb_grid->worldToIndex(openvdb::Vec3d(point.x, point.y, point.z));
    acc.setValueOn(openvdb::Coord::floor(index_coord), m_max_logodds);
  }
  openvdb::CoordBBox bbox = m_vdb_grid->evalActiveVoxelBoundingBox();

  if (set_background)
  {
    for (int x = bbox.min().x(); x <= bbox.max().x(); ++x)
    {
      for (int y = bbox.min().y(); y <= bbox.max().y(); ++y)
      {
        for (int z = bbox.min().z(); z <= bbox.max().z(); ++z)
        {
          openvdb::Coord index_coord = openvdb::Coord(x, y, z);
          if (!acc.isValueOn(index_coord))
          {
            acc.setValueOff(index_coord, m_min_logodds);
          }
        }
      }
    }
  }

  m_vdb_grid->pruneGrid();
}

void OccupancyVDBMapping::setConfig(const Config& config)
{
  // call base class function
  VDBMapping::setConfig(config);

  // Sanity Check for input config
  if (config.prob_miss > 0.5)
  {
    std::cerr << "Probability for a miss should be below 0.5 but is " << config.prob_miss
              << std::endl;
    return;
  }
  if (config.prob_hit < 0.5)
  {
    std::cerr << "Probability for a hit should be above 0.5 but is " << config.prob_hit
              << std::endl;
    return;
  }

  // Store probabilities as log odds
  m_logodds_miss = static_cast<float>(log(config.prob_miss) - log(1 - config.prob_miss));
  m_logodds_hit  = static_cast<float>(log(config.prob_hit) - log(1 - config.prob_hit));
  m_logodds_thres_min =
    static_cast<float>(log(config.prob_thres_min) - log(1 - config.prob_thres_min));
  m_logodds_thres_max =
    static_cast<float>(log(config.prob_thres_max) - log(1 - config.prob_thres_max));
  // Values to clamp the logodds in order to prevent non dynamic map behavior
  m_max_logodds = static_cast<float>(log(0.99) - log(0.01));
  m_min_logodds = static_cast<float>(log(0.01) - log(0.99));
  m_config_set  = true;
}

} // namespace vdb_mapping
