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
 * \date    2020-12-23
 *
 */
//----------------------------------------------------------------------


#include <iostream>

template <typename TData, typename TConfig>
VDBMapping<TData, TConfig>::VDBMapping(const double resolution)
  : m_resolution(resolution)
  , m_config_set(false)
{
  // Initialize Grid
  openvdb::initialize();
  if (!GridT::isRegistered())
  {
    GridT::registerGrid();
  }
  if (!UpdateGridT::isRegistered())
  {
    UpdateGridT::registerGrid();
  }
  m_vdb_grid    = createVDBMap(m_resolution);
  m_update_grid = UpdateGridT::create(false);
}

template <typename TData, typename TConfig>
void VDBMapping<TData, TConfig>::resetMap()
{
  m_vdb_grid->clear();
  m_vdb_grid    = createVDBMap(m_resolution);
  m_update_grid = UpdateGridT::create(false);
}


template <typename TData, typename TConfig>
bool VDBMapping<TData, TConfig>::saveMap() const
{
  auto timestamp     = std::chrono::system_clock::now();
  std::time_t now_tt = std::chrono::system_clock::to_time_t(timestamp);
  std::tm tm         = *std::localtime(&now_tt);
  std::stringstream sstime;
  sstime << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");

  std::string map_name = m_map_directory_path + sstime.str() + "_map.vdb";
  std::cout << map_name << std::endl;
  openvdb::io::File file_handle(map_name);
  openvdb::GridPtrVec grids;
  grids.push_back(m_vdb_grid);
  file_handle.write(grids);
  file_handle.close();
  return true;
}

template <typename TData, typename TConfig>
bool VDBMapping<TData, TConfig>::saveMapToPCD()
{
  auto timestamp     = std::chrono::system_clock::now();
  std::time_t now_tt = std::chrono::system_clock::to_time_t(timestamp);
  std::tm tm         = *std::localtime(&now_tt);
  std::stringstream sstime;
  sstime << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
  std::string pcd_path = m_map_directory_path + sstime.str() + "_active_values_map.pcd";

  PointCloudT::Ptr cloud(new PointCloudT);

  cloud->points.reserve(m_vdb_grid->activeVoxelCount());

  for (typename GridT::ValueOnCIter iter = m_vdb_grid->cbeginValueOn(); iter; ++iter)
  {
    openvdb::Vec3d world_coord = m_vdb_grid->indexToWorld(iter.getCoord());

    world_coord += 0.5 * m_resolution;
    PointT point;
    point.x = static_cast<float>(world_coord.x());
    point.y = static_cast<float>(world_coord.y());
    point.z = static_cast<float>(world_coord.z());
    cloud->points.push_back(point);
  }

  cloud->points.shrink_to_fit();
  cloud->width  = cloud->points.size();
  cloud->height = 1;

  if (pcl::io::savePCDFile(pcd_path, *cloud) != 0)
  {
    PCL_ERROR("Could not write PCD file.");
    return false;
  }
  std::cout << "Wrote pcd to: " << pcd_path << std::endl;
  return true;
}

template <typename TData, typename TConfig>
bool VDBMapping<TData, TConfig>::loadMap(const std::string& file_path)
{
  openvdb::io::File file_handle(file_path);
  file_handle.open();
  openvdb::GridBase::Ptr base_grid;
  for (openvdb::io::File::NameIterator name_iter = file_handle.beginName();
       name_iter != file_handle.endName();
       ++name_iter)
  {
    base_grid = file_handle.readGrid(name_iter.gridName());
    m_vdb_grid->clear();
    m_vdb_grid = openvdb::gridPtrCast<GridT>(base_grid);
  }
  file_handle.close();


  return true;
}

template <typename TData, typename TConfig>
bool VDBMapping<TData, TConfig>::loadMapFromPCD(const std::string& file_path,
                                                const bool set_background,
                                                const bool clear_map)
{
  PointCloudT::Ptr cloud(new PointCloudT);
  if (pcl::io::loadPCDFile<PointT>(file_path, *cloud) == -1)
  {
    PCL_ERROR("Could not open PCD file");
    return false;
  }
  createMapFromPointCloud(cloud, set_background, clear_map);
  return true;
}

template <typename TData, typename TConfig>
typename VDBMapping<TData, TConfig>::GridT::Ptr
VDBMapping<TData, TConfig>::createVDBMap(double resolution)
{
  typename GridT::Ptr new_map = GridT::create(TData());
  new_map->setTransform(openvdb::math::Transform::createLinearTransform(m_resolution));
  new_map->setGridClass(openvdb::GRID_LEVEL_SET);
  return new_map;
}

template <typename TData, typename TConfig>
openvdb::BBoxd VDBMapping<TData, TConfig>::createWorldBoundingBox(
  const Eigen::Matrix<double, 3, 1>& min_boundary,
  const Eigen::Matrix<double, 3, 1>& max_boundary,
  const Eigen::Matrix<double, 4, 4>& map_to_reference_tf) const
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr corners(new pcl::PointCloud<pcl::PointXYZ>());
  corners->points.emplace_back(static_cast<float>(min_boundary.x()),
                               static_cast<float>(min_boundary.y()),
                               static_cast<float>(min_boundary.z()));
  corners->points.emplace_back(static_cast<float>(min_boundary.x()),
                               static_cast<float>(min_boundary.y()),
                               static_cast<float>(max_boundary.z()));
  corners->points.emplace_back(static_cast<float>(min_boundary.x()),
                               static_cast<float>(max_boundary.y()),
                               static_cast<float>(min_boundary.z()));
  corners->points.emplace_back(static_cast<float>(min_boundary.x()),
                               static_cast<float>(max_boundary.y()),
                               static_cast<float>(max_boundary.z()));
  corners->points.emplace_back(static_cast<float>(max_boundary.x()),
                               static_cast<float>(min_boundary.y()),
                               static_cast<float>(min_boundary.z()));
  corners->points.emplace_back(static_cast<float>(max_boundary.x()),
                               static_cast<float>(min_boundary.y()),
                               static_cast<float>(max_boundary.z()));
  corners->points.emplace_back(static_cast<float>(max_boundary.x()),
                               static_cast<float>(max_boundary.y()),
                               static_cast<float>(min_boundary.z()));
  corners->points.emplace_back(static_cast<float>(max_boundary.x()),
                               static_cast<float>(max_boundary.y()),
                               static_cast<float>(max_boundary.z()));
  pcl::transformPointCloud(*corners, *corners, map_to_reference_tf);
  pcl::PointXYZ min_pt;
  pcl::PointXYZ max_pt;
  pcl::getMinMax3D(*corners, min_pt, max_pt);

  return openvdb::BBoxd(openvdb::Vec3d(min_pt.x, min_pt.y, min_pt.z),
                        openvdb::Vec3d(max_pt.x, max_pt.y, max_pt.z));
}

template <typename TData, typename TConfig>
openvdb::CoordBBox VDBMapping<TData, TConfig>::createIndexBoundingBox(
  const Eigen::Matrix<double, 3, 1>& min_boundary,
  const Eigen::Matrix<double, 3, 1>& max_boundary,
  const Eigen::Matrix<double, 4, 4>& map_to_reference_tf) const
{
  openvdb::BBoxd world_bb = createWorldBoundingBox(min_boundary, max_boundary, map_to_reference_tf);


  openvdb::Vec3d min_index = m_vdb_grid->worldToIndex(world_bb.min());
  openvdb::Vec3d max_index = m_vdb_grid->worldToIndex(world_bb.max());

  return {openvdb::Coord::floor(min_index), openvdb::Coord::floor(max_index)};
}

template <typename TData, typename TConfig>
typename VDBMapping<TData, TConfig>::UpdateGridT::Ptr
VDBMapping<TData, TConfig>::getMapSectionUpdateGrid(
  const Eigen::Matrix<double, 3, 1>& min_boundary,
  const Eigen::Matrix<double, 3, 1>& max_boundary,
  const Eigen::Matrix<double, 4, 4>& map_to_reference_tf) const
{
  return getMapSection<typename VDBMapping<TData, TConfig>::UpdateGridT>(
    min_boundary, max_boundary, map_to_reference_tf);
}

template <typename TData, typename TConfig>
typename VDBMapping<TData, TConfig>::GridT::Ptr VDBMapping<TData, TConfig>::getMapSectionGrid(
  const Eigen::Matrix<double, 3, 1>& min_boundary,
  const Eigen::Matrix<double, 3, 1>& max_boundary,
  const Eigen::Matrix<double, 4, 4>& map_to_reference_tf) const
{
  return getMapSection<typename VDBMapping<TData, TConfig>::GridT>(
    min_boundary, max_boundary, map_to_reference_tf);
}

template <typename TData, typename TConfig>
template <typename TResultGrid>
typename TResultGrid::Ptr VDBMapping<TData, TConfig>::getMapSection(
  const Eigen::Matrix<double, 3, 1>& min_boundary,
  const Eigen::Matrix<double, 3, 1>& max_boundary,
  const Eigen::Matrix<double, 4, 4>& map_to_reference_tf) const
{
  typename TResultGrid::Ptr temp_grid = TResultGrid::create(false);
  temp_grid->setTransform(openvdb::math::Transform::createLinearTransform(m_resolution));

  typename TResultGrid::Accessor temp_acc = temp_grid->getAccessor();

  openvdb::CoordBBox bounding_box(
    createIndexBoundingBox(min_boundary, max_boundary, map_to_reference_tf));

  for (auto iter = m_vdb_grid->cbeginValueOn(); iter; ++iter)
  {
    if (bounding_box.isInside(iter.getCoord()))
    {
      temp_acc.setValueOn(iter.getCoord(), true);
    }
  }

  openvdb::Vec3d min(bounding_box.min().x(), bounding_box.min().y(), bounding_box.min().z());
  openvdb::Vec3d max(bounding_box.max().x(), bounding_box.max().y(), bounding_box.max().z());
  temp_grid->insertMeta("bb_min", openvdb::Vec3DMetadata(min));
  temp_grid->insertMeta("bb_max", openvdb::Vec3DMetadata(max));
  return temp_grid;
}

template <typename TData, typename TConfig>
void VDBMapping<TData, TConfig>::applyMapSectionGrid(
  const typename VDBMapping<TData, TConfig>::GridT::Ptr section)
{
  applyMapSection<typename VDBMapping<TData, TConfig>::GridT>(section);
}

template <typename TData, typename TConfig>
void VDBMapping<TData, TConfig>::applyMapSectionUpdateGrid(
  const typename VDBMapping<TData, TConfig>::UpdateGridT::Ptr section)
{
  applyMapSection<typename VDBMapping<TData, TConfig>::UpdateGridT>(section);
}

template <typename TData, typename TConfig>
template <typename TSectionGrid>
void VDBMapping<TData, TConfig>::applyMapSection(typename TSectionGrid::Ptr section)
{
  typename TSectionGrid::Accessor section_acc = section->getAccessor();
  typename GridT::Accessor acc                = m_vdb_grid->getAccessor();

  openvdb::Vec3d min = section->template metaValue<openvdb::Vec3d>("bb_min");
  openvdb::Vec3d max = section->template metaValue<openvdb::Vec3d>("bb_max");
  openvdb::CoordBBox bbox(openvdb::Coord::floor(min), openvdb::Coord::floor(max));

  for (auto iter = m_vdb_grid->cbeginValueOn(); iter; ++iter)
  {
    if (bbox.isInside(iter.getCoord()))
    {
      acc.setActiveState(iter.getCoord(), false);
    }
  }
  for (auto iter = section->cbeginValueOn(); iter; ++iter)
  {
    acc.setActiveState(iter.getCoord(), true);
  }
}


template <typename TData, typename TConfig>
bool VDBMapping<TData, TConfig>::insertPointCloud(const PointCloudT::ConstPtr& cloud,
                                                  const Eigen::Matrix<double, 3, 1>& origin)
{
  UpdateGridT::Ptr update_grid;
  UpdateGridT::Ptr overwrite_grid;

  return insertPointCloud(cloud, origin, update_grid, overwrite_grid);
}

template <typename TData, typename TConfig>
bool VDBMapping<TData, TConfig>::insertPointCloud(const PointCloudT::ConstPtr& cloud,
                                                  const Eigen::Matrix<double, 3, 1>& origin,
                                                  UpdateGridT::Ptr& update_grid,
                                                  UpdateGridT::Ptr& overwrite_grid)
{
  accumulateUpdate(cloud, origin, m_max_range);
  integrateUpdate(update_grid, overwrite_grid);
  resetUpdate();
  return true;
}

template <typename TData, typename TConfig>
bool VDBMapping<TData, TConfig>::removePointsFromGrid(const PointCloudT::ConstPtr& cloud)
{
  typename GridT::Accessor acc = m_vdb_grid->getAccessor();

  auto set_node = [&](TData& voxel_value, bool& active) { setNodeToFree(voxel_value, active); };


  for (const PointT& pt : *cloud)
  {
    openvdb::Vec3d world_pt(pt.x, pt.y, pt.z);
    openvdb::Coord index_pt = openvdb::Coord::floor(m_vdb_grid->worldToIndex(world_pt));
    acc.modifyValueAndActiveState(index_pt, set_node);
  }
  return true;
}

template <typename TData, typename TConfig>
bool VDBMapping<TData, TConfig>::addPointsToGrid(const PointCloudT::ConstPtr& cloud)
{
  typename GridT::Accessor acc = m_vdb_grid->getAccessor();
  auto set_node = [&](TData& voxel_value, bool& active) { setNodeToOccupied(voxel_value, active); };
  for (const PointT& pt : *cloud)
  {
    openvdb::Vec3d world_pt(pt.x, pt.y, pt.z);
    openvdb::Coord index_pt = openvdb::Coord::floor(m_vdb_grid->worldToIndex(world_pt));
    acc.modifyValueAndActiveState(index_pt, set_node);
  }
  return true;
}


template <typename TData, typename TConfig>
void VDBMapping<TData, TConfig>::accumulateUpdate(const PointCloudT::ConstPtr& cloud,
                                                  const Eigen::Matrix<double, 3, 1>& origin,
                                                  const double& max_range)
{
  UpdateGridT::Accessor update_grid_acc = m_update_grid->getAccessor();
  if (max_range > 0)
  {
    raycastPointCloud(cloud, origin, max_range, update_grid_acc);
  }
  else
  {
    raycastPointCloud(cloud, origin, update_grid_acc);
  }
}

template <typename TData, typename TConfig>
void VDBMapping<TData, TConfig>::integrateUpdate(UpdateGridT::Ptr& update_grid,
                                                 UpdateGridT::Ptr& overwrite_grid)
{
  overwrite_grid = updateMap(m_update_grid);
  update_grid    = m_update_grid;
}

template <typename TData, typename TConfig>
void VDBMapping<TData, TConfig>::resetUpdate()
{
  m_update_grid = UpdateGridT::create(false);
}

template <typename TData, typename TConfig>
bool VDBMapping<TData, TConfig>::raycastPointCloud(const PointCloudT::ConstPtr& cloud,
                                                   const Eigen::Matrix<double, 3, 1>& origin,
                                                   UpdateGridT::Accessor& update_grid_acc)
{
  return raycastPointCloud(cloud, origin, m_max_range, update_grid_acc);
}


template <typename TData, typename TConfig>
bool VDBMapping<TData, TConfig>::raycastPointCloud(const PointCloudT::ConstPtr& cloud,
                                                   const Eigen::Matrix<double, 3, 1>& origin,
                                                   const double raycast_range,
                                                   UpdateGridT::Accessor& update_grid_acc)
{
  // Creating a temporary grid in which the new data is casted. This way we prevent the computation
  // of redundant probability updates in the actual map

  // Check if a valid configuration was loaded
  if (!m_config_set)
  {
    std::cerr << "Map not properly configured. Did you call setConfig method?" << std::endl;
    return false;
  }

  RayT ray;
  DDAT dda;

  // Ray origin in world coordinates
  openvdb::Vec3d ray_origin_world(origin.x(), origin.y(), origin.z());
  // Ray origin in index coordinates
  openvdb::Coord buffer = openvdb::Coord::floor(m_vdb_grid->worldToIndex(ray_origin_world));
  Vec3T ray_origin_index(buffer.x(), buffer.y(), buffer.z());
  // Ray end point in world coordinates
  openvdb::Vec3d ray_end_world;

  // Raycasting of every point in the input cloud
  for (const PointT& pt : *cloud)
  {
    ray_end_world      = openvdb::Vec3d(pt.x, pt.y, pt.z);
    bool max_range_ray = false;


    if (std::isnan(ray_end_world.x()) || std::isnan(ray_end_world.y()) ||
        std::isnan(ray_end_world.z()) || std::isnan(ray_origin_world.x()) ||
        std::isnan(ray_origin_world.y()) || std::isnan(ray_origin_world.z()))
    {
      continue;
    }

    if (raycast_range > 0.0 && (ray_end_world - ray_origin_world).length() > raycast_range)
    {
      ray_end_world = ray_origin_world + (ray_end_world - ray_origin_world).unit() * raycast_range;
      max_range_ray = true;
    }

    openvdb::Coord ray_end_index =
      castRayIntoGrid(ray_origin_world, ray_origin_index, ray_end_world, update_grid_acc);

    if (!max_range_ray)
    {
      update_grid_acc.setValueOn(ray_end_index, true);
    }
  }
  return true;
}

template <typename TData, typename TConfig>
openvdb::Coord
VDBMapping<TData, TConfig>::castRayIntoGrid(const openvdb::Vec3d& ray_origin_world,
                                            const Vec3T& ray_origin_index,
                                            const openvdb::Vec3d& ray_end_world,
                                            UpdateGridT::Accessor& update_grid_acc) const
{
  // In case that a coodinate is placed directly on the grid a half the resolution is added as
  // offset to push it within the grids boundaries Currently this does not support tranlational
  // adjustments of the grid since it is not used within the vdb_mapping framework.
  openvdb::Vec3d adjusted_ray_end_world = ray_end_world;
  if (std::fmod(ray_end_world.x(), m_resolution))
  {
    adjusted_ray_end_world.x() = ray_end_world.x() + (m_resolution / 2.0);
  }
  if (std::fmod(ray_end_world.y(), m_resolution))
  {
    adjusted_ray_end_world.y() = ray_end_world.y() + (m_resolution / 2.0);
  }
  if (std::fmod(ray_end_world.z(), m_resolution))
  {
    adjusted_ray_end_world.z() = ray_end_world.z() + (m_resolution / 2.0);
  }

  openvdb::Coord ray_end_index =
    openvdb::Coord::floor(m_vdb_grid->worldToIndex(adjusted_ray_end_world));

  openvdb::Vec3d ray_direction = (ray_end_index.asVec3d() - ray_origin_index);

  // Starting the ray from the center of the voxel
  RayT ray(ray_origin_index + 0.5, ray_direction, 0, 1);
  DDAT dda(ray, 0);
  if (ray_end_index.asVec3d() != ray_origin_index)
  {
    do
    {
      update_grid_acc.setActiveState(dda.voxel(), true);
    } while (dda.step());
  }
  return ray_end_index;
}

template <typename TData, typename TConfig>
bool VDBMapping<TData, TConfig>::raytrace(const openvdb::Vec3d& ray_origin_world,
                                          const openvdb::Vec3d& ray_direction,
                                          const double max_ray_length,
                                          openvdb::Vec3d& end_point)
{
  typename GridT::Accessor acc    = m_vdb_grid->getAccessor();
  openvdb::Vec3d ray_origin_index = m_vdb_grid->worldToIndex(ray_origin_world);
  RayT ray(ray_origin_index, ray_direction);
  DDAT dda(ray);
  while (true)
  {
    double distance =
      m_resolution * std::sqrt(std::pow((dda.voxel().x() - ray_origin_index.x()), 2) +
                               std::pow((dda.voxel().y() - ray_origin_index.y()), 2) +
                               std::pow((dda.voxel().z() - ray_origin_index.z()), 2));
    if (distance < max_ray_length)
    {
      if (acc.isValueOn(dda.voxel()))
      {
        end_point = m_vdb_grid->indexToWorld(dda.voxel());
        return true;
      }
      dda.step();
    }
    else
    {
      return false;
    }
  }
}

template <typename TData, typename TConfig>
VDBMapping<TData, TConfig>::UpdateGridT::Ptr
VDBMapping<TData, TConfig>::updateMap(const UpdateGridT::Ptr& temp_grid)
{
  UpdateGridT::Ptr change          = UpdateGridT::create(false);
  UpdateGridT::Accessor change_acc = change->getAccessor();
  if (temp_grid->empty())
  {
    return change;
  }

  bool state_changed           = false;
  typename GridT::Accessor acc = m_vdb_grid->getAccessor();
  // Probability update lambda for free space grid elements
  auto miss = [&](TData& voxel_value, bool& active) {
    bool last_state = active;
    updateFreeNode(voxel_value, active);
    if (last_state != active)
    {
      state_changed = true;
    }
  };

  // Probability update lambda for occupied grid elements
  auto hit = [&](TData& voxel_value, bool& active) {
    bool last_state = active;
    updateOccupiedNode(voxel_value, active);
    if (last_state != active)
    {
      state_changed = true;
    }
  };

  // Integrating the data of the temporary grid into the map using the probability update functions
  for (UpdateGridT::ValueOnCIter iter = temp_grid->cbeginValueOn(); iter; ++iter)
  {
    state_changed = false;
    if (*iter)
    {
      acc.modifyValueAndActiveState(iter.getCoord(), hit);
      if (state_changed)
      {
        change_acc.setValueOn(iter.getCoord(), true);
      }
    }
    else
    {
      acc.modifyValueAndActiveState(iter.getCoord(), miss);
      if (state_changed)
      {
        change_acc.setActiveState(iter.getCoord(), true);
      }
    }
  }
  return change;
}

template <typename TData, typename TConfig>
void VDBMapping<TData, TConfig>::overwriteMap(const UpdateGridT::Ptr& update_grid)
{
  typename GridT::Accessor acc = m_vdb_grid->getAccessor();
  for (UpdateGridT::ValueOnCIter iter = update_grid->cbeginValueOn(); iter; ++iter)
  {
    if (*iter)
    {
      acc.setActiveState(iter.getCoord(), true);
    }
    else
    {
      acc.setActiveState(iter.getCoord(), false);
    }
  }
}

template <typename TData, typename TConfig>
std::vector<uint8_t> VDBMapping<TData, TConfig>::compressString(const std::string& string) const
{
  auto uncompressed = std::vector<uint8_t>(string.begin(), string.end());

  // Create buffer with enough size for worst case scenario
  size_t len = ZSTD_compressBound(uncompressed.size());
  std::vector<uint8_t> compressed(len);

  int ret = ZSTD_compress(
    compressed.data(), len, uncompressed.data(), uncompressed.size(), m_compression_level);


  if (ZSTD_isError(ret))
  {
    std::cerr << "Compression using ZSTD failed: " << ZSTD_getErrorName(ret)
              << " , sending uncompressed byte array" << std::endl;
    return uncompressed;
  }

  // Resize compressed buffer to actual compressed size
  compressed.resize(ret);
  return compressed;
}

template <typename TData, typename TConfig>
std::string
VDBMapping<TData, TConfig>::decompressByteArray(const std::vector<uint8_t>& byte_array) const
{
  std::size_t len = ZSTD_getDecompressedSize(byte_array.data(), byte_array.size());
  std::vector<uint8_t> uncompressed(len);

  std::size_t size =
    ZSTD_decompress(uncompressed.data(), len, byte_array.data(), byte_array.size());


  std::string map_str;
  if (ZSTD_isError(size))
  {
    std::cerr << "Could not decompress map using ZSTD failed: " << ZSTD_getErrorName(size)
              << " , returning raw data" << std::endl;
    map_str = std::string(byte_array.begin(), byte_array.end());
  }
  else
  {
    map_str = std::string(uncompressed.begin(), uncompressed.end());
  }

  return map_str;
}

template <typename TData, typename TConfig>
template <typename TGrid>
std::vector<uint8_t> VDBMapping<TData, TConfig>::gridToByteArray(const typename TGrid::Ptr grid)
{
  openvdb::GridPtrVec grids;
  grids.push_back(grid);
  std::ostringstream oss(std::ios_base::binary);
  openvdb::io::Stream(oss).write(grids);
  return compressString(oss.str());
}

template <typename TData, typename TConfig>
template <typename TGrid>
typename TGrid::Ptr
VDBMapping<TData, TConfig>::byteArrayToGrid(const std::vector<uint8_t> byte_array)
{
  std::istringstream iss(decompressByteArray(byte_array));
  openvdb::io::Stream strm(iss);
  openvdb::GridPtrVecPtr grids;
  grids = strm.getGrids();
  // This cast might fail if different VDB versions are used.
  // Corresponding error messages are generated by VDB directly
  typename TGrid::Ptr update_grid = openvdb::gridPtrCast<TGrid>(grids->front());
  return update_grid;
}

template <typename TData, typename TConfig>
void VDBMapping<TData, TConfig>::setConfig(const TConfig& config)
{
  if (config.max_range < 0.0)
  {
    std::cerr << "Max range of " << config.max_range << " invalid. Range cannot be negative."
              << std::endl;
    return;
  }
  m_max_range          = config.max_range;
  m_map_directory_path = config.map_directory_path;
  m_config_set         = true;
}
