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
#ifndef VDB_MAPPING_VDB_MAPPING_H_INCLUDED
#define VDB_MAPPING_VDB_MAPPING_H_INCLUDED


#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <chrono>
#include <eigen3/Eigen/Geometry>

#include <openvdb/Types.h>
#include <openvdb/io/Stream.h>
#include <openvdb/math/DDA.h>
#include <openvdb/math/Ray.h>
#include <openvdb/openvdb.h>
#include <openvdb/tools/Clip.h>
#include <openvdb/tools/Morphology.h>

#include <zstd.h>

namespace vdb_mapping {


/*!
 * \brief Accumulation of configuration parameters
 */
struct BaseConfig
{
  double max_range;
  std::string map_directory_path;
};
/*!
 * \brief Main Mapping class which handles all data integration
 */
template <typename TData, typename TConfig = BaseConfig>
class VDBMapping
{
public:
  using PointT      = pcl::PointXYZ;
  using PointCloudT = pcl::PointCloud<PointT>;

  using RayT  = openvdb::math::Ray<double>;
  using Vec3T = RayT::Vec3Type;
  using DDAT  = openvdb::math::DDA<RayT, 0>;

  using GridT       = openvdb::Grid<typename openvdb::tree::Tree4<TData, 5, 4, 3>::Type>;
  using UpdateGridT = openvdb::Grid<openvdb::tree::Tree4<bool, 1, 4, 3>::Type>;


  VDBMapping()                  = delete;
  VDBMapping(const VDBMapping&) = delete;

  VDBMapping& operator=(const VDBMapping&) = delete;

  /*!
   * \brief Construktur creates a new VDBMapping objekt with parametrizable grid resolution
   *
   * \param resolution Resolution of the VDB Grid
   */
  VDBMapping(const double resolution);
  virtual ~VDBMapping() = default;

  /*!
   * \brief Creates a new VDB Grid
   *
   * \param resolution Resolution of the grid
   *
   * \returns Grid shared pointer
   */
  typename GridT::Ptr createVDBMap(double resolution);

  /*!
   * \brief Reset the current map
   */
  void resetMap();

  /*!
   * \brief Saves the current map
   */
  bool saveMap() const;

  /*!
   * \brief Saves the active values of the current map as PCD file
   *
   * \returns Saving pcd successfull
   */
  bool saveMapToPCD();

  /*!
   * \brief Loads a stored map
   */
  bool loadMap(const std::string& file_path);

  /*!
   * \brief Loads a stored map from a pcd file
   *
   * \param file_path Path to pcd file
   * \param set_background Specifies if the background should be set
   * \param clear_map Specifies if the map has to be cleared before inserting data
   *
   * \returns Loading of map successfull
   */
  bool
  loadMapFromPCD(const std::string& file_path, const bool set_background, const bool clear_map);

  /*!
   * \brief Accumulates a new sensor point cloud to the update grid
   *
   * \param cloud Input cloud in map coordinates
   * \param origin Sensor position in map coordinates
   * \param max_range Maximum raycasting range of this measurement
   */
  void accumulateUpdate(const PointCloudT::ConstPtr& cloud,
                        const Eigen::Matrix<double, 3, 1>& origin,
                        const double& max_range);

  /*!
   * \brief Integrates the accumulated updates into the map
   *
   * \param update_grid Update grid
   * \param overwrite_grid Overwrite grid
   */
  void integrateUpdate(UpdateGridT::Ptr& update_grid, UpdateGridT::Ptr& overwrite_grid);

  /*!
   * \brief Resets the updates grid
   */
  void resetUpdate();

  /*!
   * \brief Handles the integration of new PointCloud data into the VDB data structure.
   * All datapoints are raycasted starting from the origin position
   *
   * \param cloud Input cloud in map coordinates
   * \param origin Sensor position in map coordinates
   *
   * \returns Was the insertion of the new pointcloud successful
   */
  bool insertPointCloud(const PointCloudT::ConstPtr& cloud,
                        const Eigen::Matrix<double, 3, 1>& origin);

  /*!
   * \brief Handles the integration of new PointCloud data into the VDB data structure.
   * All datapoints are raycasted starting from the origin position
   *
   * \param cloud Input cloud in map coordinates
   * \param origin Sensor position in map coordinates
   * \param update_grid Update grid that was created internally while mapping
   * \param overwrite_grid Overwrite grid containing all changed voxel indices
   *
   * \returns Was the insertion of the new pointcloud successful
   */
  bool insertPointCloud(const PointCloudT::ConstPtr& cloud,
                        const Eigen::Matrix<double, 3, 1>& origin,
                        UpdateGridT::Ptr& update_grid,
                        UpdateGridT::Ptr& overwrite_grid);

  bool removePointsFromGrid(const PointCloudT::ConstPtr& cloud);
  bool addPointsToGrid(const PointCloudT::ConstPtr& cloud);

  /*!
   * \brief  Raycasts a Pointcloud into an update Grid
   *
   * For each point in the input pointcloud, a raycast is performed from the origin.
   * All cells along these rays are marked as active.
   *
   * All points are clipped according to the config's max_range parameter. If a point is within
   * this range, its corresponding cell value in the update grid is set to true and will be handled
   * as a sensor hit in the map update.
   *
   * \param cloud Input sensor point cloud
   * \param origin Origin of the sensor measurement
   * \param update_grid_acc Accessor to the grid in which the raycasting takes place
   *
   * \returns Raycasted update grid
   */
  bool raycastPointCloud(const PointCloudT::ConstPtr& cloud,
                         const Eigen::Matrix<double, 3, 1>& origin,
                         UpdateGridT::Accessor& update_grid_acc);

  /*!
   * \brief  Raycasts a Pointcloud into an update Grid
   * For each point in the input pointcloud, a raycast is performed from the origin.
   * All cells along these rays are marked as active.
   *
   * All points are clipped according to the config's max_range parameter. If a point is within
   * this range, its corresponding cell value in the update grid is set to true and will be handled
   * as a sensor hit in the map update.
   *
   * \param cloud Input sensor point cloud
   * \param origin Origin of the sensor measurement
   * \param raycast_range Maximum raycasting range
   * \param update_grid_acc Accessor to the grid in which the raycasting takes place
   *
   * \returns Raycasted update grid
   */
  bool raycastPointCloud(const PointCloudT::ConstPtr& cloud,
                         const Eigen::Matrix<double, 3, 1>& origin,
                         const double raycast_range,
                         UpdateGridT::Accessor& update_grid_acc);

  /*!
   * \brief Casts a single ray into an update grid structure
   *
   * Each cell along the ray is marked as active.
   *
   * \param ray_origin_world Ray origin in world coordinates
   * \param ray_origin_index Ray origin in index coordinates
   * \param ray_end_world Ray endpoint in world coordinates
   * \param update_grid_acc Accessor to the update grid
   *
   * \returns Final visited index coordinate
   */
  openvdb::Coord castRayIntoGrid(const openvdb::Vec3d& ray_origin_world,
                                 const Vec3T& ray_origin_index,
                                 const openvdb::Vec3d& ray_end_world,
                                 UpdateGridT::Accessor& update_grid_acc) const;

  bool raytrace(const openvdb::Vec3d& ray_origin_world,
                const openvdb::Vec3d& ray_direction,
                const double max_ray_length,
                openvdb::Vec3d& end_point);

  /*!
   * \brief Overwrites the active states of a map given an update grid
   *
   * \param update_grid Update Grid containing all states that changed during the last update
   */
  void overwriteMap(const UpdateGridT::Ptr& update_grid);

  /*!
   * \brief Incorporates the information of an update grid to the internal map. This will update the
   * probabilities of all cells specified by the update grid.
   *
   * \param temp_grid Grid containing all cells which shall be updated
   *
   * \returns Was the insertion of the pointcloud successuff
   */
  UpdateGridT::Ptr updateMap(const UpdateGridT::Ptr& temp_grid);

  /*!
   * \brief Returns a pointer to the VDB map structure
   *
   * \returns Map pointer
   */
  typename GridT::Ptr getGrid() const { return m_vdb_grid; }

  /*!
   * \brief Creates a world coordinate bounding box around a transform
   *
   * \param min_boundary Minimum boundary of box
   * \param max_boundary Maximum boundary of box
   * \param map_to_reference_tf Transform from map to reference frame
   *
   * \returns World coordinate bounding box
   */
  openvdb::BBoxd
  createWorldBoundingBox(const Eigen::Matrix<double, 3, 1>& min_boundary,
                         const Eigen::Matrix<double, 3, 1>& max_boundary,
                         const Eigen::Matrix<double, 4, 4>& map_to_reference_tf) const;
  /*!
   * \brief Creates an index coordinate bounding box around a transform
   *
   * \param min_boundary Minimum boundary of box
   * \param max_boundary Maximum boundary of box
   * \param map_to_reference_tf Transform from map to reference frame
   *
   * \returns Index coordinate bounding box
   */
  openvdb::CoordBBox
  createIndexBoundingBox(const Eigen::Matrix<double, 3, 1>& min_boundary,
                         const Eigen::Matrix<double, 3, 1>& max_boundary,
                         const Eigen::Matrix<double, 4, 4>& map_to_reference_tf) const;

  /*!
   * \brief Generates an update grid from a bouding box and a reference frame
   *
   * \param min_boundary Minimum boundary of the box
   * \param max_boundary Maximum boundary of the box
   * \param map_to_reference_tf Transform from map to reference frame
   *
   * \returns Update Grid containing the information within the bounding box
   */
  typename UpdateGridT::Ptr
  getMapSectionUpdateGrid(const Eigen::Matrix<double, 3, 1>& min_boundary,
                          const Eigen::Matrix<double, 3, 1>& max_boundary,
                          const Eigen::Matrix<double, 4, 4>& map_to_reference_tf) const;
  /*!
   * \brief Generates a grid from a bounding box and a reference frame
   *
   * \param min_boundary Minimum boundary of the box
   * \param max_boundary Maximum boundary of the box
   * \param map_to_reference_tf Transform from map to reference frame
   *
   * \returns Grid containing the information within the bounding box
   */
  typename GridT::Ptr
  getMapSectionGrid(const Eigen::Matrix<double, 3, 1>& min_boundary,
                    const Eigen::Matrix<double, 3, 1>& max_boundary,
                    const Eigen::Matrix<double, 4, 4>& map_to_reference_tf) const;
  /*!
   * \brief Generates a grid or update grid from a bounding box and a reference frame
   *
   * @tparam TResultGrid Resulting Grid Type
   * \param min_boundary Minimum boundary of the box
   * \param max_boundary Maximum boundary of the box
   * \param map_to_reference_tf Transform from map to reference frame
   *
   * \returns Grid/UpdateGrid containing the information within the bounding box
   */
  template <typename TResultGrid>
  typename TResultGrid::Ptr
  getMapSection(const Eigen::Matrix<double, 3, 1>& min_boundary,
                const Eigen::Matrix<double, 3, 1>& max_boundary,
                const Eigen::Matrix<double, 4, 4>& map_to_reference_tf) const;

  /*!
   * \brief Applies a map section grid to the map
   *
   * \param section Section grid containing the information about part of the map. The boundary box
   * of the section is encoded in the grids meta information
   *
   */
  void applyMapSectionGrid(const typename VDBMapping<TData, TConfig>::GridT::Ptr section);

  /*!
   * \brief Applies a map section update grid to the map
   *
   * \param section Section grid containing the information about part of the map. The boundary box
   * of the section is encoded in the grids meta information
   *
   */
  void
  applyMapSectionUpdateGrid(const typename VDBMapping<TData, TConfig>::UpdateGridT::Ptr section);

  /*!
   * \brief Applies a map section to the map
   *
   * \param section Section grid containing the information about part of the map. The boundary box
   * of the section is encoded in the grids meta information
   *
   */
  template <typename TSectionGrid>
  void applyMapSection(typename TSectionGrid::Ptr section);

  /*!
   * \brief Handles changing the mapping config
   *
   * \param config Configuration structure
   */
  virtual void setConfig(const TConfig& config);

  std::vector<uint8_t> compressString(const std::string& string) const;

  std::string decompressByteArray(const std::vector<uint8_t>& byte_array) const;

  template <typename TGrid>
  std::vector<uint8_t> gridToByteArray(typename TGrid::Ptr grid);

  template <typename TGrid>
  typename TGrid::Ptr byteArrayToGrid(std::vector<uint8_t> byte_array);


protected:
  virtual bool updateFreeNode(TData& voxel_value, bool& active) { return false; }
  virtual bool updateOccupiedNode(TData& voxel_value, bool& active) { return false; }
  virtual bool setNodeToFree(TData& voxel_value, bool& active) { return false; }
  virtual bool setNodeToOccupied(TData& voxel_value, bool& active) { return false; }

  virtual void createMapFromPointCloud(const PointCloudT::Ptr& cloud,
                                       const bool set_background,
                                       const bool clear_map)
  {
    std::cerr << "Not implemented for data type" << std::endl;
  }

  /*!
   * \brief VDB grid pointer
   */
  typename GridT::Ptr m_vdb_grid;
  /*!
   * \brief Maximum raycasting distance
   */
  double m_max_range;
  /*!
   * \brief Grid resolution of the map
   */
  double m_resolution;
  /*!
   * \brief path where the maps will be stored
   */
  std::string m_map_directory_path;
  /*!
   * \brief Flag checking wether a valid config was already loaded
   */
  bool m_config_set;

  UpdateGridT::Ptr m_update_grid;

  unsigned int m_compression_level = 1;
};

#include "VDBMapping.hpp"

} // namespace vdb_mapping

#endif /* VDB_MAPPING_VDB_MAPPING_H_INCLUDED */
