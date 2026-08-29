// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <vector>
#include <memory>
#include <mutex>

#include <Eigen/Dense>
#include <pcl/common/common_headers.h>

#include "voxel_grid/voxel_grid.h"

namespace global_mapper {

class PlanningGrids {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual ~PlanningGrids() = default;
  virtual void UpdateData(const PlanningGrids &planning_grids) {
    std::lock_guard<std::mutex> lock_guard(data_mutex_);
    occupancy_grid_ = planning_grids.occupancy_grid_;
    distance_grid_ = planning_grids.distance_grid_;
    cost_grid_ = planning_grids.cost_grid_;
    occupancy_threshold_ = planning_grids.occupancy_threshold_;
    max_squared_distance_ = planning_grids.max_squared_distance_;
    goal_ = planning_grids.goal_;
  }
  
  int GetCost(const Eigen::Vector3d &position) {
    std::lock_guard<std::mutex> lock_guard(data_mutex_);
    return cost_grid_.ReadValue(position[0], position[1], position[2]);
  }

  int GetSquaredDistance(const Eigen::Vector3d &position) {
    std::lock_guard<std::mutex> lock_guard(data_mutex_);
    return distance_grid_.ReadValue(position[0], position[1], position[2]);
  }

  float GetOccupancy(const Eigen::Vector3d &position) {
    std::lock_guard<std::mutex> lock_guard(data_mutex_);
    return occupancy_grid_.ReadValue(position[0], position[1], position[2]);
  }

  bool IsOccupied(const Eigen::Vector3d &position) {
    std::lock_guard<std::mutex> lock_guard(data_mutex_);
    return occupancy_grid_.ReadValue(position[0], position[1], position[2]) > occupancy_threshold_;
  }

  bool IsOccupied(const Eigen::Vector3i &position) {
    std::lock_guard<std::mutex> lock_guard(data_mutex_);
    return occupancy_grid_.ReadValue(position.data()) > occupancy_threshold_;
  }

  bool IsInMap(const Eigen::Vector3d &position) {
    std::lock_guard<std::mutex> lock_guard(data_mutex_);
    double xyz[3] = {position[0], position[1], position[2]};
    return occupancy_grid_.IsInMap(xyz);
  }

  bool IsUnknown(const Eigen::Vector3d &position) {
    std::lock_guard<std::mutex> lock_guard(data_mutex_);
    return occupancy_grid_.ReadValue(position[0], position[1], position[2]) < 0.0;
  }

  void SetOccupancyThreshold(const float occupancy_threshold) {
    std::lock_guard<std::mutex> lock_guard(data_mutex_);
    occupancy_threshold_ = occupancy_threshold;
  }

  void SetMaxSquaredDistance(const int max_squared_distance) {
    std::lock_guard<std::mutex> lock_guard(data_mutex_);
    max_squared_distance_ = max_squared_distance;
  }

  void SetOccupancyGrid(const double origin[3],
                        const int grid_dimensions[3],
                        const double resolution,
                        const std::vector<float> &data) {
    std::lock_guard<std::mutex> lock_guard(data_mutex_);
    occupancy_grid_ = voxel_grid::VoxelGrid<float>(origin, 
                                                   grid_dimensions, 
                                                   resolution, 
                                                   data);
  }
  void SetDistanceGrid(const double origin[3],
                       const int grid_dimensions[3],
                       const double resolution,
                       const std::vector<int> &data) {
    std::lock_guard<std::mutex> lock_guard(data_mutex_);
    distance_grid_ = voxel_grid::VoxelGrid<int>(origin, 
                                                grid_dimensions, 
                                                resolution, 
                                                data);
  }
  void SetCostGrid(const double origin[3],
                   const int grid_dimensions[3],
                   const double resolution,
                   const std::vector<int> &data) {
    std::lock_guard<std::mutex> lock_guard(data_mutex_);
    cost_grid_ = voxel_grid::VoxelGrid<int>(origin, 
                                            grid_dimensions, 
                                            resolution, 
                                            data);
  }
  Eigen::Vector3d GetGoal() {
    std::lock_guard<std::mutex> lock_guard(data_mutex_);
    return goal_;
  }
  void SetGoal(const Eigen::Vector3d &goal) {
    std::lock_guard<std::mutex> lock_guard(data_mutex_);
    goal_ = goal;
  }
  float GetOccupancyThreshold() {
    std::lock_guard<std::mutex> lock_guard(data_mutex_);
    return occupancy_threshold_;
  }
  int GetMaxSquaredDistance() {
    std::lock_guard<std::mutex> lock_guard(data_mutex_);
    return max_squared_distance_;
  }

  double GetResolution() const {
    return cost_grid_.GetResolution();
  }

  Eigen::Vector3i GetGridDimensions() {
    Eigen::Vector3i grid_dimensions;
    occupancy_grid_.GetGridDimensions(grid_dimensions.data());
    return grid_dimensions;
  }

  Eigen::Vector3d GridToWorld(const Eigen::Vector3i &ixyz) {
    Eigen::Vector3d xyz;
    occupancy_grid_.GridToWorld(ixyz.data(), xyz.data());
    return xyz;
  }

  pcl::PointCloud<pcl::PointXYZ> GetOccupancyPointCloud() {
    pcl::PointCloud<pcl::PointXYZ> pointcloud;
    Eigen::Vector3i grid_dimensions = GetGridDimensions();
    for (int x = 0; x < grid_dimensions[0]; x++) {
      for (int y = 0; y < grid_dimensions[1]; y++) {
        for (int z = 0; z < grid_dimensions[2]; z++) {
          Eigen::Vector3i ixyz = {x, y, z};
          if (IsOccupied(ixyz)) {
            Eigen::Vector3d xyz = GridToWorld(ixyz);
            pointcloud.push_back(pcl::PointXYZ(xyz[0], xyz[1], xyz[2]));
          }
        }
      }
    }

    return pointcloud;
  }

 private:
  voxel_grid::VoxelGrid<float> occupancy_grid_;
  voxel_grid::VoxelGrid<int> distance_grid_;
  voxel_grid::VoxelGrid<int> cost_grid_;
  float occupancy_threshold_;  // between 0 and 1
  int max_squared_distance_;  // in voxels
  Eigen::Vector3d goal_;
  std::mutex data_mutex_;
};

}  // namespace global_mapper
