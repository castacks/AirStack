// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <csignal>
#include <array>
#include <deque>
#include <vector>
#include <thread>
#include <mutex>
#include <memory>
#include <condition_variable>
#include <chrono>

#include <pcl/common/common_headers.h>

#include "global_mapper/params.h"
#include "occupancy_grid/occupancy_grid.h"
#include "distance_grid/distance_grid.h"
#include "cost_grid/cost_grid.h"
#include "temporal_grid/temporal_grid.h"
#include "occupancy_grid_2d/occupancy_grid_2d.h"
#include "distance_grid_2d/distance_grid_2d.h"

namespace global_mapper
{
using PointCloud = pcl::PointCloud<pcl::PointXYZI>;

class GlobalMapper
{
public:
  GlobalMapper(Params& params);
  ~GlobalMapper();
  GlobalMapper(const GlobalMapper& rhs) = delete;
  GlobalMapper& operator=(const GlobalMapper& rhs) = delete;
  GlobalMapper(GlobalMapper&& rhs) = delete;
  GlobalMapper& operator=(GlobalMapper&& rhs) = delete;
  void GetVoxelGrids(voxel_grid::VoxelGrid<float>* occupancy_grid, voxel_grid::VoxelGrid<int>* distance_grid,
                     voxel_grid::VoxelGrid<int>* cost_grid, voxel_grid::VoxelGrid<std::array<double, 7>>* temporal_grid);
  void PushPointCloud(const PointCloud::ConstPtr& cloud_ptr, double timetsamp);
  void UpdateOrigin(const double xyz[3]);
  void GetOrigin(double xyz[3]) const;
  bool GetProjectedGoal(double xyz[3]) const;
  void SetGoal(const double xyz[3]);
  void GetGoal(double xyz[3]) const;
  void GetPaths(std::vector<std::array<double, 3>>* dense_path, std::vector<std::array<double, 3>>* sparse_path);
  void SetCostWeights(int distance_weight, int inflation_weight, int unknown_weight, int obstacle_weight);
  void SetObstacle(const double xyz[3]);
  void RemoveObstacle(const double xyz[3]);
  void UpdateGrids();
  void Run();

  Params params_;
  occupancy_grid::OccupancyGrid occupancy_grid_;
  distance_grid::DistanceGrid distance_grid_;
  cost_grid::CostGrid cost_grid_;
  temporal_grid::TemporalGrid temporal_grid_;
  occupancy_grid_2d::OccupancyGrid2D occupancy_grid_2d_;
  distance_grid_2d::DistanceGrid2D distance_grid_2d_;
  double timestamp_;

  std::mutex output_mutex_; // TODO Move back to private


private:
  const PointCloud::ConstPtr PopPointCloud();
  void InsertPointCloud(const PointCloud::ConstPtr& point_cloud);
  void UpdateOccupancyGrid();
  void UpdateDistanceGrid();
  void UpdateCostGrid();
  void UpdateTemporalGrid(double timestamp);
  void UpdateOccupancyGrid2D();
  void UpdateDistanceGrid2D();
  void Spin();

  std::deque<PointCloud::ConstPtr> point_cloud_buffer_;
  std::deque<double> timestamp_buffer_;

  std::mutex cloud_mutex_;
  // std::mutex output_mutex_;
  std::mutex data_mutex_;
  std::mutex origin_mutex_;
  std::mutex planner_mutex_;
  std::vector<std::array<double, 3>> dense_path_;
  std::vector<std::array<double, 3>> sparse_path_;

  std::vector<float> height_max_2d_;  // per-column max occupied z; NaN = no data
  std::vector<float> height_min_2d_;  // per-column min occupied z; NaN = no data

  std::thread thread_;
  std::condition_variable condition_;
  std::sig_atomic_t data_ready_;
  double origin_[3];
  double goal_[3];
};
}  // namespace global_mapper
