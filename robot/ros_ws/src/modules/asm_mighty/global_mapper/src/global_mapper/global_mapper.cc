// Copyright 2017 Massachusetts Institute of Technology

#include <cmath>
#include <limits>

#include "occupancy_grid/occupancy_grid.h"
#include "global_mapper/global_mapper.h"

namespace global_mapper
{
  GlobalMapper::GlobalMapper(Params &params)
      : params_(params), occupancy_grid_(params_.origin.data(), params_.world_dimensions.data(), params_.resolution,
                                         params_.occupancy_threshold),
        distance_grid_(params_.origin.data(), params_.world_dimensions.data(), params_.resolution,
                       params_.truncation_distance),
        cost_grid_(params_.origin.data(), params_.world_dimensions.data(), params_.resolution),
        temporal_grid_(params_.origin.data(), params_.world_dimensions.data(), params_.resolution,
                       params_.temporal_occupied_thresh, params_.temporal_unoccupied_thresh,
                       params_.dynamic_neighbor_thresh, params_.dynamic_persistence,
                       params_.dynamic_maturity_time),
        data_ready_(0)
  {
    origin_[0] = 0.0;
    origin_[1] = 0.0;
    origin_[2] = 0.0;

    // Initialize 2D grids if enabled
    if (params_.use_distance_grid_2d)
    {
      double origin_2d[2] = {params_.origin[0], params_.origin[1]};
      double dims_2d[2] = {params_.world_dimensions[0], params_.world_dimensions[1]};
      occupancy_grid_2d_ = occupancy_grid_2d::OccupancyGrid2D(origin_2d, dims_2d, params_.resolution);
      distance_grid_2d_ = distance_grid_2d::DistanceGrid2D(origin_2d, dims_2d, params_.resolution,
                                                            params_.truncation_distance_2d);
      int gx = static_cast<int>(params_.world_dimensions[0] / params_.resolution);
      int gy = static_cast<int>(params_.world_dimensions[1] / params_.resolution);
      height_max_2d_.assign(gx * gy, std::numeric_limits<float>::quiet_NaN());
      height_min_2d_.assign(gx * gy, std::numeric_limits<float>::quiet_NaN());
    }
    goal_[0] = 0.0;
    goal_[1] = 0.0;
    goal_[2] = 0.0;
    cost_grid_.SetDistanceGrid(&distance_grid_);
    cost_grid_.SetOccupancyGrid(&occupancy_grid_);
    cost_grid_.SetInflationDistance(params.inflation_distance);
    SetCostWeights(params_.altitude_weight, params_.inflation_weight, params_.unknown_weight, params_.obstacle_weight);
    occupancy_grid_.Reset(params.init_value);
  }

  GlobalMapper::~GlobalMapper()
  {
    if (thread_.joinable())
    {
      thread_.join();
    }
  }

  void GlobalMapper::SetCostWeights(int altitude_weight, int inflation_weight, int unknown_weight, int obstacle_weight)
  {
    cost_grid_.SetCostWeights(altitude_weight, inflation_weight, unknown_weight, obstacle_weight);
  }

  void GlobalMapper::SetObstacle(const double xyz[3])
  {
    occupancy_grid_.UpdateValue(xyz, 1.0);
  }

  void GlobalMapper::RemoveObstacle(const double xyz[3])
  {
    occupancy_grid_.UpdateValue(xyz, -1.0);
  }

  void GlobalMapper::UpdateGrids()
  {
    UpdateDistanceGrid();
    UpdateCostGrid();
    occupancy_grid_.ResetDiffs();
  }

  void GlobalMapper::UpdateOrigin(const double xyz[3])
  {
    std::lock_guard<std::mutex> origin_lock(origin_mutex_);
    memcpy(origin_, xyz, sizeof(double) * 3);
  }

  void GlobalMapper::PushPointCloud(const PointCloud::ConstPtr &cloud_ptr, double timestamp)
  {
    // push
    std::lock_guard<std::mutex> cloud_lock(cloud_mutex_);

    point_cloud_buffer_.clear(); // Jesus added this (remove all the previous point clouds--> queue will be only 1)
    timestamp_buffer_.clear();

    point_cloud_buffer_.push_back(cloud_ptr);
    timestamp_buffer_.push_back(timestamp);

    // notify
    std::unique_lock<std::mutex> data_lock(data_mutex_);
    data_ready_ = true;
    data_lock.unlock();
    condition_.notify_one();
  }

  const PointCloud::ConstPtr GlobalMapper::PopPointCloud()
  {
    // pop
    std::lock_guard<std::mutex> cloud_lock(cloud_mutex_);
    PointCloud::ConstPtr cloud_ptr = nullptr;
    if (!point_cloud_buffer_.empty())
    {
      cloud_ptr = point_cloud_buffer_.front();
      point_cloud_buffer_.pop_front();
    }

    if (!timestamp_buffer_.empty())
    {
      timestamp_ = timestamp_buffer_.front();
      timestamp_buffer_.pop_front(); 
    }
    
    
    // notify
    std::unique_lock<std::mutex> data_lock(data_mutex_);
    if (point_cloud_buffer_.size() == 0)
    {
      data_ready_ = false;
    }

    return cloud_ptr;
  }

  void GlobalMapper::InsertPointCloud(const PointCloud::ConstPtr &cloud_ptr)
  {
    if (!cloud_ptr)
      return;

    // Sensor origin (world frame)
    double start[3] = {
        cloud_ptr->sensor_origin_[0],
        cloud_ptr->sensor_origin_[1],
        cloud_ptr->sensor_origin_[2]};
    double end[3];

    // Maximum sensor range and squared threshold
    const double max_r = params_.depth_max;
    const double max_r_sq = max_r * max_r;

    // --- 1) MISS pass (ray‐trace): clear cells along every beam ---
    for (size_t i = 0; i < cloud_ptr->points.size(); ++i)
    {
      const auto &pt = cloud_ptr->points[i];
      end[0] = pt.x;
      end[1] = pt.y;
      end[2] = pt.z;

      // Use miss_inc (typically negative) to decrement occupancy along the ray
      occupancy_grid_.RayTrace(start, end, params_.miss_inc);
    }

    // --- 2) HIT pass (endpoints): only mark true returns as obstacles ---
    for (size_t i = 0; i < cloud_ptr->points.size(); ++i)
    {
      const auto &pt = cloud_ptr->points[i];

      // Compute squared distance from sensor origin
      double dx = pt.x - start[0];
      double dy = pt.y - start[1];
      double dz = pt.z - start[2];
      double dist_sq = dx * dx + dy * dy + dz * dz;

      if (dist_sq <= max_r_sq && std::isfinite(pt.intensity))
      {
        // It’s a valid return within range ⇒ mark occupied
        end[0] = pt.x;
        end[1] = pt.y;
        end[2] = pt.z;
        occupancy_grid_.UpdateValue(end, params_.hit_inc);
      }
      // Else: beam simply clipped at max range ⇒ do not insert a false obstacle
    }
  }

  void GlobalMapper::GetVoxelGrids(voxel_grid::VoxelGrid<float> *occupancy_grid,
                                   voxel_grid::VoxelGrid<int> *distance_grid, 
                                   voxel_grid::VoxelGrid<int> *cost_grid,
                                   voxel_grid::VoxelGrid<std::array<double, 7>> *temporal_grid)
  {
    std::lock_guard<std::mutex> output_lock(output_mutex_);
    *occupancy_grid = occupancy_grid_;
    *distance_grid = distance_grid_;
    *cost_grid = cost_grid_;
    *temporal_grid = temporal_grid_;
  }

  void GlobalMapper::UpdateOccupancyGrid()
  {
    PointCloud::ConstPtr cloud_ptr = PopPointCloud();
    InsertPointCloud(cloud_ptr);

    int origin_ixyz[3];
    occupancy_grid_.WorldToGrid(origin_, origin_ixyz);

    // Clear unknown voxels around vehicle in a cylinder
    {
      const double hr = (params_.clear_unknown_radius >= 0.0) ? params_.clear_unknown_radius : 0.5; // default to 0.5m if not set
      const double hz = (params_.clear_unknown_height >= 0.0) ? params_.clear_unknown_height : 0.5; // default to 0.5m if not set
      const int nr = static_cast<int>(hr / params_.resolution + 0.999);
      const int nz = static_cast<int>(hz / params_.resolution + 0.999);
      const int nr_safe = (nr > 1) ? nr : 1;
      const int nz_safe = (nz > 1) ? nz : 1;
      const double nr2 = static_cast<double>(nr_safe) * static_cast<double>(nr_safe);

      for (int i = -nr_safe; i <= nr_safe; i++)
      {
        for (int j = -nr_safe; j <= nr_safe; j++)
        {
          if (static_cast<double>(i * i + j * j) > nr2)
            continue;
          for (int k = -nz_safe; k <= nz_safe; k++)
          {
            int ixyz[3];
            ixyz[0] = origin_ixyz[0] + i;
            ixyz[1] = origin_ixyz[1] + j;
            ixyz[2] = origin_ixyz[2] + k;
            // OOB check is mandatory: VoxelGrid uses modulo addressing, so an
            // OOB index would wrap around and corrupt cells on the opposite
            // side of the grid. Harmless when sliding (robot is always near
            // grid center) but unsafe with lock_origin=true.
            if (!occupancy_grid_.IsInMap(ixyz)) continue;
            if (occupancy_grid_.IsUnknown(ixyz))
            {
              occupancy_grid_.UpdateValue(ixyz, -1.0);
            }
          }
        }
      }
    }

    // Clear occupied voxels around vehicle in a (typically smaller) cylinder
    if (params_.clear_occupied_radius >= 0.0 && params_.clear_occupied_height >= 0.0)
    {
      const double hr = params_.clear_occupied_radius;
      const double hz = params_.clear_occupied_height;
      const int nr = static_cast<int>(hr / params_.resolution + 0.999);
      const int nz = static_cast<int>(hz / params_.resolution + 0.999);
      const int nr_safe = (nr > 1) ? nr : 1;
      const int nz_safe = (nz > 1) ? nz : 1;
      const double nr2 = static_cast<double>(nr_safe) * static_cast<double>(nr_safe);

      for (int i = -nr_safe; i <= nr_safe; i++)
      {
        for (int j = -nr_safe; j <= nr_safe; j++)
        {
          if (static_cast<double>(i * i + j * j) > nr2)
            continue;
          for (int k = -nz_safe; k <= nz_safe; k++)
          {
            int ixyz[3];
            ixyz[0] = origin_ixyz[0] + i;
            ixyz[1] = origin_ixyz[1] + j;
            ixyz[2] = origin_ixyz[2] + k;
            if (!occupancy_grid_.IsInMap(ixyz)) continue;
            if (occupancy_grid_.IsOccupied(ixyz))
            {
              occupancy_grid_.UpdateValue(ixyz, -1.0);
            }
          }
        }
      }
    }
  }

  void GlobalMapper::UpdateCostGrid()
  {
    cost_grid_.UpdateDijkstra(goal_);
    dense_path_.clear();
    sparse_path_.clear();
    cost_grid_.GetPaths(origin_, &dense_path_, &sparse_path_);
  }

  void GlobalMapper::GetPaths(std::vector<std::array<double, 3>> *dense_path,
                              std::vector<std::array<double, 3>> *sparse_path)
  {
    std::lock_guard<std::mutex> output_lock(output_mutex_);
    *dense_path = dense_path_;
    *sparse_path = sparse_path_;
  }

  void GlobalMapper::UpdateDistanceGrid()
  {
    for (auto const &cleared_point : occupancy_grid_.GetCleared())
    {
      distance_grid_.RemoveObstacle(cleared_point.data());
    }
    distance_grid_.UpdateDistances();

    for (auto const &marked_point : occupancy_grid_.GetMarked())
    {
      distance_grid_.SetObstacle(marked_point.data());
    }
    distance_grid_.UpdateDistances();
  }

  void GlobalMapper::UpdateTemporalGrid(double timestamp)
  {
    int grid_dimensions[3];
    temporal_grid_.GetGridDimensions(grid_dimensions);

    for (int x = 0; x < grid_dimensions[0]; x++)
    {
      for (int y = 0; y < grid_dimensions[1]; y++)
      {
        for (int z = 0; z < grid_dimensions[2]; z++)
        {
          int ixyz[3] = {x, y, z};
          float occupancy_value = occupancy_grid_.ReadValue(ixyz);
          bool is_occupied = occupancy_grid_.IsOccupied(occupancy_value);
          bool is_unknown = occupancy_grid_.IsUnknown(occupancy_value);
          temporal_grid_.UpdateTemporalInfo(ixyz, is_occupied, is_unknown, timestamp);
        }
      }
    }
  }

  void GlobalMapper::UpdateOccupancyGrid2D()
  {
    int grid_dims[3];
    occupancy_grid_.GetGridDimensions(grid_dims);

    occupancy_grid_2d_.ResetDiffs();

    // Tristate projection over [z_ground, agent_height]:
    //   1) any occupied voxel anywhere in the column  -> OCCUPIED
    //      (collision-correct: an obstacle anywhere along the column blocks the cell)
    //   2) the voxel at z = visibility_check_z is observed-free -> FREE
    //      (visibility-correct: "free" requires the sensor's eye-line slice to
    //       have actually swept this cell. Off-axis beams pass over/under walls
    //       and tag voxels at OTHER z values free along the same column, which
    //       would otherwise label cells behind walls as free. The single-slice
    //       check at the sensor mounting height prevents this.)
    //   3) otherwise                                  -> UNKNOWN
    // If visibility_check_z < 0 we fall back to the legacy "any observed-free
    // in the column" rule (useful for cameras with narrow vertical FOV where
    // there is no single canonical eye-line height).
    const bool use_slice_visibility = params_.visibility_check_z >= 0.0;
    for (int x = 0; x < grid_dims[0]; x++)
    {
      for (int y = 0; y < grid_dims[1]; y++)
      {
        bool any_occupied = false;
        bool any_observed_free = false;          // legacy fallback
        bool slice_observed_free = false;        // for use_slice_visibility
        for (int z = 0; z < grid_dims[2]; z++)
        {
          int ixyz[3] = {x, y, z};
          double xyz[3];
          occupancy_grid_.GridToWorld(ixyz, xyz);
          if (xyz[2] <= params_.z_ground || xyz[2] >= params_.agent_height)
          {
            continue;
          }
          if (occupancy_grid_.IsOccupied(ixyz))
          {
            any_occupied = true;
            // Don't break — we still need to know if the visibility slice
            // is observed-free if the column happens to also have an
            // occupied voxel above/below (rare but possible).
            // Actually for OCCUPIED-dominates we can break early for perf.
            break;
          }
          if (!occupancy_grid_.IsUnknown(ixyz))
          {
            any_observed_free = true;
            if (use_slice_visibility)
            {
              // Test whether this voxel covers the visibility slice height.
              const double half_res = 0.5 * params_.resolution;
              if (xyz[2] - half_res <= params_.visibility_check_z &&
                  xyz[2] + half_res >  params_.visibility_check_z)
              {
                slice_observed_free = true;
              }
            }
          }
        }

        int ixyz[3] = {x, y, 0};
        double xyz[3];
        occupancy_grid_.GridToWorld(ixyz, xyz);

        if (any_occupied)
        {
          occupancy_grid_2d_.SetOccupied(xyz[0], xyz[1]);
        }
        else if (use_slice_visibility ? slice_observed_free : any_observed_free)
        {
          occupancy_grid_2d_.SetFree(xyz[0], xyz[1]);
        }
        else
        {
          occupancy_grid_2d_.SetUnknown(xyz[0], xyz[1]);
        }
      }
    }
  }

  void GlobalMapper::UpdateDistanceGrid2D()
  {
    for (const auto& cleared : occupancy_grid_2d_.GetCleared())
    {
      distance_grid_2d_.RemoveObstacle(cleared[0], cleared[1]);
    }
    distance_grid_2d_.UpdateDistances();

    for (const auto& marked : occupancy_grid_2d_.GetMarked())
    {
      distance_grid_2d_.SetObstacle(marked[0], marked[1]);
    }
    distance_grid_2d_.UpdateDistances();
  }

  void GlobalMapper::GetOrigin(double xyz[3]) const
  {
    memcpy(xyz, origin_, sizeof(double) * 3);
  }

  bool GlobalMapper::GetProjectedGoal(double xyz[3]) const
  {
    return cost_grid_.GetProjectedGoal(xyz);
  }

  void GlobalMapper::SetGoal(const double xyz[3])
  {
    memcpy(goal_, xyz, sizeof(double) * 3);
  }

  void GlobalMapper::GetGoal(double xyz[3]) const
  {
    memcpy(xyz, goal_, sizeof(double) * 3);
  }

  void GlobalMapper::Spin()
  {
    static int spincount = 0;
    while (true) // TODO switch back to loop 
    {
      std::unique_lock<std::mutex> data_lock(data_mutex_);
      condition_.wait(data_lock, [this]
                       { return data_ready_; });
      data_lock.unlock();

      std::unique_lock<std::mutex> output_lock(output_mutex_);
 
      origin_mutex_.lock();
      if (!params_.lock_origin)
      {
        occupancy_grid_.UpdateOrigin(origin_);
        // distance_grid_.UpdateOrigin(origin_);
        // cost_grid_.UpdateOrigin(origin_);
      }
      origin_mutex_.unlock();

      occupancy_grid_.ResetDiffs();
      UpdateOccupancyGrid();
      if (params_.use_temporal_grid)
      {
        if (!params_.lock_origin)
        {
          temporal_grid_.UpdateOrigin(origin_);
        }
        UpdateTemporalGrid(timestamp_);
      }
      if (params_.use_distance_grid_2d)
      {
        if (!params_.lock_origin)
        {
          occupancy_grid_2d_.UpdateOrigin(origin_[0], origin_[1]);
          distance_grid_2d_.UpdateOrigin(origin_[0], origin_[1]);
        }
        UpdateOccupancyGrid2D();
        UpdateDistanceGrid2D();
      }
      // UpdateDistanceGrid();
      // if ((spincount++ % 15) == 0)
      //{
      //  UpdateCostGrid();
      //}
      output_lock.unlock();
    }
  }

  void GlobalMapper::Run()
  {
    // fprintf(stderr, "GlobalMapper::Run\n");
    thread_ = std::thread(&GlobalMapper::Spin, this);
    // this->Spin();
  }

} // namespace global_mapper
