// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <vector>
#include <string>

namespace global_mapper
{
struct Params
{
  std::string global_frame = "map";
  std::vector<double> origin = { 0, 0, 0 };
  std::vector<double> world_dimensions = { 5.0, 5.0, 1.0 };
  double resolution = 1.0;
  // When true, the grid origin is *locked* to its initial value from this
  // params struct and never slides with the robot. Use this to build a
  // persistent global map for missions where the robot stays inside the
  // configured world_dimensions box. When false (default), the origin
  // re-centers on the robot every odom update — small bounded memory but
  // forgets cells that fall off the window.
  bool lock_origin = false;
  double clear_unknown_radius = -1.0;  // [m] horizontal clearing radius for unknown cells
  double clear_unknown_height = -1.0;  // [m] vertical clearing half-height for unknown cells
  double clear_occupied_radius = -1.0; // [m] horizontal clearing radius for occupied cells; <0 means disabled
  double clear_occupied_height = -1.0; // [m] vertical clearing half-height for occupied cells; <0 means disabled
  double z_ground = 0;
  int skip = 0;
  double depth_max = 10;
  double r1 = 0.8;
  double r2 = 8.0;
  double z_min_unknown = 0.2;
  double z_max_unknown = 5.0;

  // occupancy_grid
  double init_value = 0;
  double hit_inc = 0.2;
  double miss_inc = -0.01;
  double occupancy_threshold = 0.6;

  // temporal_grid
  bool use_temporal_grid = true;
  double temporal_occupied_thresh = 3.0;    // [s] how long a voxel must be occupied to become static
  double temporal_unoccupied_thresh = 0.1;  // [s] how long unoccupied before marked free
  int    dynamic_neighbor_thresh = 5;       // min static neighbors to suppress dynamic label
  double dynamic_persistence = 1.0;         // [s] once dynamic, stay dynamic for this long
  double dynamic_maturity_time = 0.0;       // [s] voxel must exist this long before dynamic labeling (0=disabled)

  // downsampled occupancy grid (for visualization over network)
  double downsample_resolution = 0.0; // [m] leaf size for downsampled grid (0=disabled)

  // distance_grid
  int truncation_distance = 6;  // in voxels

  // distance_grid_2d
  bool use_distance_grid_2d = false;
  int truncation_distance_2d = 10;  // [voxels] can be larger since 2D is cheaper
  double height_diff_threshold = 0.3;  // [m] max height diff between neighbors for traversability
  double agent_height = 1.0;           // [m] robot height — obstacles above this are passable
  double min_obstacle_height = 0.3;    // [m] min vertical extent of occupied voxels to count as obstacle
  int inflation_cells_2d = 0;          // [cells] inflate non-traversable cells by this radius (0=disabled)
  // 2D visibility check height. Used by UpdateOccupancyGrid2D to decide
  // whether a cell is FREE or UNKNOWN. Set this to the lidar / depth-camera
  // mounting height so that "free" means "the sensor's eye-line actually
  // swept this cell". Without it, the column projection over [z_ground,
  // agent_height] marks cells behind walls as free because off-axis beams
  // pass over/under the wall and tag voxels at OTHER z values in the same
  // column. Cells with z_ground < z < agent_height should still be checked
  // for OCCUPIED across the whole column (collision-correct), but FREE
  // requires observation at this specific z slice.
  double visibility_check_z = 0.5;     // [m] (set negative to disable and fall back to column-any-observed)

  // cost_grid
  int inflation_distance = 4;
  int altitude_weight = 20;
  int inflation_weight = 0;
  int unknown_weight = 20;
  int obstacle_weight = 10000;
};
}  // namespace global_mapper
