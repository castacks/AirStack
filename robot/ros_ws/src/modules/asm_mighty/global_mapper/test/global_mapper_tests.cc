// Copyright 2017 Massachusetts Institute of Technology

#include <chrono>
#include "gtest/gtest.h"
#include "global_mapper/global_mapper.h"

TEST(GlobalMapper, SetObstacles) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {5.0, 5.0, 1.0};
  double resolution = 1.0;

  global_mapper::Params params;
  params.inflation_distance = 0;
  params.altitude_weight = 0;
  params.inflation_weight = 0;
  params.unknown_weight = 0;
  params.obstacle_weight = 10;

  global_mapper::GlobalMapper mapper(params);
  
  double obstacle[3] = {0, 1, 0};
  double goal[3] = {0, 2, 0};
  
  mapper.UpdateOrigin(origin);
  mapper.SetGoal(goal);
  mapper.SetObstacle(obstacle);
  mapper.UpdateGrids();

  voxel_grid::VoxelGrid<float> occupancy_grid;
  voxel_grid::VoxelGrid<int> distance_grid;
  voxel_grid::VoxelGrid<int> cost_grid;
  voxel_grid::VoxelGrid<std::vector<double>> temporal_grid;
  
  mapper.GetVoxelGrids(&occupancy_grid, &distance_grid, &cost_grid, &temporal_grid);
  
  std::cout << "Occupancy Grid: " << std::endl;
  occupancy_grid.PrintValues();

  std::cout << "Distance Grid: " << std::endl;
  distance_grid.PrintValues();

  std::cout << "Cost Grid: " << std::endl;
  cost_grid.PrintValues();

  EXPECT_EQ(cost_grid.ReadValue(origin), 4);
}
