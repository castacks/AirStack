// Copyright 2017 Massachusetts Institute of Technology

#include <chrono>
#include "gtest/gtest.h"
#include "cost_grid/cost_grid.h"

class Stopwatch {
public:

  Stopwatch() 
  : running(false),
    start_time(std::chrono::high_resolution_clock::now()),
    end_time(start_time) { }

  void Start() {
    running = true;
    start_time = std::chrono::high_resolution_clock::now();
  }

  void Stop() {
    running = false;
    end_time = std::chrono::high_resolution_clock::now();
  }

  double ElapsedMillis() const {
    using namespace std::chrono;
    duration<double> time_span;
    if(running) {
      time_span = duration_cast<duration<double>>(high_resolution_clock::now() - start_time);
    } else {
      time_span = duration_cast<duration<double>>(end_time - start_time);
    }
    return time_span.count()*1000;
  }

private:
  bool running;
  std::chrono::high_resolution_clock::time_point start_time;
  std::chrono::high_resolution_clock::time_point end_time;
};

TEST(CostGrid, CompleteExplorationGoalInMap) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {60.0, 60.0, 10.0};
  double resolution = 0.2;

  cost_grid::CostGrid cost_grid(origin, world_dimensions, resolution);

  double goal[3] = {20.0, 0.0, 0.0};
  cost_grid.UpdateDijkstra(goal);
  int count = 0;
  
  int grid_dimensions[3];
  cost_grid.GetGridDimensions(grid_dimensions);

  double xyz[3] = {0.0};
  for (int x = 0; x < grid_dimensions[0]; x++) {
    for (int y = 0; y < grid_dimensions[1]; y++) {
      for (int z = 0; z < grid_dimensions[2]; z++) {
        int ixyz[3] = {x, y, z};
        cost_grid.GridToWorld(ixyz, xyz);
        int cost = cost_grid.ReadValue(xyz);

        if (cost == cost_grid::MAX_COST) {
          count++;
        }
      }
    }
  }

  EXPECT_EQ(count, 0);
}

TEST(CostGrid, CompleteExplorationGoalNotInMap) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {60.0, 60.0, 10.0};
  double resolution = 0.2;

  cost_grid::CostGrid cost_grid(origin, world_dimensions, resolution);

  double goal[3] = {100.0, 0.0, 0.0};
  cost_grid.UpdateDijkstra(goal);

  int count = 0;
  int grid_dimensions[3];
  cost_grid.GetGridDimensions(grid_dimensions);

  double xyz[3] = {0.0};
  for (int x = 0; x < grid_dimensions[0]; x++) {
    for (int y = 0; y < grid_dimensions[1]; y++) {
      for (int z = 0; z < grid_dimensions[2]; z++) {
        int ixyz[3] = {x, y, z};
        cost_grid.GridToWorld(ixyz, xyz);
        int cost = cost_grid.ReadValue(xyz);

        if (cost == cost_grid::MAX_COST) {
          count++;
        }
      }
    }
  }

  EXPECT_EQ(count, 0);
}

TEST(CostGrid, Timing1) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {60.0, 60.0, 10.0};
  double resolution = 0.5;

  cost_grid::CostGrid cost_grid(origin, world_dimensions, resolution);
  double goal[3] = {100.0, -0.5, -0.5};

  Stopwatch sw;
  sw.Start();
  cost_grid.UpdateDijkstra(goal);
  sw.Stop();
  std::cout << "Took " << sw.ElapsedMillis() <<  " ms" << std::endl;

  EXPECT_GT(sw.ElapsedMillis(), 75);
  EXPECT_LT(sw.ElapsedMillis(), 125);
}

TEST(CostGrid, UpdateOrigin1) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {5.0, 5.0, 1.0};
  double resolution = 1.0;

  cost_grid::CostGrid cost_grid(origin, world_dimensions, resolution);

  double goal[3] = {3 * resolution, 0, 0};
  cost_grid.UpdateDijkstra(goal);
  int ixyz[3] = {0, 0, 0};
  double xyz[3];
  cost_grid.GridToWorld(ixyz, xyz);
  EXPECT_EQ(cost_grid.ReadValue(xyz), 6);
  
  origin[0] = 2 * resolution;
  auto slice_inds = cost_grid.UpdateOrigin(origin);
  cost_grid.UpdateDijkstra(goal);
  
  cost_grid.GridToWorld(ixyz, xyz);
  EXPECT_EQ(cost_grid.ReadValue(xyz), 5);
  EXPECT_EQ(slice_inds.size(), 10);
  
  cost_grid.UpdateOrigin(goal);
  cost_grid.UpdateDijkstra(goal);
  ixyz[0] = 2;
  ixyz[1] = 2;
  cost_grid.GridToWorld(ixyz, xyz);
  EXPECT_EQ(cost_grid.ReadValue(xyz), 0);
}

TEST(CostGrid, GetLineCost) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {5.0, 5.0, 1.0};
  double resolution = 1.0;

  cost_grid::CostGrid cost_grid(origin, world_dimensions, resolution);

  double goal[3] = {3 * resolution, 0, 0};
  cost_grid.UpdateDijkstra(goal);
  cost_grid.PrintGraph();

  double start[3] = {-2, 0, 0};
  double end[3] = {2, 0, 0};
  int line_cost = cost_grid.GetLineCost(start, end);

  EXPECT_EQ(line_cost, cost_grid.GetLineCost(end, start));
  EXPECT_EQ(line_cost, 10);

}
