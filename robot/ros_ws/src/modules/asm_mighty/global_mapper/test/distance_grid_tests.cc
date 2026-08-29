// Copyright 2017 Massachusetts Institute of Technology

#include <chrono>
#include "gtest/gtest.h"
#include "distance_grid/distance_grid.h"

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

TEST(DistanceGrid, AddObstacle1) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;

  distance_grid::DistanceGrid dist_grid(origin, world_dimensions, resolution, 25);

  dist_grid.SetObstacle(1, 1, 0);
  dist_grid.UpdateDistances();

  EXPECT_EQ(2, dist_grid.ReadValue(0, 0, 0));
  EXPECT_EQ(8, dist_grid.ReadValue(-1, -1, 0));
  EXPECT_EQ(1, dist_grid.ReadValue(1, 0, 0));
}

TEST(DistanceGrid, AddObstacle2) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;

  distance_grid::DistanceGrid dist_grid(origin, world_dimensions, resolution, 25);

  dist_grid.SetObstacle(-1, -1, 0);
  dist_grid.SetObstacle(1, 1, 0);

  dist_grid.UpdateDistances();

  EXPECT_EQ(0, dist_grid.ReadValue(-1, -1, 0));
  EXPECT_EQ(0, dist_grid.ReadValue(1, 1, 0));
  EXPECT_EQ(2, dist_grid.ReadValue(0, 0, 0));
  EXPECT_EQ(4, dist_grid.ReadValue(1, -1, 0));
}

TEST(DistanceGrid, Truncation) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;

  distance_grid::DistanceGrid dist_grid(origin, world_dimensions, resolution, 2);

  dist_grid.SetObstacle(-1, -1, 0);
  dist_grid.UpdateDistances();

  EXPECT_EQ(2, dist_grid.ReadValue(0, 0, 0));
  EXPECT_EQ(4, dist_grid.ReadValue(1, 1, 0));
}

TEST(DistanceGrid, RemoveObstacle) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;

  distance_grid::DistanceGrid dist_grid(origin, world_dimensions, resolution, 3);

  dist_grid.SetObstacle(-1, -1, 0);

  dist_grid.UpdateDistances();

  EXPECT_EQ(2, dist_grid.ReadValue(0, 0, 0));

  dist_grid.RemoveObstacle(-1, -1, 0);

  dist_grid.UpdateDistances();

  EXPECT_EQ(9, dist_grid.ReadValue(-1, -1, 0));
  EXPECT_EQ(9, dist_grid.ReadValue(0, 0, 0));

}

TEST(DistanceGrid, RemoveObstacle2) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;

  distance_grid::DistanceGrid dist_grid(origin, world_dimensions, resolution, 3);

  dist_grid.SetObstacle(-1, -1, 0);
  dist_grid.SetObstacle(1, 1, 0);

  dist_grid.UpdateDistances();

  EXPECT_EQ(2, dist_grid.ReadValue(0, 0, 0));
  EXPECT_EQ(0, dist_grid.ReadValue(-1, -1, 0));

  dist_grid.RemoveObstacle(-1, -1, 0);

  dist_grid.UpdateDistances();

  EXPECT_EQ(8, dist_grid.ReadValue(-1, -1, 0));

}

TEST(DistanceGrid, SlidingWindow1) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;

  distance_grid::DistanceGrid dist_grid(origin, world_dimensions, resolution, 3);

  dist_grid.SetObstacle(-1, -1, 0);
  dist_grid.SetObstacle(1, 1, 0);
  dist_grid.UpdateDistances();

  dist_grid.UpdateOrigin(0, 1, 0);
  EXPECT_EQ(5, dist_grid.ReadValue(-1, 0, 0));
  EXPECT_EQ(4, dist_grid.ReadValue(-1, 1, 0));
  EXPECT_EQ(5, dist_grid.ReadValue(-1, 2, 0));
}

TEST(DistanceGrid, SlidingWindow2) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {3, 3, 1};
  double resolution = 1;

  distance_grid::DistanceGrid dist_grid(origin, world_dimensions, resolution, 3);

  dist_grid.SetObstacle(-1, -1, 0);
  dist_grid.SetObstacle(1, 1, 0);
  dist_grid.UpdateDistances();

  dist_grid.UpdateOrigin(1, 1, 0);
  EXPECT_EQ(2, dist_grid.ReadValue(0, 0, 0));
  EXPECT_EQ(1, dist_grid.ReadValue(0, 1, 0));
  EXPECT_EQ(2, dist_grid.ReadValue(0, 2, 0));
}

// TEST(DistanceGrid, SlidingWindow3) {
//   double origin[3] = {0, 0, 0};
//   double world_dimensions[3] = {7, 7, 1};
//   double resolution = 1;

//   distance_grid::DistanceGrid dist_grid(origin, world_dimensions, resolution, 3);

//   dist_grid.SetObstacle(2.4, -3.5, 0);
//   dist_grid.SetObstacle(2.4, 0, 0);
//   dist_grid.SetObstacle(2.4, 2.5, 0);
//   dist_grid.SetObstacle(-2.4, -3.5, 0);
//   dist_grid.SetObstacle(-2.4, 0, 0);
//   dist_grid.SetObstacle(-2.4, 2.5, 0);
  
//   dist_grid.UpdateDistances();
//   dist_grid.PrintGrid();

//   int shift = 1;
//   printf("Shift: %i\n", shift);
//   dist_grid.UpdateOrigin(shift, 0, 0);
//   dist_grid.PrintGrid();

//   shift = 2;
//   printf("Shift: %i\n", shift);
//   dist_grid.UpdateOrigin(shift, 0, 0);
//   dist_grid.PrintGrid();

//   shift = 3;
//   printf("Shift: %i\n", shift);
//   dist_grid.UpdateOrigin(shift, 0, 0);
//   dist_grid.PrintGrid();

//   shift = 4;
//   printf("Shift: %i\n", shift);
//   dist_grid.UpdateOrigin(shift, 0, 0);
//   dist_grid.PrintGrid();

//   shift = 5;
//   printf("Shift: %i\n", shift);
//   dist_grid.UpdateOrigin(shift, 0, 0);
//   dist_grid.PrintGrid();

//   shift = 4;
//   printf("Shift: %i\n", shift);
//   dist_grid.UpdateOrigin(shift, 0, 0);
//   dist_grid.PrintGrid();

//   shift = 3;
//   printf("Shift: %i\n", shift);
//   dist_grid.UpdateOrigin(shift, 0, 0);
//   dist_grid.PrintGrid();

//   shift = 2;
//   printf("Shift: %i\n", shift);
//   dist_grid.UpdateOrigin(shift, 0, 0);
//   dist_grid.PrintGrid();

//   shift = 1;
//   printf("Shift: %i\n", shift);
//   dist_grid.UpdateOrigin(shift, 0, 0);
//   dist_grid.PrintGrid();

//   shift = 0;
//   printf("Shift: %i\n", shift);
//   dist_grid.UpdateOrigin(shift, 0, 0);
//   dist_grid.PrintGrid();

//   shift = -1;
//   printf("Shift: %i\n", shift);
//   dist_grid.UpdateOrigin(shift, 0, 0);
//   dist_grid.PrintGrid();

//   shift = -2;
//   printf("Shift: %i\n", shift);
//   dist_grid.UpdateOrigin(shift, 0, 0);
//   dist_grid.PrintGrid();
// }

TEST(DistanceGrid, SlidingWindow4) {
  double origin[3] = {0, 0, 0};
  double world_dimensions[3] = {7, 7, 1};
  double resolution = 1;

  distance_grid::DistanceGrid dist_grid(origin, world_dimensions, resolution, 3);

  dist_grid.SetObstacle(2.4, -3.5, 0);
  dist_grid.SetObstacle(2.4, 0, 0);
  dist_grid.SetObstacle(2.4, 2.5, 0);
  dist_grid.SetObstacle(-2.4, -3.5, 0);
  dist_grid.SetObstacle(-2.4, 0, 0);
  dist_grid.SetObstacle(-2.4, 2.5, 0);
  dist_grid.UpdateDistances();

  for (float shift = 1.0; shift < 10*resolution; shift += 0.01) {
    auto slice_inds = dist_grid.UpdateOrigin(shift, 0, 0);
    EXPECT_GE(slice_inds.size(), 0);
    if (slice_inds.size() > 0) {
      EXPECT_EQ(slice_inds.size(), 7);
    }
  }

  for (float shift = -1.0; shift > 10*resolution; shift -= 0.01) {
    auto slice_inds = dist_grid.UpdateOrigin(shift, 0, 0);
    EXPECT_GE(slice_inds.size(), 0);
    if (slice_inds.size() > 0) {
      EXPECT_EQ(slice_inds.size(), 7);
    }
  }

  for (float shift = 1.0; shift > 10*resolution; shift += 0.01) {
    auto slice_inds = dist_grid.UpdateOrigin(0, shift, 0);
    EXPECT_GE(slice_inds.size(), 0);
    if (slice_inds.size() > 0) {
      EXPECT_EQ(slice_inds.size(), 7);
    }
  }

  for (float shift = -1.0; shift > 10*resolution; shift -= 0.01) {
    auto slice_inds = dist_grid.UpdateOrigin(0, shift, 0);
    EXPECT_GE(slice_inds.size(), 0);
    if (slice_inds.size() > 0) {
      EXPECT_EQ(slice_inds.size(), 7);
    }
  }
}
