// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace occupancy_grid_2d {

using Point2D = std::array<double, 2>;
using PointList2D = std::vector<Point2D>;

// Tristate cell encoding — chosen to match nav_msgs/OccupancyGrid codes so the
// publisher path becomes a copy. Frontier detection downstream needs the
// distinction between UNKNOWN and FREE.
static constexpr int8_t kUnknown  = -1;
static constexpr int8_t kFree     = 0;
static constexpr int8_t kOccupied = 100;

class OccupancyGrid2D {
 public:
  OccupancyGrid2D();
  OccupancyGrid2D(const double origin[2], const double world_dimensions[2],
                   double resolution);

  void SetOccupied(double x, double y);
  void SetFree(double x, double y);
  void SetUnknown(double x, double y);
  bool IsOccupied(int ix, int iy) const;
  bool IsFree(int ix, int iy) const;
  bool IsUnknown(int ix, int iy) const;
  int8_t GetCell(int ix, int iy) const;
  void UpdateOrigin(double x, double y);
  void ResetDiffs();
  const PointList2D& GetMarked() const { return marked_; }
  const PointList2D& GetCleared() const { return cleared_; }

  void GetGridDimensions(int dims[2]) const;
  double GetResolution() const { return resolution_; }
  void GetOrigin(double xy[2]) const;
  bool IsInMap(int ix, int iy) const;
  void WorldToGrid(double x, double y, int& ix, int& iy) const;
  void GridToWorld(int ix, int iy, double& wx, double& wy) const;

 private:
  int GridToIndex(int ix, int iy) const;
  void WorldToVoxels(double x, double y, int& vx, int& vy) const;
  void ComputeLowerLeft(double cx, double cy, int new_ll[2]) const;
  void GetSliceIndices(const int new_ll[2], std::vector<int>* indices) const;
  void ShiftOrigin(const int new_ll[2]);

  int grid_dims_[2];
  int lower_left_voxels_[2];
  double resolution_;
  int num_cells_;
  std::vector<int8_t> data_;
  PointList2D marked_;
  PointList2D cleared_;
};

}  // namespace occupancy_grid_2d
