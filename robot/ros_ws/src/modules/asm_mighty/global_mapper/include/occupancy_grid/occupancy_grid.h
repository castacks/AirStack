// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <array>
#include <vector>
#include <cstring>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

#include "voxel_grid/voxel_grid.h"

namespace occupancy_grid {

using Point = std::array<double, 3>;
using PointList = std::vector<Point>;

class OccupancyGrid : public voxel_grid::VoxelGrid<float> {
 public:
  OccupancyGrid(const double origin[3], const double world_dimensions[3], const double resolution, float threshold);
  void UpdateValue(const int ixyz[3], float value);
  void UpdateValue(const double xyz[3], float value);
  void RayTrace(const int start[3], const int end[3], float increment);
  void RayTrace(const double start[3], const double end[3], float increment);
  bool IsOccupied(const int ixyz[3]) const;
  bool IsOccupied(const float value) const;
  bool IsOccupied(const double xyz[3]) const;
  bool IsUnknown(const int ixyz[3]) const;
  bool IsUnknown(const double xyz[3]) const;
  bool IsUnknown(const float value) const;
  const PointList& GetMarked() const { return marked_list_; }
  const PointList& GetCleared() const { return cleared_list_; }
  void ResetDiffs() { marked_list_.clear(); cleared_list_.clear(); }
  float GetThreshold() const  { return threshold_; }

  bool IsOccupied(const int ind) const; // TODO: Move back to private!

 private:
  virtual void PreShiftOrigin(const std::vector<int>& slice_indexes) override;
  virtual void PostShiftOrigin(const std::vector<int>& slice_indexes) override;
  
  void UpdateValue(const int ind, float delta);
  float clamp_value(float x, float min, float max) const;

  PointList cleared_list_;
  PointList marked_list_;
  float threshold_;
};

} // namespace occupancy_grid
