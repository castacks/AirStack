#include "occupancy_grid_2d/occupancy_grid_2d.h"

#include <algorithm>
#include <cmath>

namespace occupancy_grid_2d {

OccupancyGrid2D::OccupancyGrid2D()
    : resolution_(0.0), num_cells_(0) {
  grid_dims_[0] = grid_dims_[1] = 0;
  lower_left_voxels_[0] = lower_left_voxels_[1] = 0;
}

OccupancyGrid2D::OccupancyGrid2D(const double origin[2],
                                   const double world_dimensions[2],
                                   double resolution)
    : resolution_(resolution) {
  for (int i = 0; i < 2; i++) {
    grid_dims_[i] = static_cast<int>(world_dimensions[i] / resolution_);
  }
  num_cells_ = grid_dims_[0] * grid_dims_[1];
  data_.assign(num_cells_, kUnknown);

  // Initialize origin
  int new_ll[2];
  ComputeLowerLeft(origin[0], origin[1], new_ll);
  ShiftOrigin(new_ll);
}

void OccupancyGrid2D::SetOccupied(double x, double y) {
  int ix, iy;
  WorldToGrid(x, y, ix, iy);
  if (!IsInMap(ix, iy)) return;

  int idx = GridToIndex(ix, iy);
  if (data_[idx] != kOccupied) {
    data_[idx] = kOccupied;
    marked_.push_back({x, y});
  }
}

void OccupancyGrid2D::SetFree(double x, double y) {
  int ix, iy;
  WorldToGrid(x, y, ix, iy);
  if (!IsInMap(ix, iy)) return;

  int idx = GridToIndex(ix, iy);
  if (data_[idx] == kFree) return;

  // Only push to cleared_ when transitioning OUT of OCCUPIED, since
  // DistanceGrid2D only cares about obstacle add/remove. UNKNOWN→FREE is
  // invisible to the ESDF pipeline.
  if (data_[idx] == kOccupied) {
    cleared_.push_back({x, y});
  }
  data_[idx] = kFree;
}

void OccupancyGrid2D::SetUnknown(double x, double y) {
  int ix, iy;
  WorldToGrid(x, y, ix, iy);
  if (!IsInMap(ix, iy)) return;

  int idx = GridToIndex(ix, iy);
  if (data_[idx] == kUnknown) return;

  // OCCUPIED→UNKNOWN is an obstacle removal for the ESDF; FREE→UNKNOWN is not.
  if (data_[idx] == kOccupied) {
    cleared_.push_back({x, y});
  }
  data_[idx] = kUnknown;
}

bool OccupancyGrid2D::IsOccupied(int ix, int iy) const {
  if (!IsInMap(ix, iy)) return false;
  return data_[GridToIndex(ix, iy)] == kOccupied;
}

bool OccupancyGrid2D::IsFree(int ix, int iy) const {
  if (!IsInMap(ix, iy)) return false;
  return data_[GridToIndex(ix, iy)] == kFree;
}

bool OccupancyGrid2D::IsUnknown(int ix, int iy) const {
  // Cells outside the map are conceptually unknown.
  if (!IsInMap(ix, iy)) return true;
  return data_[GridToIndex(ix, iy)] == kUnknown;
}

int8_t OccupancyGrid2D::GetCell(int ix, int iy) const {
  if (!IsInMap(ix, iy)) return kUnknown;
  return data_[GridToIndex(ix, iy)];
}

void OccupancyGrid2D::UpdateOrigin(double x, double y) {
  int new_ll[2];
  ComputeLowerLeft(x, y, new_ll);

  int shift[2];
  for (int i = 0; i < 2; i++) {
    shift[i] = new_ll[i] - lower_left_voxels_[i];
  }

  if (shift[0] == 0 && shift[1] == 0) return;

  // Reset cells in shifted slices to UNKNOWN. If the wiped cell was OCCUPIED,
  // also push to cleared_ so the ESDF can drop the obstacle.
  std::vector<int> slice_indices;
  GetSliceIndices(new_ll, &slice_indices);
  for (int idx : slice_indices) {
    if (data_[idx] == kOccupied) {
      // Reverse-lookup world coords from the old origin (we're about to shift,
      // so use current lower_left).
      int raw_iy = idx / grid_dims_[0];
      int raw_ix = idx - raw_iy * grid_dims_[0];
      int gx = (raw_ix - lower_left_voxels_[0]) % grid_dims_[0];
      if (gx < 0) gx += grid_dims_[0];
      int gy = (raw_iy - lower_left_voxels_[1]) % grid_dims_[1];
      if (gy < 0) gy += grid_dims_[1];
      double wx, wy;
      GridToWorld(gx, gy, wx, wy);
      cleared_.push_back({wx, wy});
    }
    data_[idx] = kUnknown;
  }

  ShiftOrigin(new_ll);
}

void OccupancyGrid2D::ResetDiffs() {
  marked_.clear();
  cleared_.clear();
}

void OccupancyGrid2D::GetGridDimensions(int dims[2]) const {
  dims[0] = grid_dims_[0];
  dims[1] = grid_dims_[1];
}

void OccupancyGrid2D::GetOrigin(double xy[2]) const {
  for (int i = 0; i < 2; i++) {
    xy[i] = (lower_left_voxels_[i] + (grid_dims_[i] >> 1)) * resolution_;
  }
}

bool OccupancyGrid2D::IsInMap(int ix, int iy) const {
  return ix >= 0 && ix < grid_dims_[0] && iy >= 0 && iy < grid_dims_[1];
}

void OccupancyGrid2D::WorldToGrid(double x, double y, int& ix, int& iy) const {
  int vx, vy;
  WorldToVoxels(x, y, vx, vy);
  ix = vx - lower_left_voxels_[0];
  iy = vy - lower_left_voxels_[1];
}

void OccupancyGrid2D::GridToWorld(int ix, int iy, double& wx, double& wy) const {
  wx = (ix + lower_left_voxels_[0]) * resolution_;
  wy = (iy + lower_left_voxels_[1]) * resolution_;
}

int OccupancyGrid2D::GridToIndex(int ix, int iy) const {
  int ix_off = (ix + lower_left_voxels_[0]) % grid_dims_[0];
  if (ix_off < 0) ix_off += grid_dims_[0];
  int iy_off = (iy + lower_left_voxels_[1]) % grid_dims_[1];
  if (iy_off < 0) iy_off += grid_dims_[1];
  return iy_off * grid_dims_[0] + ix_off;
}

void OccupancyGrid2D::WorldToVoxels(double x, double y, int& vx, int& vy) const {
  vx = static_cast<int>(std::floor(x / resolution_ + 1e-6));
  vy = static_cast<int>(std::floor(y / resolution_ + 1e-6));
}

void OccupancyGrid2D::ComputeLowerLeft(double cx, double cy, int new_ll[2]) const {
  int cv[2];
  WorldToVoxels(cx, cy, cv[0], cv[1]);
  for (int i = 0; i < 2; i++) {
    new_ll[i] = cv[i] - (grid_dims_[i] >> 1);
  }
}

void OccupancyGrid2D::GetSliceIndices(const int new_ll[2],
                                       std::vector<int>* indices) const {
  int shift[2];
  for (int i = 0; i < 2; i++) {
    shift[i] = new_ll[i] - lower_left_voxels_[i];
  }

  int clear_width[2];
  for (int i = 0; i < 2; i++) {
    clear_width[i] = std::min(std::abs(shift[i]), grid_dims_[i]);
  }

  // X slices
  if (shift[0] > 0) {
    for (int ix = 0; ix < clear_width[0]; ix++) {
      for (int iy = 0; iy < grid_dims_[1]; iy++) {
        indices->push_back(GridToIndex(ix, iy));
      }
    }
  } else if (shift[0] < 0) {
    for (int ix = grid_dims_[0] - clear_width[0]; ix < grid_dims_[0]; ix++) {
      for (int iy = 0; iy < grid_dims_[1]; iy++) {
        indices->push_back(GridToIndex(ix, iy));
      }
    }
  }

  // Y slices
  if (shift[1] > 0) {
    for (int iy = 0; iy < clear_width[1]; iy++) {
      for (int ix = 0; ix < grid_dims_[0]; ix++) {
        indices->push_back(GridToIndex(ix, iy));
      }
    }
  } else if (shift[1] < 0) {
    for (int iy = grid_dims_[1] - clear_width[1]; iy < grid_dims_[1]; iy++) {
      for (int ix = 0; ix < grid_dims_[0]; ix++) {
        indices->push_back(GridToIndex(ix, iy));
      }
    }
  }
}

void OccupancyGrid2D::ShiftOrigin(const int new_ll[2]) {
  lower_left_voxels_[0] = new_ll[0];
  lower_left_voxels_[1] = new_ll[1];
}

}  // namespace occupancy_grid_2d
