#include "occupancy_grid/occupancy_grid.h"

namespace occupancy_grid
{
OccupancyGrid::OccupancyGrid(const double origin[3], const double world_dimensions[3], const double resolution,
                             float threshold)
  : voxel_grid::VoxelGrid<float>(origin, world_dimensions, resolution), threshold_(threshold)
{
  int num_cells = GetNumCells();
  Reset(-1);
  double indexer_origin[3];
  GetOrigin(indexer_origin);
  UpdateOrigin(indexer_origin);
}

void OccupancyGrid::PreShiftOrigin(const std::vector<int>& slice_indexes)
{
  // nothing
}

void OccupancyGrid::PostShiftOrigin(const std::vector<int>& slice_indexes)
{
  for (int index : slice_indexes)
  {
    WriteValue(index, -1);
  }
}

bool OccupancyGrid::IsOccupied(const int ixyz[3]) const
{
  return IsOccupied(ReadValue(ixyz));
}

bool OccupancyGrid::IsOccupied(const double xyz[3]) const
{
  return IsOccupied(ReadValue(xyz));
}

bool OccupancyGrid::IsOccupied(float value) const
{
  return value > threshold_;
}

bool OccupancyGrid::IsUnknown(const int ixyz[3]) const
{
  return IsUnknown(ReadValue(ixyz));
}

bool OccupancyGrid::IsUnknown(const double xyz[3]) const
{
  return IsUnknown(ReadValue(xyz));
}
bool OccupancyGrid::IsUnknown(float value) const
{
  return value < 0;
}

void OccupancyGrid::UpdateValue(const int ixyz[3], float value)
{
  int ind = GridToIndex(ixyz);
  UpdateValue(ind, value);
}

void OccupancyGrid::UpdateValue(const double xyz[3], float value)
{
  int ixyz[3];
  WorldToGrid(xyz, ixyz);
  if (IsInMap(ixyz))
  {
    UpdateValue(ixyz, value);
  }
  else
  {
    return;
  }
}

bool OccupancyGrid::IsOccupied(const int ind) const
{
  return ReadValue(ind) > threshold_;
}

void OccupancyGrid::UpdateValue(const int ind, float delta)
{
  float old_value = ReadValue(ind);

  if (old_value < 0) {
    WriteValue(ind, 0.0f);
    old_value = 0.0f;
  }

  bool occupied_before = (old_value > threshold_);
  float new_value = clamp_value(old_value + delta, 0.0f, 1.0f);
  WriteValue(ind, new_value);

  bool occupied_after = (new_value > threshold_);
  if (!occupied_before && occupied_after) {
  }
  if (occupied_before && !occupied_after) {
  }
}


void OccupancyGrid::RayTrace(const int start[3], const int end[3], float increment)
{
  // 3D Bresenham implimentation copied from:
  // http://www.cit.griffith.edu.au/~anthony/info/graphics/bresenham.procs

  int x1, y1, z1, x2, y2, z2;
  x1 = start[0];
  y1 = start[1];
  z1 = start[2];
  x2 = end[0];
  y2 = end[1];
  z2 = end[2];
  int i, dx, dy, dz, l, m, n, x_inc, y_inc, z_inc, err_1, err_2, dx2, dy2, dz2;
  int voxel[3];

  voxel[0] = x1;
  voxel[1] = y1;
  voxel[2] = z1;
  dx = x2 - x1;
  dy = y2 - y1;
  dz = z2 - z1;
  x_inc = (dx < 0) ? -1 : 1;
  l = abs(dx);
  y_inc = (dy < 0) ? -1 : 1;
  m = abs(dy);
  z_inc = (dz < 0) ? -1 : 1;
  n = abs(dz);
  dx2 = l << 1;
  dy2 = m << 1;
  dz2 = n << 1;

  if ((l >= m) && (l >= n))
  {
    err_1 = dy2 - l;
    err_2 = dz2 - l;
    for (i = 0; i <= l && IsInMap(voxel); i++)
    {
      UpdateValue(voxel, increment);
      if (err_1 > 0)
      {
        voxel[1] += y_inc;
        err_1 -= dx2;
      }
      if (err_2 > 0)
      {
        voxel[2] += z_inc;
        err_2 -= dx2;
      }
      err_1 += dy2;
      err_2 += dz2;
      voxel[0] += x_inc;
    }
  }
  else if ((m >= l) && (m >= n))
  {
    err_1 = dx2 - m;
    err_2 = dz2 - m;
    for (i = 0; i <= m && IsInMap(voxel); i++)
    {
      UpdateValue(voxel, increment);
      if (err_1 > 0)
      {
        voxel[0] += x_inc;
        err_1 -= dy2;
      }
      if (err_2 > 0)
      {
        voxel[2] += z_inc;
        err_2 -= dy2;
      }
      err_1 += dx2;
      err_2 += dz2;
      voxel[1] += y_inc;
    }
  }
  else
  {
    err_1 = dy2 - n;
    err_2 = dx2 - n;
    for (i = 0; i <= n && IsInMap(voxel); i++)
    {
      UpdateValue(voxel, increment);

      if (err_1 > 0)
      {
        voxel[1] += y_inc;
        err_1 -= dz2;
      }
      if (err_2 > 0)
      {
        voxel[0] += x_inc;
        err_2 -= dz2;
      }
      err_1 += dy2;
      err_2 += dx2;
      voxel[2] += z_inc;
    }
  }
}

void OccupancyGrid::RayTrace(const double start[3], const double end[3], float increment)
{
  int istart[3];
  int iend[3];
  WorldToGrid(start, istart);
  WorldToGrid(end, iend);
  if (IsInMap(istart))
  {
    RayTrace(istart, iend, increment);
  }
  else
  {
    return;
  }
}

float OccupancyGrid::clamp_value(float x, float min, float max) const
{
  if (x < min)
    return min;
  if (x > max)
    return max;
  return x;
}

}  // namespace occupancy_grid
