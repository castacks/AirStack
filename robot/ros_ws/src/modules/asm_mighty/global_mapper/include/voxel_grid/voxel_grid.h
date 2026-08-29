// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

#include <vector>
#include <cstring>
#include <iostream>

namespace voxel_grid
{
template <typename T>
class VoxelGrid
{
public:
  VoxelGrid();
  VoxelGrid(const double origin[3], const double world_dimensions[3], const double resolution);
  VoxelGrid(const double origin[3], const int grid_dimensions[3], const double resolution, const std::vector<T>& data);
  void GetGridDimensions(int dimensions[3]) const
  {
    memcpy(dimensions, grid_dimensions_, 3 * sizeof(int));
  }
  void WorldToGrid(const double xyz[3], int ixyz[3]) const;
  void GridToWorld(const int ixyz[3], double* xyz) const;
  bool IsInMap(int index) const;
  bool IsInMap(const int ixyz[3]) const;
  bool IsInMap(const double xyz[3]) const;
  std::vector<int> UpdateOrigin(const double xyz[3]);
  std::vector<int> UpdateOrigin(const double x, const double y, const double z);
  int GridToIndex(const int ixyz[3]) const;
  int WorldToIndex(const double xyz[3]) const;
  void IndexToGrid(int ind, int ixyz[3]) const;
  void IndexToWorld(int ind, double xyz[3]) const;
  void SnapToGrid(const double xyz[3], double grid_xyz[3]) const;
  virtual void PreShiftOrigin(const std::vector<int>& slice_indexes)
  {
  }
  virtual void PostShiftOrigin(const std::vector<int>& slice_indexes)
  {
  }
  void WorldToVoxels(const double xyz[3], int voxels[3]) const;
  int GetNumCells() const
  {
    return num_cells_;
  }
  std::vector<T> GetData() const
  {
    return data_;
  }
  double GetResolution() const
  {
    return resolution_;
  }
  void GetOrigin(double xyz[3]) const;
  void PrintIndexes() const;
  void PrintValues() const;
  void PrintWorldX() const;
  void PrintWorldZ() const;
  void PrintState() const;
  void ComputeShiftVoxels(const int new_lower_left_voxels[3], int voxel_shift[3]) const;
  void Reset(T reset_value);
  T ReadValue(const int ind) const;
  T ReadValue(const int ixyz[3]) const;
  T ReadValue(const double xyz[3]) const;
  T ReadValue(const double x, const double y, const double z) const;
  void WriteValue(const int ixyz[3], T value);
  void WriteValue(const double xyz[3], T value);
  void WriteValue(const double x, const double y, const double z, T value);
  void WriteValue(const int ind, T value);

  T& GetReference(const int ind);

private:
  void ShiftOrigin(const int new_lower_left_voxels[3]);
  void ComputeLowerLeftVoxels(const double new_grid_center_xyz[3], int new_lower_left_voxels[3]) const;
  void GetSlice(const int i, const int width, const int dimension, std::vector<int>* slice_indexes) const;
  std::vector<int> GetSliceIndexes(const int new_lower_left_voxels[3]) const;

  void InitializeData()
  {
    num_cells_ = grid_dimensions_[0] * grid_dimensions_[1] * grid_dimensions_[2];
    for (int i = 0; i < num_cells_; i++)
    {
      data_.push_back(T());
    }
  }

  void InitializeData(const std::vector<T>& data)
  {
    num_cells_ = grid_dimensions_[0] * grid_dimensions_[1] * grid_dimensions_[2];
    data_ = data;
  }

  void InitializeOrigin(const double origin[3])
  {
    int new_lower_left_voxels[3] = { 0 };
    ComputeLowerLeftVoxels(origin, new_lower_left_voxels);
    ShiftOrigin(new_lower_left_voxels);
  }

  int lower_left_voxels_[3];
  double resolution_;
  int grid_dimensions_[3];  // map bounds in grid coordinates
  int num_cells_;
  std::vector<T> data_;
};

template <typename T>
VoxelGrid<T>::VoxelGrid()
{
  for (int i = 0; i < 3; i++)
  {
    lower_left_voxels_[i] = 0;
    grid_dimensions_[i] = 0;
    num_cells_ = 0;
    resolution_ = 0;
  }
}

template <typename T>
VoxelGrid<T>::VoxelGrid(const double origin[3], const double world_dimensions[3], const double resolution)
  : resolution_(resolution)
{
  for (int i = 0; i < 3; i++)
  {
    grid_dimensions_[i] = static_cast<int>(world_dimensions[i] / resolution_);
  }
  InitializeData();
  InitializeOrigin(origin);
}

template <typename T>
VoxelGrid<T>::VoxelGrid(const double origin[3], const int grid_dimensions[3], const double resolution,
                        const std::vector<T>& data)
  : resolution_(resolution)
{
  memcpy(grid_dimensions_, grid_dimensions, sizeof(int) * 3);
  InitializeData(data);
  InitializeOrigin(origin);
  if (data.size() != num_cells_)
  {
    std::cout << "VoxelGrid Error: Data vector size does not match grid dimensions.  Aborting." << std::endl;
    exit(1);
  }
}

template <typename T>
T& VoxelGrid<T>::GetReference(const int ind)
{
  return data_[ind];
}

template <typename T>
void VoxelGrid<T>::Reset(T reset_value)
{
  for (int i = 0; i < num_cells_; i++)
  {
    WriteValue(i, reset_value);
  }
}

template <typename T>
T VoxelGrid<T>::ReadValue(const int ind) const
{
  return data_[ind];
}

template <typename T>
T VoxelGrid<T>::ReadValue(const int ixyz[3]) const
{
  int ind = GridToIndex(ixyz);
  return ReadValue(ind);
}

template <typename T>
T VoxelGrid<T>::ReadValue(const double xyz[3]) const
{
  int ixyz[3];
  WorldToGrid(xyz, ixyz);
  return ReadValue(ixyz);
}

template <typename T>
T VoxelGrid<T>::ReadValue(const double x, const double y, const double z) const
{
  double xyz[3] = { x, y, z };
  return ReadValue(xyz);
}

template <typename T>
void VoxelGrid<T>::WriteValue(const int ixyz[3], T value)
{
  int ind = GridToIndex(ixyz);
  WriteValue(ind, value);
}

template <typename T>
void VoxelGrid<T>::WriteValue(const double xyz[3], T value)
{
  int ixyz[3];
  WorldToGrid(xyz, ixyz);
  WriteValue(ixyz, value);
}

template <typename T>
void VoxelGrid<T>::WriteValue(const int ind, T value)
{
  data_[ind] = value;
}

template <typename T>
void VoxelGrid<T>::WriteValue(const double x, const double y, const double z, T value)
{
  double xyz[3] = { x, y, z };
  WriteValue(xyz, value);
}

template <typename T>
void VoxelGrid<T>::ComputeLowerLeftVoxels(const double new_grid_center_xyz[3], int new_lower_left_voxels[3]) const
{
  int new_grid_center_voxels[3];
  WorldToVoxels(new_grid_center_xyz, new_grid_center_voxels);
  for (int i = 0; i < 3; i++)
  {
    new_lower_left_voxels[i] = new_grid_center_voxels[i] - (grid_dimensions_[i] >> 1);
  }
}

template <typename T>
int VoxelGrid<T>::GridToIndex(const int ixyz[3]) const
{
  int ixyz_offset[3];
  for (int i = 0; i < 3; i++)
  {
    ixyz_offset[i] = (ixyz[i] + lower_left_voxels_[i]) % grid_dimensions_[i];
    if (ixyz_offset[i] < 0)
    {
      ixyz_offset[i] += grid_dimensions_[i];
    }
  }
  int index = ixyz_offset[2] * (grid_dimensions_[0] * grid_dimensions_[1]) + ixyz_offset[1] * grid_dimensions_[0] +
              ixyz_offset[0];
  return index;
}

template <typename T>
void VoxelGrid<T>::PrintState() const
{
  printf("resolution: %0.2f\n", resolution_);
  printf("grid_dimensions: %i, %i, %i\n", grid_dimensions_[0], grid_dimensions_[1], grid_dimensions_[2]);
  printf("num_cells: %i\n", num_cells_);
  printf("lower_left_voxels: %i, %i, %i\n", lower_left_voxels_[0], lower_left_voxels_[1], lower_left_voxels_[2]);
}

template <typename T>
int VoxelGrid<T>::WorldToIndex(const double xyz[3]) const
{
  int ixyz[3];
  WorldToGrid(xyz, ixyz);
  return GridToIndex(ixyz);
}

template <typename T>
void VoxelGrid<T>::SnapToGrid(const double xyz[3], double grid_xyz[3]) const
{
  int ixyz[3];
  WorldToGrid(xyz, ixyz);
  GridToWorld(ixyz, grid_xyz);
}

template <typename T>
void VoxelGrid<T>::IndexToGrid(int ind, int ixyz[3]) const
{
  const int cells_per_floor = grid_dimensions_[0] * grid_dimensions_[1];
  const int cells_per_row = grid_dimensions_[0];
  ixyz[2] = ind / cells_per_floor;
  ind -= ixyz[2] * cells_per_floor;
  ixyz[1] = ind / cells_per_row;
  ind -= ixyz[1] * cells_per_row;
  ixyz[0] = ind;

  for (int i = 0; i < 3; i++)
  {
    ixyz[i] = (ixyz[i] - lower_left_voxels_[i]) % grid_dimensions_[i];
    if (ixyz[i] < 0)
    {
      ixyz[i] += grid_dimensions_[i];
    }
  }
}

template <typename T>
void VoxelGrid<T>::IndexToWorld(int ind, double xyz[3]) const
{
  int ixyz[3];
  IndexToGrid(ind, ixyz);
  GridToWorld(ixyz, xyz);
}

template <typename T>
void VoxelGrid<T>::WorldToGrid(const double xyz[3], int ixyz[3]) const
{
  int world_voxels[3];
  WorldToVoxels(xyz, world_voxels);
  for (int i = 0; i < 3; i++)
  {
    ixyz[i] = world_voxels[i] - lower_left_voxels_[i];
  }
}

template <typename T>
void VoxelGrid<T>::GridToWorld(const int ixyz[3], double* xyz) const
{
  for (int i = 0; i < 3; i++)
  {
    xyz[i] = (ixyz[i] + lower_left_voxels_[i]) * resolution_;
  }
}

template <typename T>
bool VoxelGrid<T>::IsInMap(const int ixyz[3]) const
{
  for (int i = 0; i < 3; i++)
  {
    // printf("ixyz[%i] = %i\n", i, ixyz[i]);
    if (ixyz[i] < 0 || ixyz[i] >= grid_dimensions_[i])
    {
      return false;
    }
  }
  return true;
}

template <typename T>
bool VoxelGrid<T>::IsInMap(const double xyz[3]) const
{
  int ixyz[3];
  WorldToGrid(xyz, ixyz);
  return IsInMap(ixyz);
}

template <typename T>
bool VoxelGrid<T>::IsInMap(int index) const
{
  return index >= 0 && index < num_cells_;
}

template <typename T>
void VoxelGrid<T>::GetSlice(const int i, const int width, const int dimension, std::vector<int>* slice_indexes) const
{
  // set minimum dimensions
  int ixyz_min[3] = { 0, 0, 0 };
  ixyz_min[dimension] = i;

  // set max dimensions
  int ixyz_max[3] = { grid_dimensions_[0], grid_dimensions_[1], grid_dimensions_[2] };
  ixyz_max[dimension] = i + width;

  int ixyz[3];
  for (int ix = ixyz_min[0]; ix < ixyz_max[0]; ix++)
  {
    for (int iy = ixyz_min[1]; iy < ixyz_max[1]; iy++)
    {
      for (int iz = ixyz_min[2]; iz < ixyz_max[2]; iz++)
      {
        ixyz[0] = ix;
        ixyz[1] = iy;
        ixyz[2] = iz;
        int ind = GridToIndex(ixyz);
        slice_indexes->push_back(ind);
      }
    }
  }
}

template <typename T>
void VoxelGrid<T>::ComputeShiftVoxels(const int new_lower_left_voxels[3], int voxel_shift[3]) const
{
  for (int i = 0; i < 3; i++)
  {
    voxel_shift[i] = new_lower_left_voxels[i] - lower_left_voxels_[i];
  }
}

template <typename T>
void VoxelGrid<T>::WorldToVoxels(const double xyz[3], int voxels[3]) const
{
  for (int i = 0; i < 3; i++)
  {
    double float_voxels = floor(xyz[i] / resolution_ + 1e-6);
    voxels[i] = static_cast<int>(float_voxels);
  }
}

// Update lower_left_voxels
template <typename T>
void VoxelGrid<T>::ShiftOrigin(const int new_lower_left_voxels[3])
{
  memcpy(lower_left_voxels_, new_lower_left_voxels, sizeof(int) * 3);
}

template <typename T>
void VoxelGrid<T>::PrintIndexes() const
{
  for (int y = grid_dimensions_[1] - 1; y >= 0; y--)
  {
    for (int x = 0; x < grid_dimensions_[0]; x++)
    {
      int ixyz[3] = { x, y, 0 };
      int index = GridToIndex(ixyz);
      printf("%i ", index);
    }
    printf("\n");
  }
  printf("\n");
}

template <typename T>
void VoxelGrid<T>::PrintValues() const
{
  for (int y = grid_dimensions_[1] - 1; y >= 0; y--)
  {
    for (int x = 0; x < grid_dimensions_[0]; x++)
    {
      int ixyz[3] = { x, y, 0 };
      int index = GridToIndex(ixyz);
      std::cout << ReadValue(index) << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

template <typename T>
void VoxelGrid<T>::PrintWorldX() const
{
  for (int y = grid_dimensions_[1] - 1; y >= 0; y--)
  {
    for (int x = 0; x < grid_dimensions_[0]; x++)
    {
      int ixyz[3] = { x, y, 0 };
      double xyz[3];
      GridToWorld(ixyz, xyz);
      printf("%0.2f ", xyz[0]);
    }
    printf("\n");
  }
  printf("\n");
}

template <typename T>
void VoxelGrid<T>::PrintWorldZ() const
{
  for (int y = grid_dimensions_[1] - 1; y >= 0; y--)
  {
    for (int x = 0; x < grid_dimensions_[0]; x++)
    {
      int ixyz[3] = { x, y, 0 };
      double xyz[3];
      GridToWorld(ixyz, xyz);
      printf("%0.2f ", xyz[2]);
    }
    printf("\n");
  }
  printf("\n");
}

template <typename T>
std::vector<int> VoxelGrid<T>::GetSliceIndexes(const int new_lower_left_voxels[3]) const
{
  int clear_width[3];
  int shift_voxels[3];
  ComputeShiftVoxels(new_lower_left_voxels, shift_voxels);

  for (int i = 0; i < 3; i++)
  {
    clear_width[i] = std::min(abs(shift_voxels[i]), grid_dimensions_[i]);
  }

  std::vector<int> slice_inds;
  for (int i = 0; i < 3; i++)
  {
    if (shift_voxels[i] > 0)
    {
      GetSlice(0, clear_width[i], i, &slice_inds);
    }
    else if (shift_voxels[i] < 0)
    {
      GetSlice(grid_dimensions_[i] - clear_width[i], clear_width[i], i, &slice_inds);
    }
  }
  return slice_inds;
}

template <typename T>
std::vector<int> VoxelGrid<T>::UpdateOrigin(const double x, const double y, const double z)
{
  double xyz[3] = { x, y, z };
  return UpdateOrigin(xyz);
}

template <typename T>
std::vector<int> VoxelGrid<T>::UpdateOrigin(const double new_grid_center[3])
{
  int new_lower_left_voxels[3];
  int voxel_shift[3];
  std::vector<int> slice_inds;
  ComputeLowerLeftVoxels(new_grid_center, new_lower_left_voxels);
  ComputeShiftVoxels(new_lower_left_voxels, voxel_shift);

  if (voxel_shift[0] == 0 && voxel_shift[1] == 0 && voxel_shift[2] == 0)
  {
    return slice_inds;
  }

  slice_inds = GetSliceIndexes(new_lower_left_voxels);
  PreShiftOrigin(slice_inds);
  ShiftOrigin(new_lower_left_voxels);
  PostShiftOrigin(slice_inds);
  return slice_inds;
}

template <typename T>
void VoxelGrid<T>::GetOrigin(double xyz[3]) const
{
  for (int i = 0; i < 3; i++)
  {
    xyz[i] = (lower_left_voxels_[i] + (grid_dimensions_[i] >> 1)) * resolution_;
  }
}

}  // namespace voxel_grid
