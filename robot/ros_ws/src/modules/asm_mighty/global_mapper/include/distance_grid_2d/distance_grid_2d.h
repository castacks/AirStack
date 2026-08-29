// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <cmath>
#include <cstring>
#include <iostream>
#include <queue>
#include <vector>

namespace distance_grid_2d {

class DistanceGrid2D {
 public:
  DistanceGrid2D();
  DistanceGrid2D(const double origin[2], const double world_dimensions[2],
                 double resolution, int truncation_distance_voxels);

  void SetObstacle(double x, double y);
  void RemoveObstacle(double x, double y);
  void UpdateDistances();
  void UpdateOrigin(double x, double y);

  int ReadValue(int ix, int iy) const;
  int ReadValue(double x, double y) const;
  void GetGridDimensions(int dims[2]) const;
  double GetResolution() const { return resolution_; }
  void GetOrigin(double xy[2]) const;
  int GetMaxSquaredDistance() const { return dmax_; }
  bool IsInMap(int ix, int iy) const;
  bool IsInMap(double x, double y) const;
  void WorldToGrid(double x, double y, int& ix, int& iy) const;
  void GridToWorld(int ix, int iy, double& wx, double& wy) const;
  int GetNumCells() const { return num_cells_; }

 private:
  struct Node {
    Node* obstacle = nullptr;
    bool in_q = false;
    int index = 0;
    double wx = 0.0, wy = 0.0;  // world coordinates
  };

  using PriorityNode = std::pair<int, Node*>;

  struct PriorityNodeCompare {
    bool operator()(const PriorityNode& a, const PriorityNode& b) const {
      return a.first > b.first;
    }
  };

  void Initialize();
  void Lower(Node* node);
  void Raise(Node* node);
  void Insert(int priority, Node* node);
  Node* Pop();
  int GridToIndex(int ix, int iy) const;
  void IndexToGrid(int idx, int& ix, int& iy) const;
  void IndexToWorld(int idx, double& wx, double& wy) const;
  int WorldToIndex(double x, double y) const;
  int DistanceSquared(const Node& from, const Node& neighbor) const;
  std::vector<Node*> GetNeighbors(const Node& node);
  bool IsValid(const Node& node) const;
  void WorldToVoxels(double x, double y, int& vx, int& vy) const;
  void ComputeLowerLeft(double cx, double cy, int new_ll[2]) const;
  void GetSliceIndices(const int new_ll[2], std::vector<int>* indices) const;
  void ShiftOrigin(const int new_ll[2]);
  void PreShiftClear(const std::vector<int>& indices);
  void PostShiftUpdate(const std::vector<int>& indices);

  Node* GetNode(double x, double y);
  void SetObstacle(Node* node);
  void RemoveObstacle(Node* node);

  int grid_dims_[2];
  int lower_left_voxels_[2];
  double resolution_;
  int dmax_;
  int num_cells_;
  std::vector<int> data_;
  std::vector<Node> nodes_;
  std::priority_queue<PriorityNode, std::vector<PriorityNode>, PriorityNodeCompare> pq_;
};

}  // namespace distance_grid_2d
