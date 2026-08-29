// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <cmath>
#include <cstdint>
#include <vector>
#include <iostream>
#include <queue>
#include <set>

#include "voxel_grid/voxel_grid.h"

namespace distance_grid {

class DistanceGrid : public voxel_grid::VoxelGrid<int> {
 public:
  DistanceGrid(const double origin[3], const double world_dimensions[3], const double resolution, int truncation_distance_voxels);
  void SetObstacle(const double xyz[3]);
  void SetObstacle(double x, double y, double z);
  void RemoveObstacle(const double xyz[3]);
  void RemoveObstacle(double x, double y, double z);
  int GetMaxSquaredDistance() const { return dmax_; }
  void UpdateDistances();
  void PrintGrid() const;
  
 private:
  struct Node {
    Node* obstacle;
    bool in_q;
    int index;
    double xyz[3];
  };
  
  using PriorityNode = std::pair<int, Node*>;

  void SetObstacle(Node* node);
  void RemoveObstacle(Node* node);
  void Initialize();
  virtual void PreShiftOrigin(const std::vector<int>& slice_indexes) override;
  virtual void PostShiftOrigin(const std::vector<int>& slice_indexes) override;
  void ClearIndexes(const std::vector<int>& slice_indexes);
  void UpdateIndexes(const std::vector<int>& slice_indexes);
  Node* GetNode(const double xyz[3]);
  std::vector<Node*> GetNeighbors(const Node& node);
  bool IsValid(const Node& node) const;
  int DistanceSquared(const Node& node, const Node& neighbor) const;
  void RegisterUpdate(Node* node);
  void Lower(Node* node);
  void Raise(Node* node);
  void Insert(int priority, Node* node);
  Node* Pop();

  struct PriorityNodeCompareFunctor {
    bool operator()(const PriorityNode& pn1, const PriorityNode& pn2) const {
      return pn1.first > pn2.first;
    }
  };

  const int dmax_;
  std::vector<Node> nodes_;
  std::priority_queue<PriorityNode, std::vector<PriorityNode>, PriorityNodeCompareFunctor> priority_queue_;
};

}  // namespace distance_grid
