// Copyright 2017 Massachusetts Institute of Technology
#pragma once

#include <math.h>
#include <limits>
#include <cstdint>
#include <vector>
#include <array>
#include <iostream>
#include <queue>

#include "voxel_grid/voxel_grid.h"
#include "distance_grid/distance_grid.h"
#include "occupancy_grid/occupancy_grid.h"

namespace cost_grid {

const int MAX_COST = 1e6;

class CostGrid : public voxel_grid::VoxelGrid<int> {
 public:
  CostGrid(const double origin[3], const double world_dimensions[3], const double resolution);
  void SetDistanceGrid(distance_grid::DistanceGrid* dt);
  void SetOccupancyGrid(occupancy_grid::OccupancyGrid* og);
  void UpdateDijkstra(const double goal[3]);
  void PrintGraph();
  void PrintNeighbors();
  void SetCostWeights(int altitude_weight, int inflation_weight, int unknown_weight, int obstacle_weight);
  void SetInflationDistance(int inflation_distance) { inflation_distance_ = inflation_distance; }
  void GetPaths(double x, double y, double z, std::vector<std::array<double, 3>>* dense_path, std::vector<std::array<double, 3>>* sparse_path);
  void GetPaths(double xyz[3], std::vector<std::array<double, 3>>* dense_path, std::vector<std::array<double, 3>>* sparse_path);
  int GetLineCost(const double xyz1[3], const double xyz2[3]) const;
  int SparsifyPath(const std::vector<std::array<double, 3>>& dense_path,
                           std::vector<std::array<double, 3>>* sparse_path) const;
  bool GetProjectedGoal(double xyz[3]) const;

 private:
  struct Node {
    Node* parent = nullptr;
    bool visited = false;
    int index = 0;
    std::vector<Node*> neighbors;
  };

  using PriorityNode = std::pair<int, Node*>;

  void Initialize();
  void UpdateDijkstra();
  void ClearNeighbors();
  void ResetNode(Node* node);
  void ResetAllNodes();
  Node* GetClosestEdgeNodeToGoal();
  void SetGoal(const double goal[3]);
  void UpdateAllNeighbors();
  int GetEdgeCost(const Node* node1_ptr, const Node* node2_ptr) const;
  int GetManhattanDistanceToGoal(const Node* node) const ;
  virtual void PreShiftOrigin(const std::vector<int>& slice_indexes) override;
  virtual void PostShiftOrigin(const std::vector<int>& slice_indexes) override;
  std::vector<Node*> GetNeighbors(Node* node);
  void UpdateEdgeGridCoords();
  void UpdateNeighbors(Node* node);
  void Insert(int priority, Node* node);
  Node* Pop();

  struct PriorityNodeCompareFunctor {
    bool operator()(const PriorityNode& pn1, const PriorityNode& pn2) const {
      return pn1.first > pn2.first;
    }
  };


  distance_grid::DistanceGrid* distance_grid_ptr_;
  occupancy_grid::OccupancyGrid* occupancy_grid_ptr_;
  
  std::priority_queue<PriorityNode, std::vector<PriorityNode>, PriorityNodeCompareFunctor> priority_queue_;
  Node* goal_ptr_;
  int goal_ixyz_[3] = {0, 0, 0};
  double goal_xyz_[3];
  std::vector<Node> nodes_;  
  std::set<Node*> neighbor_set_;
  int goal_voxels_[3];
  std::vector<std::array<int, 3>> edge_grid_coords_;
  int inflation_distance_;
  int altitude_weight_;
  int inflation_weight_;
  int unknown_weight_;
  int obstacle_weight_;
};

}  // namespace cost_grid
