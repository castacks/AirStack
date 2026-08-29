#include "distance_grid/distance_grid.h"

namespace distance_grid {

DistanceGrid::DistanceGrid(const double origin[3], const double world_dimensions[3], const double resolution, int truncation_distance_voxels)
: voxel_grid::VoxelGrid<int>(origin, world_dimensions, resolution), dmax_(truncation_distance_voxels * truncation_distance_voxels) {
  Initialize();
}

void DistanceGrid::Initialize() {
  priority_queue_ = std::priority_queue<PriorityNode, std::vector<PriorityNode>, PriorityNodeCompareFunctor>();

  int num_cells = GetNumCells();

  Node node;
  nodes_.clear();
  
  for(int ii = 0; ii < num_cells; ii++) {
    node.obstacle = nullptr;
    node.in_q = false;
    node.index = ii;
    IndexToWorld(ii, node.xyz);
    WriteValue(ii, dmax_);
    nodes_.push_back(node);
  }
}

void DistanceGrid::PreShiftOrigin(const std::vector<int>& slice_indexes) {
  ClearIndexes(slice_indexes);
  UpdateDistances();
}

void DistanceGrid::PostShiftOrigin(const std::vector<int>& slice_indexes) {
  UpdateIndexes(slice_indexes);
  UpdateDistances();
}

void DistanceGrid::ClearIndexes(const std::vector<int>& slice_indexes) {
  for(int ind : slice_indexes) {
    if(ReadValue(ind) == 0) {
      RemoveObstacle(&nodes_[ind]);
    }
  }
}

void DistanceGrid::UpdateIndexes(const std::vector<int>& slice_indexes) {
  for(int ind : slice_indexes) {
    Node& node = nodes_[ind];
    IndexToWorld(ind, node.xyz);
    WriteValue(ind, 0);
    RemoveObstacle(&node);
  }
}

void DistanceGrid::SetObstacle(Node* node) {
  if(node && ReadValue(node->index) != 0) {
    WriteValue(node->index, 0);
    node->obstacle = node;
    Insert(0, node);
  } else {
    if(!node) {
      std::cout << "SetObstacle called with nullptr" << std::endl;
    } else {
      std::cout << "SetObstacle called on obstacle" << std::endl;
    }
  }
}

void DistanceGrid::SetObstacle(const double xyz[3]) {
  Node* node = GetNode(xyz);
  SetObstacle(node);
}

void DistanceGrid::SetObstacle(double x, double y, double z) {
  double point[3] = {x, y, z};
  SetObstacle(point);    
}

void DistanceGrid::RemoveObstacle(Node* node) {
  if (node && ReadValue(node->index) == 0) {
    WriteValue(node->index, dmax_);
    node->obstacle = nullptr;
    Insert(0, node);
  } else {
    if(!node) {
      std::cout << "RemoveObstacle called with nullptr" << std::endl;
    } else {
      std::cout << "RemoveObstacle called on Non-obstacle" << std::endl;
    }
  }
}

void DistanceGrid::RemoveObstacle(const double xyz[3]) {
  Node* node = GetNode(xyz);
  RemoveObstacle(node);
}

void DistanceGrid::RemoveObstacle(double x, double y, double z) {
  double point[3] = {x, y, z};
  RemoveObstacle(point);    
}

DistanceGrid::Node* DistanceGrid::GetNode(const double xyz[3]) {
  int ind = WorldToIndex(xyz);
  if(IsInMap(ind)) {
    return &nodes_[ind];
  } else {
    return nullptr;
  }
}

void DistanceGrid::UpdateDistances() {
  while (!priority_queue_.empty()) {
    Node* node = Pop();
    if (ReadValue(node->index) == dmax_) {
      Raise(node);
    } else {
      Lower(node);
    }
  }  
}

void DistanceGrid::PrintGrid() const {
  int grid_dims[3];
  GetGridDimensions(grid_dims);
  for(int z = 0; z < grid_dims[2]; z++){
    std::cout << "Z = " << z << std::endl << std::endl;
    for (int y = grid_dims[1] - 1; y >= 0; y--) {
      for (int x = 0; x < grid_dims[0]; x++) {
        int ixyz[3] = {x, y, z};
        int index = GridToIndex(ixyz);
        int dist_squared = ReadValue(index);
        std::string s = (dist_squared != dmax_ ? std::to_string(dist_squared) : " - ");
        printf("%3s", s.c_str());
        //std::cout << (dist_squared != dmax_ ? std::to_string(dist_squared) : " - ") << "   ";
      }
      std::cout << std::endl;
    }
    std::cout << std::endl;
  }
}

std::vector<DistanceGrid::Node*> DistanceGrid::GetNeighbors(const Node& node) {
  std::vector<Node*> neighbors;

  int ixyz[3];
  WorldToGrid(node.xyz, ixyz);
  
  int num_cells = GetNumCells();

  for(int dx = -1; dx <= 1; dx++) {
    for(int dy = -1; dy <= 1; dy++) {
      for(int dz = -1; dz <= 1; dz++) {
        if(dx == 0 && dy == 0 && dz ==0) { 
          continue; //skip middle cell
        }
        int neighbor_ixyz[3] = {ixyz[0] + dx, ixyz[1] + dy, ixyz[2] + dz};
        if(IsInMap(neighbor_ixyz)) {
          int neighbor_index = GridToIndex(neighbor_ixyz);
          neighbors.push_back(&nodes_[neighbor_index]);
        }
      }
    }
  }

  return neighbors;
}

bool DistanceGrid::IsValid(const Node& node) const {
  if (ReadValue(node.index) == 0 || (node.obstacle && ReadValue(node.obstacle->index) == 0)) {
    return true;
  } else {
    return false;
  }
}

int DistanceGrid::DistanceSquared(const Node& node, const Node& neighbor) const {
  // Round to the nearest voxel — a precomputed integer voxels_per_meter (e.g.
  // floor(1/0.15)=6 instead of 6.666) underestimates distances and makes cells
  // one voxel from an obstacle register as dist²=0, i.e. indistinguishable
  // from the obstacle itself.
  const double res = GetResolution();
  const int x_dist_voxels = static_cast<int>(std::lround((neighbor.xyz[0] - node.obstacle->xyz[0]) / res));
  const int y_dist_voxels = static_cast<int>(std::lround((neighbor.xyz[1] - node.obstacle->xyz[1]) / res));
  const int z_dist_voxels = static_cast<int>(std::lround((neighbor.xyz[2] - node.obstacle->xyz[2]) / res));

  return x_dist_voxels * x_dist_voxels + y_dist_voxels * y_dist_voxels + z_dist_voxels * z_dist_voxels;
}

void DistanceGrid::Lower(Node* node) {
  for (auto neighbor : GetNeighbors(*node)) {
    int d = DistanceSquared(*node, *neighbor);
    if (ReadValue(neighbor->index) > d) {
      WriteValue(neighbor->index, d);
      neighbor->obstacle = node->obstacle;
      if (!neighbor->in_q) {
        Insert(dmax_ + d, neighbor);
      }
    }
  }
}

void DistanceGrid::Raise(Node* node) {
  for (auto neighbor : GetNeighbors(*node)) {
    if (!neighbor->in_q) {

      int neighbor_dist_squared = ReadValue(neighbor->index);
      if (IsValid(*neighbor)) {
        Insert(dmax_ + neighbor_dist_squared, neighbor);
        continue;
      }
      if (neighbor_dist_squared != dmax_) {
        neighbor->obstacle = nullptr;
        Insert(neighbor_dist_squared, neighbor);
        WriteValue(neighbor->index, dmax_);
      }
    }
  }
}

void DistanceGrid::Insert(int priority, Node* node) {
  node->in_q = true;
  priority_queue_.push(std::make_pair(priority, node));
}

DistanceGrid::Node* DistanceGrid::Pop() {
  PriorityNode priority_node = priority_queue_.top();
  priority_queue_.pop();
  priority_node.second->in_q = false;
  return priority_node.second;
}

}  // namespace distance_grid
