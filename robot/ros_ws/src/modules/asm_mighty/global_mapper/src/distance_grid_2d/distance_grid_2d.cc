#include "distance_grid_2d/distance_grid_2d.h"

#include <algorithm>

namespace distance_grid_2d {

DistanceGrid2D::DistanceGrid2D()
    : resolution_(0.0), dmax_(0), num_cells_(0) {
  grid_dims_[0] = grid_dims_[1] = 0;
  lower_left_voxels_[0] = lower_left_voxels_[1] = 0;
}

DistanceGrid2D::DistanceGrid2D(const double origin[2],
                               const double world_dimensions[2],
                               double resolution,
                               int truncation_distance_voxels)
    : resolution_(resolution),
      dmax_(truncation_distance_voxels * truncation_distance_voxels) {
  for (int i = 0; i < 2; i++) {
    grid_dims_[i] = static_cast<int>(world_dimensions[i] / resolution_);
  }
  num_cells_ = grid_dims_[0] * grid_dims_[1];

  // Initialize origin
  int new_ll[2];
  ComputeLowerLeft(origin[0], origin[1], new_ll);
  lower_left_voxels_[0] = new_ll[0];
  lower_left_voxels_[1] = new_ll[1];

  Initialize();
}

void DistanceGrid2D::Initialize() {
  pq_ = std::priority_queue<PriorityNode, std::vector<PriorityNode>,
                            PriorityNodeCompare>();

  data_.assign(num_cells_, dmax_);
  nodes_.clear();
  nodes_.reserve(num_cells_);

  for (int i = 0; i < num_cells_; i++) {
    Node node;
    node.obstacle = nullptr;
    node.in_q = false;
    node.index = i;
    IndexToWorld(i, node.wx, node.wy);
    nodes_.push_back(node);
  }
}

// --- Public API ---

void DistanceGrid2D::SetObstacle(double x, double y) {
  Node* node = GetNode(x, y);
  SetObstacle(node);
}

void DistanceGrid2D::RemoveObstacle(double x, double y) {
  Node* node = GetNode(x, y);
  RemoveObstacle(node);
}

void DistanceGrid2D::UpdateDistances() {
  while (!pq_.empty()) {
    Node* node = Pop();
    if (data_[node->index] == dmax_) {
      Raise(node);
    } else {
      Lower(node);
    }
  }
}

void DistanceGrid2D::UpdateOrigin(double x, double y) {
  int new_ll[2];
  ComputeLowerLeft(x, y, new_ll);

  int shift[2];
  for (int i = 0; i < 2; i++) {
    shift[i] = new_ll[i] - lower_left_voxels_[i];
  }

  if (shift[0] == 0 && shift[1] == 0) return;

  std::vector<int> slice_indices;
  GetSliceIndices(new_ll, &slice_indices);

  PreShiftClear(slice_indices);
  UpdateDistances();
  ShiftOrigin(new_ll);
  PostShiftUpdate(slice_indices);
  UpdateDistances();
}

// --- Accessors ---

int DistanceGrid2D::ReadValue(int ix, int iy) const {
  if (!IsInMap(ix, iy)) return dmax_;
  return data_[GridToIndex(ix, iy)];
}

int DistanceGrid2D::ReadValue(double x, double y) const {
  int ix, iy;
  WorldToGrid(x, y, ix, iy);
  return ReadValue(ix, iy);
}

void DistanceGrid2D::GetGridDimensions(int dims[2]) const {
  dims[0] = grid_dims_[0];
  dims[1] = grid_dims_[1];
}

void DistanceGrid2D::GetOrigin(double xy[2]) const {
  for (int i = 0; i < 2; i++) {
    xy[i] = (lower_left_voxels_[i] + (grid_dims_[i] >> 1)) * resolution_;
  }
}

bool DistanceGrid2D::IsInMap(int ix, int iy) const {
  return ix >= 0 && ix < grid_dims_[0] && iy >= 0 && iy < grid_dims_[1];
}

bool DistanceGrid2D::IsInMap(double x, double y) const {
  int ix, iy;
  WorldToGrid(x, y, ix, iy);
  return IsInMap(ix, iy);
}

void DistanceGrid2D::WorldToGrid(double x, double y, int& ix, int& iy) const {
  int vx, vy;
  WorldToVoxels(x, y, vx, vy);
  ix = vx - lower_left_voxels_[0];
  iy = vy - lower_left_voxels_[1];
}

void DistanceGrid2D::GridToWorld(int ix, int iy, double& wx, double& wy) const {
  wx = (ix + lower_left_voxels_[0]) * resolution_;
  wy = (iy + lower_left_voxels_[1]) * resolution_;
}

// --- Private: obstacle manipulation ---

DistanceGrid2D::Node* DistanceGrid2D::GetNode(double x, double y) {
  int idx = WorldToIndex(x, y);
  if (idx >= 0 && idx < num_cells_) {
    return &nodes_[idx];
  }
  return nullptr;
}

void DistanceGrid2D::SetObstacle(Node* node) {
  if (node && data_[node->index] != 0) {
    data_[node->index] = 0;
    node->obstacle = node;
    Insert(0, node);
  }
}

void DistanceGrid2D::RemoveObstacle(Node* node) {
  if (node && data_[node->index] == 0) {
    data_[node->index] = dmax_;
    node->obstacle = nullptr;
    Insert(0, node);
  }
}

// --- Private: ESDF algorithm ---

void DistanceGrid2D::Lower(Node* node) {
  for (auto* neighbor : GetNeighbors(*node)) {
    int d = DistanceSquared(*node, *neighbor);
    if (data_[neighbor->index] > d) {
      data_[neighbor->index] = d;
      neighbor->obstacle = node->obstacle;
      if (!neighbor->in_q) {
        Insert(dmax_ + d, neighbor);
      }
    }
  }
}

void DistanceGrid2D::Raise(Node* node) {
  for (auto* neighbor : GetNeighbors(*node)) {
    if (!neighbor->in_q) {
      int neighbor_dist = data_[neighbor->index];
      if (IsValid(*neighbor)) {
        Insert(dmax_ + neighbor_dist, neighbor);
        continue;
      }
      if (neighbor_dist != dmax_) {
        neighbor->obstacle = nullptr;
        Insert(neighbor_dist, neighbor);
        data_[neighbor->index] = dmax_;
      }
    }
  }
}

void DistanceGrid2D::Insert(int priority, Node* node) {
  node->in_q = true;
  pq_.push(std::make_pair(priority, node));
}

DistanceGrid2D::Node* DistanceGrid2D::Pop() {
  PriorityNode pn = pq_.top();
  pq_.pop();
  pn.second->in_q = false;
  return pn.second;
}

int DistanceGrid2D::DistanceSquared(const Node& from,
                                    const Node& neighbor) const {
  // Round to the nearest voxel — truncating an integer voxels_per_meter (e.g.
  // floor(1/0.15)=6 instead of 6.666) underestimates distances and makes cells
  // one voxel from an obstacle register as dist²=0, i.e. indistinguishable
  // from the obstacle itself.
  const int dx = static_cast<int>(std::lround((neighbor.wx - from.obstacle->wx) / resolution_));
  const int dy = static_cast<int>(std::lround((neighbor.wy - from.obstacle->wy) / resolution_));
  return dx * dx + dy * dy;
}

std::vector<DistanceGrid2D::Node*> DistanceGrid2D::GetNeighbors(
    const Node& node) {
  std::vector<Node*> neighbors;
  neighbors.reserve(8);

  int ix, iy;
  WorldToGrid(node.wx, node.wy, ix, iy);

  for (int dx = -1; dx <= 1; dx++) {
    for (int dy = -1; dy <= 1; dy++) {
      if (dx == 0 && dy == 0) continue;
      int nx = ix + dx;
      int ny = iy + dy;
      if (IsInMap(nx, ny)) {
        int idx = GridToIndex(nx, ny);
        neighbors.push_back(&nodes_[idx]);
      }
    }
  }

  return neighbors;
}

bool DistanceGrid2D::IsValid(const Node& node) const {
  if (data_[node.index] == 0 ||
      (node.obstacle && data_[node.obstacle->index] == 0)) {
    return true;
  }
  return false;
}

// --- Private: coordinate transforms ---

int DistanceGrid2D::GridToIndex(int ix, int iy) const {
  int ix_off = (ix + lower_left_voxels_[0]) % grid_dims_[0];
  if (ix_off < 0) ix_off += grid_dims_[0];
  int iy_off = (iy + lower_left_voxels_[1]) % grid_dims_[1];
  if (iy_off < 0) iy_off += grid_dims_[1];
  return iy_off * grid_dims_[0] + ix_off;
}

void DistanceGrid2D::IndexToGrid(int idx, int& ix, int& iy) const {
  int raw_iy = idx / grid_dims_[0];
  int raw_ix = idx - raw_iy * grid_dims_[0];
  ix = (raw_ix - lower_left_voxels_[0]) % grid_dims_[0];
  if (ix < 0) ix += grid_dims_[0];
  iy = (raw_iy - lower_left_voxels_[1]) % grid_dims_[1];
  if (iy < 0) iy += grid_dims_[1];
}

void DistanceGrid2D::IndexToWorld(int idx, double& wx, double& wy) const {
  int ix, iy;
  IndexToGrid(idx, ix, iy);
  GridToWorld(ix, iy, wx, wy);
}

int DistanceGrid2D::WorldToIndex(double x, double y) const {
  int ix, iy;
  WorldToGrid(x, y, ix, iy);
  if (!IsInMap(ix, iy)) return -1;
  return GridToIndex(ix, iy);
}

void DistanceGrid2D::WorldToVoxels(double x, double y, int& vx, int& vy) const {
  vx = static_cast<int>(std::floor(x / resolution_ + 1e-6));
  vy = static_cast<int>(std::floor(y / resolution_ + 1e-6));
}

void DistanceGrid2D::ComputeLowerLeft(double cx, double cy,
                                      int new_ll[2]) const {
  int cv[2];
  WorldToVoxels(cx, cy, cv[0], cv[1]);
  for (int i = 0; i < 2; i++) {
    new_ll[i] = cv[i] - (grid_dims_[i] >> 1);
  }
}

void DistanceGrid2D::GetSliceIndices(const int new_ll[2],
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

void DistanceGrid2D::ShiftOrigin(const int new_ll[2]) {
  lower_left_voxels_[0] = new_ll[0];
  lower_left_voxels_[1] = new_ll[1];
}

void DistanceGrid2D::PreShiftClear(const std::vector<int>& indices) {
  for (int idx : indices) {
    if (data_[idx] == 0) {
      RemoveObstacle(&nodes_[idx]);
    }
  }
}

void DistanceGrid2D::PostShiftUpdate(const std::vector<int>& indices) {
  for (int idx : indices) {
    Node& node = nodes_[idx];
    IndexToWorld(idx, node.wx, node.wy);
    data_[idx] = 0;
    RemoveObstacle(&node);
  }
}

}  // namespace distance_grid_2d
