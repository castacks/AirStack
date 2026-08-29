#include "cost_grid/cost_grid.h"

namespace cost_grid
{
CostGrid::CostGrid(const double origin[3], const double world_dimensions[3], const double resolution)
  : voxel_grid::VoxelGrid<int>(origin, world_dimensions, resolution)
  , goal_ptr_(nullptr)
  , distance_grid_ptr_(nullptr)
  , occupancy_grid_ptr_(nullptr)
{
  Initialize();
}

void CostGrid::Initialize()
{
  priority_queue_ = std::priority_queue<PriorityNode, std::vector<PriorityNode>, PriorityNodeCompareFunctor>();
  int num_cells = GetNumCells();
  Node node;
  nodes_.clear();
  for (int i = 0; i < num_cells; i++)
  {
    node.index = i;
    nodes_.push_back(node);
  }
  UpdateEdgeGridCoords();
  UpdateAllNeighbors();
  SetCostWeights(1, 1, 1, 1);
  SetInflationDistance(0);
}

int CostGrid::GetLineCost(const double xyz1[3], const double xyz2[3]) const
{
  int start[3], end[3];
  WorldToGrid(xyz1, start);
  WorldToGrid(xyz2, end);

  if (!IsInMap(start) || !IsInMap(end))
  {
    return MAX_COST;
  }

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

  int accumulated_cost = 0;

  if ((l >= m) && (l >= n))
  {
    err_1 = dy2 - l;
    err_2 = dz2 - l;
    for (i = 0; i <= l; i++)
    {
      if (occupancy_grid_ptr_ && occupancy_grid_ptr_->IsOccupied(voxel))
      {
        accumulated_cost += MAX_COST;
      }
      else
      {
        accumulated_cost += ReadValue(voxel);
      }
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
    for (i = 0; i <= m; i++)
    {
      if (occupancy_grid_ptr_ && occupancy_grid_ptr_->IsOccupied(voxel))
      {
        accumulated_cost += MAX_COST;
      }
      else
      {
        accumulated_cost += ReadValue(voxel);
      }
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
    for (i = 0; i <= n; i++)
    {
      if (occupancy_grid_ptr_ && occupancy_grid_ptr_->IsOccupied(voxel))
      {
        accumulated_cost += MAX_COST;
      }
      else
      {
        accumulated_cost += ReadValue(voxel);
      }
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

  return accumulated_cost;
}

void CostGrid::SetDistanceGrid(distance_grid::DistanceGrid* dt)
{
  distance_grid_ptr_ = dt;
}

void CostGrid::SetOccupancyGrid(occupancy_grid::OccupancyGrid* og)
{
  occupancy_grid_ptr_ = og;
}

void CostGrid::UpdateDijkstra()
{
  while (!priority_queue_.empty())
  {
    auto node_ptr = Pop();
    if (!node_ptr->visited)
    {
      for (auto neighbor : node_ptr->neighbors)
      {
        int candidate_cost = ReadValue(node_ptr->index) + GetEdgeCost(node_ptr, neighbor);
        if (candidate_cost < ReadValue(neighbor->index) && !neighbor->visited)
        {
          neighbor->parent = node_ptr;
          WriteValue(neighbor->index, candidate_cost);
          Insert(candidate_cost, neighbor);
        }
      }
      node_ptr->visited = true;
    }
  }
}

void CostGrid::UpdateDijkstra(const double goal[3])
{
  ResetAllNodes();
  SetGoal(goal);
  Insert(0, goal_ptr_);
  UpdateDijkstra();
}

void CostGrid::GetPaths(double start_xyz[3], std::vector<std::array<double, 3>>* dense_path,
                        std::vector<std::array<double, 3>>* sparse_path)
{
  int ind = WorldToIndex(start_xyz);
  Node* current_node = nullptr;
  if (IsInMap(ind))
  {
    current_node = &nodes_[ind];
  }
  else
  {
    return;
  }
  double node_xyz[3];

  do
  {
    IndexToWorld(current_node->index, node_xyz);
    dense_path->push_back(std::array<double, 3>({ node_xyz[0], node_xyz[1], node_xyz[2] }));
    current_node = current_node->parent;
  } while (current_node != nullptr && ReadValue(current_node->index) != 0);

  SparsifyPath(*dense_path, sparse_path);
}

void CostGrid::GetPaths(double x, double y, double z, std::vector<std::array<double, 3>>* dense_path,
                        std::vector<std::array<double, 3>>* sparse_path)
{
  double start_xyz[3] = { x, y, z };
  GetPaths(start_xyz, dense_path, sparse_path);
}

int CostGrid::SparsifyPath(const std::vector<std::array<double, 3>>& dense_path,
                           std::vector<std::array<double, 3>>* sparse_path) const
{
  int num_points;
  // Empty the sparse_path just in case
  sparse_path->clear();
  // Iterate through the dense plan
  for (auto itr = dense_path.begin(); itr != dense_path.end(); ++itr)
  {
    auto dense_goal = *itr;
    // Check if the plan vector is long enough to index into
    if (sparse_path->size() >= 2)
    {
      if (GetLineCost(sparse_path->at(sparse_path->size() - 2).data(), sparse_path->back().data()) +
              GetLineCost(sparse_path->back().data(), dense_goal.data()) <
          (GetLineCost(sparse_path->at(sparse_path->size() - 2).data(), dense_goal.data())))
      {
        sparse_path->push_back(dense_goal);
        num_points++;
      }
      else
      {
        sparse_path->at(sparse_path->size() - 1) = dense_goal;
      }
    }
    else
    {
      sparse_path->push_back(dense_goal);
    }
  }
  return num_points;
}

void CostGrid::PreShiftOrigin(const std::vector<int>& slice_indexes)
{
  if (slice_indexes.size() >= GetNumCells())
  {
    return;
  }
  for (int index : slice_indexes)
  {
    // mark slice nodes
    Node& node = nodes_[index];
    neighbor_set_.insert(&node);

    // and all of their neighbors
    for (auto neighbor : node.neighbors)
    {
      neighbor_set_.insert(neighbor);
    }
  }
}

void CostGrid::PostShiftOrigin(const std::vector<int>& slice_indexes)
{
  for (int index : slice_indexes)
  {
    Node& node = nodes_[index];
    // mark new neighbors after shift
    for (auto neighbor : GetNeighbors(&node))
    {
      neighbor_set_.insert(neighbor);
    }
    ResetNode(&node);
  }

  // update neighbors
  for (Node* node : neighbor_set_)
  {
    UpdateNeighbors(node);
  }

  neighbor_set_.clear();

  if (goal_ptr_)
  {
    IndexToGrid(goal_ptr_->index, goal_ixyz_);
  }
}

void CostGrid::ClearNeighbors()
{
  int num_cells = GetNumCells();
  for (int i = 0; i < num_cells; i++)
  {
    nodes_[i].neighbors.clear();
  }
}

void CostGrid::ResetNode(Node* node)
{
  node->visited = false;
  WriteValue(node->index, MAX_COST);
}

void CostGrid::ResetAllNodes()
{
  int num_cells = GetNumCells();
  for (int i = 0; i < num_cells; i++)
  {
    ResetNode(&nodes_[i]);
  }
}

CostGrid::Node* CostGrid::GetClosestEdgeNodeToGoal()
{
  int min_distance = std::numeric_limits<int>::max();
  Node* closest_node = nullptr;
  for (const auto& grid_coord : edge_grid_coords_)
  {
    int ixyz[3] = { grid_coord[0], grid_coord[1], grid_coord[2] };
    int edge_index = GridToIndex(ixyz);
    Node& edge_node = nodes_[edge_index];
    int distance = GetManhattanDistanceToGoal(&edge_node);
    if (distance < min_distance)
    {
      min_distance = distance;
      closest_node = &edge_node;
    }
  }

  return closest_node;
}

void CostGrid::SetGoal(const double goal[3])
{
  WorldToVoxels(goal, goal_voxels_);
  bool goal_in_map = IsInMap(goal);
  if (!goal_in_map)
  {
    goal_ptr_ = GetClosestEdgeNodeToGoal();
  }
  else
  {
    int goal_index = WorldToIndex(goal);
    goal_ptr_ = &nodes_[goal_index];
  }
  goal_ptr_->parent = nullptr;
  goal_ptr_->visited = false;
  WriteValue(goal_ptr_->index, 0);
  IndexToWorld(goal_ptr_->index, goal_xyz_);
}

bool CostGrid::GetProjectedGoal(double xyz[3]) const
{
  bool have_goal = goal_ptr_ != nullptr;
  if (have_goal)
  {
    memcpy(xyz, goal_xyz_, sizeof(double) * 3);
  }
  return have_goal;
}

void CostGrid::UpdateAllNeighbors()
{
  int num_cells = GetNumCells();
  for (int i = 0; i < num_cells; i++)
  {
    UpdateNeighbors(&nodes_[i]);
  }
}

int CostGrid::GetEdgeCost(const Node* node1_ptr, const Node* node2_ptr) const
{
  // obstacle cost
  int obstacle_cost = 0;
  int inflation_cost = 0;
  if (distance_grid_ptr_)
  {
    static int dmax = distance_grid_ptr_->GetMaxSquaredDistance();

    int squared_distance = distance_grid_ptr_->ReadValue(node2_ptr->index);
    inflation_cost = dmax - squared_distance;
    inflation_cost *= inflation_cost;

    if (squared_distance <= inflation_distance_)
    {
      obstacle_cost = 1;
    }
  }

  // unknown cost
  int unknown_cost = 0;
  if (occupancy_grid_ptr_)
  {
    bool unknown_space = occupancy_grid_ptr_->ReadValue(node2_ptr->index) < 0;
    if (unknown_space)
    {
      unknown_cost = 1;
    }
  }

  int ixyz2[3];
  IndexToGrid(node2_ptr->index, ixyz2);
  int altitude_cost = std::abs(ixyz2[2] - goal_ixyz_[2]);

  return 1 + altitude_cost * altitude_weight_ + inflation_cost * inflation_weight_ + unknown_cost * unknown_weight_ +
         obstacle_cost * obstacle_weight_;
}

void CostGrid::SetCostWeights(int altitude_weight, int inflation_weight, int unknown_weight, int obstacle_weight)
{
  altitude_weight_ = altitude_weight;
  inflation_weight_ = inflation_weight;
  unknown_weight_ = unknown_weight;
  obstacle_weight_ = obstacle_weight;
}

int CostGrid::GetManhattanDistanceToGoal(const Node* node) const
{
  int voxel_xyz[3];

  double xyz[3];
  IndexToWorld(node->index, xyz);
  WorldToVoxels(xyz, voxel_xyz);

  int x_dist = std::abs(voxel_xyz[0] - goal_voxels_[0]);
  int y_dist = std::abs(voxel_xyz[1] - goal_voxels_[1]);
  int z_dist = std::abs(voxel_xyz[2] - goal_voxels_[2]);
  int dist = x_dist + y_dist + z_dist;
  return dist;
}

void CostGrid::PrintGraph()
{
  int dimensions[3];
  GetGridDimensions(dimensions);
  for (int y = dimensions[1] - 1; y >= 0; y--)
  {
    for (int x = 0; x < dimensions[0]; x++)
    {
      int ixyz[3] = { x, y, 0 };
      int cost = ReadValue(GridToIndex(ixyz));
      cost != MAX_COST ? printf("%i ", cost) : printf(" -  ");
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

void CostGrid::PrintNeighbors()
{
  int dimensions[3];
  GetGridDimensions(dimensions);
  for (int y = dimensions[1] - 1; y >= 0; y--)
  {
    for (int x = 0; x < dimensions[0]; x++)
    {
      int ixyz[3] = { x, y, 0 };
      Node& node = nodes_[GridToIndex(ixyz)];
      int cost = ReadValue(node.index);
      cost != MAX_COST ? printf("%lu ", node.neighbors.size()) : printf(" -  ");
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
}

std::vector<CostGrid::Node*> CostGrid::GetNeighbors(Node* node)
{
  std::vector<Node*> neighbors;
  int ixyz[3];
  IndexToGrid(node->index, ixyz);
  for (int dimension = 0; dimension < 3; dimension++)
  {
    int neighbor_ixyz[3] = { ixyz[0], ixyz[1], ixyz[2] };
    for (int delta : { -1, 1 })
    {
      neighbor_ixyz[dimension] = ixyz[dimension] + delta;
      if (IsInMap(neighbor_ixyz))
      {
        int neighbor_index = GridToIndex(neighbor_ixyz);
        neighbors.push_back(&nodes_[neighbor_index]);
      }
    }
  }

  return neighbors;
}

void CostGrid::UpdateEdgeGridCoords()
{
  std::set<int> edge_indexes;
  int num_cells = GetNumCells();
  for (int i = 0; i < num_cells; i++)
  {
    Node& node = nodes_[i];
    std::vector<Node*> neighbors = GetNeighbors(&node);
    if (neighbors.size() < 6)
    {
      edge_indexes.insert(node.index);
    }
  }

  edge_grid_coords_.clear();
  for (int edge_index : edge_indexes)
  {
    int ixyz[3];
    IndexToGrid(edge_index, ixyz);
    edge_grid_coords_.push_back(std::array<int, 3>({ ixyz[0], ixyz[1], ixyz[2] }));
  }
}

void CostGrid::UpdateNeighbors(Node* node)
{
  node->neighbors.clear();
  node->neighbors = GetNeighbors(node);
}

void CostGrid::Insert(int priority, Node* node)
{
  priority_queue_.push(std::make_pair(priority, node));
}

CostGrid::Node* CostGrid::Pop()
{
  PriorityNode priority_node = priority_queue_.top();
  priority_queue_.pop();
  return priority_node.second;
}

}  // namespace cost_grid
