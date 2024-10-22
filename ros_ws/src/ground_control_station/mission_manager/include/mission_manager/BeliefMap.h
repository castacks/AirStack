#ifndef BELIEFMAP_H
#define BELIEFMAP_H

#include <tuple>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <float.h>
#include <cstdint>

#include <grid_map_ros/grid_map_ros.hpp>
#include <geometry_msgs/msg/polygon.hpp>

#include "airstack_msgs/msg/search_mission_request.hpp"
#include "airstack_msgs/msg/search_prior.hpp"
#include "airstack_msgs/msg/keep_out_zone.hpp"
#include "airstack_msgs/msg/belief_map_data.hpp"

class BeliefMap
{
public:

  std::vector<std::vector<double>> polygon_bounds;

  // std::vector<planner_map_interfaces::GridPrior> prior_list;  // list of priors

  // map of structs to keep track of various traits at each grid point
  // std::vector<std::vector<double>> priority;
  // std::vector<std::vector<int>> sensor_model_id;

  BeliefMap();
  bool reset_map(rclcpp::Logger logger, airstack_msgs::msg::SearchMissionRequest search_mission_request, double grid_cell_size);
  bool update_map(rclcpp::Logger logger, const airstack_msgs::msg::BeliefMapData::SharedPtr new_belief_data);
  grid_map::GridMap map_;

  bool is_initialized() const
  {
    return !map_.getSize()(0) == 0;
  }

  double get_min_x() const
  {
    return min_x;
  }
  double get_max_x() const
  {
    return max_x;
  }
  double get_min_y() const
  {
    return min_y;
  }
  double get_max_y() const
  {
    return max_y;
  }

private:
  //search map allocation
  double min_x = DBL_MAX;
  double max_x = -DBL_MAX;
  double min_y = DBL_MAX;
  double max_y = -DBL_MAX;
  double grid_cell_size_;
  
  
};

#endif // BELIEFMAP_H