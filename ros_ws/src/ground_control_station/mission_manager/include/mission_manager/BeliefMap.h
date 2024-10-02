#ifndef BELIEFMAP_H
#define BELIEFMAP_H

#include <tuple>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <float.h>

#include <grid_map_ros/grid_map_ros.hpp>
#include <geometry_msgs/msg/polygon.hpp>

#include "airstack_msgs/msg/search_mission_request.hpp"
#include "airstack_msgs/msg/search_prior.hpp"
#include "airstack_msgs/msg/keep_out_zone.hpp"

class BeliefMap
{
public:

  std::vector<std::vector<double>> polygon_bounds;

  // std::vector<planner_map_interfaces::GridPrior> prior_list;  // list of priors

  // map of structs to keep track of various traits at each grid point
  // std::vector<std::vector<double>> priority;
  // std::vector<std::vector<int>> sensor_model_id;

  BeliefMap();
  bool reset_map(rclcpp::Logger logger, airstack_msgs::msg::SearchMissionRequest search_mission_request);
  grid_map::GridMap map_;

  bool is_initialized() const
  {
    return !map_.getSize()(0) == 0;
  }

private:
  
};

#endif // BELIEFMAP_H