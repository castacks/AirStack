#ifndef BELIEFMAP_H
#define BELIEFMAP_H

#include <tuple>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <float.h>


class BeliefMap
{
private:

public:

  std::vector<std::vector<double>> polygon_bounds;

  // std::vector<planner_map_interfaces::GridPrior> prior_list;  // list of priors

  // map use grid map toolbox

  // map of structs to keep track of various traits at each grid point
  // std::vector<std::vector<double>> priority;
  // std::vector<std::vector<int>> sensor_model_id;

  BeliefMap();

};

#endif // BELIEFMAP_H