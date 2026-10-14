// =============================================================================
//  ipp_baselines_search_planner/params.hpp — one key=value setter shared by the CLI
//  (--set) and the ROS 2 node (parameter names are the same).
// =============================================================================
#ifndef IPP_BASELINES_SEARCH_PLANNER_PARAMS_HPP
#define IPP_BASELINES_SEARCH_PLANNER_PARAMS_HPP

#include <string>
#include <vector>

#include "ipp_baselines_search_planner/receding.hpp"

namespace ipp_baselines_search {

/// Applies one `key=value`. Returns false for an unknown key. camSet is set when the camera
/// FOV / tilt is overridden.
bool applyParam(const std::string& key, const std::string& value, BaselineParams& bp, HorizonParams& hp,
                Camera& cam, bool& camSet);
/// Every key applyParam knows (the planner yaml keys the CLI accepts).
const std::vector<std::string>& paramKeys();

}  // namespace ipp_baselines_search

#endif  // IPP_BASELINES_SEARCH_PLANNER_PARAMS_HPP
