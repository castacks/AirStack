// =============================================================================
//  planners.cpp — ipp_planners_node.h init_planner: the planner named by `planner`.
// =============================================================================
#include "ipp_baselines_search_planner/greedy.hpp"
#include "ipp_baselines_search_planner/mcts_search.hpp"
#include "ipp_baselines_search_planner/rect_coverage.hpp"

#include <stdexcept>

namespace ipp_baselines_search {

std::unique_ptr<SearchPlanner> makePlanner(const PlanningGrid& grid, const PlannerSetup& setup,
                                           const BaselineParams& params) {
    validateParams(params);
    if (params.planner == "greedy") return std::make_unique<GreedyPlanner>(grid, setup, params);
    if (params.planner == "rect_coverage") return std::make_unique<RectCoveragePlanner>(grid, setup, params);
    if (params.planner == "mcts_search") return std::make_unique<MctsSearchPlanner>(grid, setup, params);
    throw std::invalid_argument("Invalid planner name: " + params.planner);
}

}  // namespace ipp_baselines_search
