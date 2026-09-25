// =============================================================================
//  mtl/planning/team_allocation.hpp
//
//  BUDGETED SORTIES FOR THE WHOLE TEAM, WITH LEFTOVERS RE-OFFERED.
//
//  Partitioning the macro-clusters geographically with k-means and giving each
//  agent its share is fine when range is infinite - every agent eventually
//  services everything it owns.  Under a budget it is not: k-means balances
//  CLUSTER COUNT and geography, not information per metre, so one agent
//  routinely ends up unable to reach its last few clusters while another lands
//  with budget to spare.
//
//  So after the first pass this runs a REALLOCATION: the clusters nobody reached
//  go into a pool, and each agent - richest slack first - is re-planned against
//  its own selection PLUS the pool.  An agent takes from the pool only if its
//  re-plan is MEASURED feasible and collects strictly more information than
//  before; anything it releases goes back to the pool for the next agent.  That
//  turns the k-means partition from a hard constraint into a starting point,
//  which is the team-orienteering view of the problem.
//
//  Set BudgetParams::reallocate = false to keep the strict partition.  With an
//  infinite budget the reallocation is skipped entirely (nothing is ever left
//  over) and each agent gets exactly its k-means share.
// =============================================================================
#ifndef MTL_PLANNING_TEAM_ALLOCATION_HPP
#define MTL_PLANNING_TEAM_ALLOCATION_HPP

#include <vector>

#include "mtl/planning/agent_sortie.hpp"
#include "mtl/types.hpp"

namespace mtl::planning {

struct TeamResult {
    std::vector<AgentPlan> plans;
    TeamInfo               info;
};

/// @param ctx              global clusters, cells and parameters
/// @param agentStarts      one launch point per agent
/// @param agentOfCluster   K-by-1 agent index per cluster (the k-means partition)
/// @param budgets          per-agent distance budget [m]
TeamResult allocateBudgetedTeam(const SortieContext& ctx, const std::vector<Vec2>& agentStarts,
                                const std::vector<int>& agentOfCluster,
                                const std::vector<double>& budgets);

}  // namespace mtl::planning

#endif  // MTL_PLANNING_TEAM_ALLOCATION_HPP
