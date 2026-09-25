// =============================================================================
//  mtl/planning/agent_sortie.hpp
//
//  ONE AGENT'S BUDGETED SORTIE, END TO END, BUDGET GUARANTEED.
//
//  THE THREE THINGS THAT MAKE THIS MORE THAN "CUT THE TSP SHORT"
//  ------------------------------------------------------------
//   1. Subset and order are chosen together (planBudgetedMacroRoute -> the
//      orienteering solver), so the route is the best subset of clusters, not
//      the first however-many clusters of a route planned for all of them.
//   2. Leftover budget is then spent at CELL granularity (refineWithCellAnchors),
//      because clusters are chunky and the last few hundred metres of budget will
//      not buy a whole one.  That pass is gimbal-aware: the aircraft only has to
//      come within maxClusterRadius of a cell, and once there the gimbal sweeps
//      every leftover cell inside that radius, so an anchor is priced as a
//      one-off cluster.
//   3. The budget is checked against the TRAJECTORY THAT COMES OUT, not the plan
//      that went in.  The Dubins arcs are inside the planner's own re-costing
//      loop; the lateral-coverage extension is not (generateTrajectories decides
//      it from the cell layout), so it is charged to an adaptive RESERVE: plan,
//      generate, measure, and if the flown arc overran, raise the reserve and
//      re-plan.  Two or three passes is normal.  The returned plan HAS BEEN
//      MEASURED.
//
//  Feasibility is monotone in the reserve - a bigger hold-back means a shorter
//  route - so the reserve is found by BISECTION for the smallest value that
//  still comes out feasible.  It cannot be solved by substituting the measured
//  extension cost, because that cost is DISCONTINUOUS in the reserve: shorten
//  the route enough and the extension is not needed at all, the measurement
//  reads zero, the reserve is dropped, the longer route brings the extension
//  back, and the loop oscillates between two states forever.
// =============================================================================
#ifndef MTL_PLANNING_AGENT_SORTIE_HPP
#define MTL_PLANNING_AGENT_SORTIE_HPP

#include <vector>

#include "mtl/params.hpp"
#include "mtl/types.hpp"

namespace mtl::planning {

/// Everything one agent's sortie is planned against.  `clusters` and `cells`
/// are the GLOBAL lists; `candClusters` names this agent's share of them.
struct SortieContext {
    const ClusterSet* clusters = nullptr;
    const CellSet*    cells    = nullptr;
    const PlannerParams* params = nullptr;
};

struct AgentSpec {
    Vec2               startPos = Vec2::Zero();
    std::vector<Index> candClusters;  ///< global cluster ids this agent may service
    int                id = 0;
    double             budgetDist = kInf;
};

/// Plan one agent's sortie.  Never throws on an empty candidate list: a grounded
/// agent comes back as a one-sample plan the rest of the pipeline handles.
AgentPlan planAgentSortie(const AgentSpec& agent, const SortieContext& ctx);

}  // namespace mtl::planning

#endif  // MTL_PLANNING_AGENT_SORTIE_HPP
