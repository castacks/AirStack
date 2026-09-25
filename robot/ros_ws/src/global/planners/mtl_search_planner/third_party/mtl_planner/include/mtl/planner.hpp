// =============================================================================
//  mtl/planner.hpp
//
//  THE INTEGRATION SURFACE.
//
//  One object, constructed once with the whole parameter set, then called as
//  often as the host simulation likes:
//
//      mtl::PlannerParams params;              // every tunable, with defaults
//      params.numAgents = 4;
//      params.maxFlightTime = 400.0;
//      mtl::Planner planner(params);           // validates and finalises
//
//      const auto result = planner.plan(belief, starts);
//      for (const auto& t : result.trajectories) { /* fly t.drone / t.rpy */ }
//
//  The constructor houses every tunable (see mtl/params.hpp) and does the
//  validation once, so nothing downstream has to re-check a parameter or invent
//  a default.  Two entry points are offered:
//
//    plan(belief, starts)       from a prior belief GRID: the planner extracts
//                               its own cells, clusters them and plans.
//    planFromCells(cells, ...)  from cells the host already has - drop-in for a
//                               simulation with its own mapping stack, where the
//                               belief grid lives somewhere else entirely.
//
//  WHAT THE PLANNER OWNS AND WHAT IT DOES NOT
//  ------------------------------------------
//  It owns everything from "here is the belief" to "here are the flight-ready
//  trajectories and attitudes": cell extraction, macro-clustering, the team
//  partition, the budgeted route, the leftover-budget refinement, the sensor
//  sweep order, the Dubins trajectory, the lateral-coverage extension and the
//  gimbal schedule.
//
//  It does NOT own scenario generation (mtl/mapgen) or scoring (mtl/eval).
//  Those are separate libraries precisely so a host simulation can supply its
//  own belief and its own sensor model without linking the ones here.
// =============================================================================
#ifndef MTL_PLANNER_HPP
#define MTL_PLANNER_HPP

#include <memory>
#include <vector>

#include "mtl/params.hpp"
#include "mtl/types.hpp"

namespace mtl {

class Planner {
public:
    /// @throws std::invalid_argument on a parameter set that cannot produce a
    ///         trajectory (see PlannerParams::validate).
    explicit Planner(PlannerParams params);
    ~Planner();

    Planner(const Planner&)            = delete;
    Planner& operator=(const Planner&) = delete;
    Planner(Planner&&) noexcept;
    Planner& operator=(Planner&&) noexcept;

    /// Plan the whole team from a prior belief grid.
    /// @param agentStarts one launch point per agent.  Extra rows are ignored;
    ///                    too few is an error.
    PlanningResult plan(const BeliefField& belief, const std::vector<Vec2>& agentStarts);

    /// Plan from cells the host already extracted (its own mapping stack, a
    /// previous run's cells, a hand-built target list).  `cells.mass` is the
    /// currency the budgeted planner maximises; if the host has no belief mass,
    /// set every entry to 1 and the planner maximises cell COUNT instead.
    PlanningResult planFromCells(const CellSet& cells, const std::vector<Vec2>& agentStarts);

    /// Plan from cells AND a pre-built macro-cluster abstraction, for a host
    /// that wants to control the clustering itself.  `clusters.reward` and
    /// `clusters.cellIdx` are filled here if they are empty.
    PlanningResult planFromClusters(const CellSet& cells, ClusterSet clusters,
                                    const std::vector<Vec2>& agentStarts);

    const PlannerParams& params() const noexcept;

    /// Retune between calls.  The set is re-finalised and re-validated, so the
    /// derived quantities stay consistent.
    void setParams(PlannerParams params);

    /// Per-agent distance budgets [m] the current parameters imply.
    std::vector<double> agentBudgets() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace mtl

#endif  // MTL_PLANNER_HPP
