// =============================================================================
//  mtl_curve/planner.hpp
//
//  THE INTEGRATION SURFACE - the same shape as cpp_planner's mtl::Planner.
//
//      mtl::curve::PlannerParams params;          // every tunable, with defaults
//      params.numAgents = 4;
//      mtl::curve::Planner planner(params);       // validates and finalises
//
//      const auto result = planner.plan(belief, starts);
//      for (const auto& t : result.trajectories) { /* fly t.drone / t.rpy, aim t.sensor */ }
//
//  One continuous flight curve per aircraft, of EXACTLY the endurance budget
//  and never tighter than minTurnRadius, optimised jointly with the others to
//  minimise the team's residual belief; the single-axis gimbal sweeps its
//  cross-track line along it.  Replaces cpp_planner's cells -> clusters ->
//  budgeted orienteering -> Dubins -> gimbal schedule -> run-out chain.
//
//  Entry points (as in mtl::Planner):
//    plan(belief, starts)          from a prior belief GRID - the curve planner
//                                  optimises against the prior itself.
//    planFromCells(cells, starts)  from cells the host already extracted: the
//                                  prior the objective sees is each cell's mass
//                                  spread over its targetCellSize block (the
//                                  mtl.scenario/1 interchange carries cells).
//    planFromClusters(cells, clusters, starts[, belief])
//                                  also control the macro-clustering (only used
//                                  to seed and audit the curves).
//
//  WHAT IT DOES NOT OWN: scenario generation (mtl_curve/mapgen) and scoring
//  (mtl_curve/eval) - separate libraries, as in cpp_planner.
// =============================================================================
#ifndef MTLC_PLANNER_HPP
#define MTLC_PLANNER_HPP

#include <memory>
#include <vector>

#include "mtl_curve/params.hpp"
#include "mtl_curve/types.hpp"

namespace mtl::curve {

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
    /// @param agentStarts one launch point per agent; extra rows are ignored.
    PlanningResult plan(const BeliefField& belief, const std::vector<Vec2>& agentStarts);

    /// Plan from cells the host already extracted.  cells.mass is the prior
    /// the objective sees (spread over each cell's block).
    PlanningResult planFromCells(const CellSet& cells, const std::vector<Vec2>& agentStarts);

    /// Plan from cells AND a pre-built macro-cluster abstraction.  With
    /// `belief` the objective uses the grid; without it, the rasterised cells.
    PlanningResult planFromClusters(const CellSet& cells, ClusterSet clusters,
                                    const std::vector<Vec2>& agentStarts,
                                    const BeliefField* belief = nullptr);

    const PlannerParams& params() const noexcept;
    void setParams(PlannerParams params);

    /// Per-agent distance budgets [m] (= curve lengths) and cruise altitudes [m].
    std::vector<double> agentBudgets() const;
    std::vector<double> agentAltitudes() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace mtl::curve

#endif  // MTLC_PLANNER_HPP
