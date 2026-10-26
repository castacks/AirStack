// =============================================================================
//  mtl_curve/curve_planning/team_curves.hpp
//
//  The team solve: initial curves, service audit, slack, reallocation - the
//  port of reallocateCurveClusters.m (the plan's PHASE 5).
//
//  1. INITIAL SOLVE.  Each agent seeds a curve - one seed per entry of
//     team.initStrategies: 'sector' (default: the coverage orienteering over a
//     sector grid, planning::solveSectorOrienteering, on the belief the agents
//     already planned leave unswept), 'clusters' (the plan: its partition of
//     the macro-clusters ordered by solveBudgetedOrienteering on a Euclidean
//     budget of orderBudgetFrac * L), 'greedy', 'gcs' - and optimises each
//     AGAINST THE TEAM (the other agents' log-miss frozen: Gauss-Seidel),
//     keeping the best seed.  team.coordinationSweeps passes (default 1: one
//     optimisation per seed); the later ones re-optimise warm.
//  2. SERVICE AUDIT.  Per cluster, the residual fraction of its prior mass the
//     team leaves (fast grid); UNSERVICED when above unservicedResidualFrac.
//     The plan's centroid-in-swath test is reported alongside.
//  3. SLACK.  y_j = -ds_j sum_x W(x) k_j(x) - the plan's dJ/dL made local.
//     slack_a = share of agent a's length with yield density below
//     slackYieldFrac of the team mean; an agent is a slack agent if slack_a >
//     0.02 or its own clusters are swept to slackMarginThreshold.
//  4. REALLOCATION ROUNDS (team.reallocate, default off).  Pooled clusters, most residual mass first, are
//     offered to the nearest slack agents.  The candidate re-seeds with the
//     cluster claimed ('path' (default): spliced into its current OPTIMISED
//     curve, budget repaired on the lowest-yield arc, prefix kept exactly -
//     see path_insert.hpp; 'insert': cheapest insertion into its cluster
//     waypoints and a fresh seed, the plan; 'direct': fly to it first),
//     re-optimises at the SAME length, and
//     the move is committed only if the team objective falls by more than
//     reallocEps and the curve stays feasible.  Each accepting round ends with
//     a warm coordination sweep.
// =============================================================================
#ifndef MTLC_CURVE_PLANNING_TEAM_CURVES_HPP
#define MTLC_CURVE_PLANNING_TEAM_CURVES_HPP

#include <optional>
#include <vector>

#include "mtl_curve/params.hpp"
#include "mtl_curve/types.hpp"

namespace mtl::curve::curve_planning {

struct TeamConfig {
    std::vector<Vec2>                starts;
    std::vector<double>              L;          ///< per-agent curve length [m]
    std::vector<EndpointMode>        modes;
    std::vector<std::optional<Vec2>> dests;
    const FastGrid*                  G  = nullptr;  ///< fast grid (with stencil)
    const FastGrid*                  Gx = nullptr;  ///< exploration grid, or nullptr
    std::vector<const SwathKernelSet*> kernels;     ///< per agent (per altitude)
    Path2            centroids;
    VecX             rewards;
    std::vector<int> assign;                      ///< initial agent of each cluster
    Eigen::MatrixXi  clusterOfPixel;              ///< ny-by-nx cluster of each fast pixel, -1 = none
    VecX             clusterRadius;               ///< K-by-1 max cell-to-centroid distance (GcsSetRadius::Capture)
    /// [xmin xmax ymin ymax] the GCS sets are clipped to (unset = the fast grid's extent)
    std::optional<Eigen::Vector4d> bounds;
};

struct TeamSolution {
    std::vector<VecX>     v, vStatic;
    std::vector<CurveRep> rep, repStatic;
    std::vector<MatX>     lam;                    ///< per-agent deposit on G
    std::vector<std::vector<Index>> wp;           ///< waypoint clusters, in order
    std::vector<std::string> initStrategy;
    std::vector<OptimizeInfo> lastOpt;
    std::vector<Path2>    seedPath;
    TeamInfo info;
};

/// seedOnly: build each agent's cluster-dependent seeds ('clusters', 'gcs';
/// 'greedy' only if nothing else is listed) WITHOUT optimising them, keep the
/// best by its fast objective, and return after the first sweep with
/// info.Jstatic = the team objective of the seeds.  This is how the
/// information-aware cluster search scores an abstraction cheaply.
TeamSolution reallocateCurveClusters(const TeamConfig& cfg, const PlannerParams& params, bool seedOnly = false);

/// Coarse pixel -> the targetCellSize block (valid cell) containing it -> its
/// cluster (-1 where no valid cell).
Eigen::MatrixXi clusterOfPixel(const FastGrid& G, const Path2& cellCenters,
                               const std::vector<int>& cellCluster, double targetCellSize);

}  // namespace mtl::curve::curve_planning

#endif  // MTLC_CURVE_PLANNING_TEAM_CURVES_HPP
