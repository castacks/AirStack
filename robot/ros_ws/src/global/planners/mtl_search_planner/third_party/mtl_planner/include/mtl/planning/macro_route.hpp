// =============================================================================
//  mtl/planning/macro_route.hpp
//
//  Budgeted cluster route, costed on the FLOWN arc.
//
//  WHAT THIS ADDS OVER THE BARE ORIENTEERING SOLVER
//  ------------------------------------------------
//  solveBudgetedOrienteering reasons in straight lines, because 2-opt and
//  cheapest insertion need the triangle inequality.  The aircraft does not fly
//  straight lines: it flies Dubins arcs with a minTurnRadius floor, and the
//  ratio between the two is route-dependent - a tight zig-zag through nearby
//  clusters can be 30-40% longer once the turns are drawn, while a long straight
//  leg costs nothing extra.  Budgeting on the Euclidean tour would therefore
//  silently overspend, sometimes badly.
//
//  So the budget is enforced on the REAL number:
//    1. solve the orienteering problem against an effective budget s*B (s = 1);
//    2. draw the Dubins path through the chosen route and MEASURE its arc L
//       (a caller-supplied lengthFcn can measure something even more complete);
//    3. if L > B, shrink s toward B/L; if L <= B, remember this solution and push
//       s UP to see whether one more cluster fits.  Bisect between the best known
//       feasible scale and the smallest known infeasible one.
//
//  The loop returns the highest-reward solution that was actually MEASURED
//  feasible, so the budget is a hard guarantee at this stage rather than an
//  estimate.  It typically converges in 3-5 iterations.
//
//  An infinite budget short-circuits the whole thing and returns exactly what
//  the plain TSP heuristic returns, so the unbudgeted pipeline is unchanged.
// =============================================================================
#ifndef MTL_PLANNING_MACRO_ROUTE_HPP
#define MTL_PLANNING_MACRO_ROUTE_HPP

#include <functional>
#include <vector>

#include "mtl/params.hpp"
#include "mtl/types.hpp"

namespace mtl::planning {

/// Measures what a route actually costs to fly.  Defaults to the Dubins arc;
/// planAgentSortie can pass a closure that also charges the coverage extension.
using LengthFcn = std::function<double(const Path2&)>;

struct MacroRouteSolution {
    Path2              routeXY;     ///< (1+n)-by-2 [startPos; chosen centroids in order]
    std::vector<Index> visitOrder;  ///< indices into `centroids`, in visit order
    MacroRouteInfo     info;
};

/// @param centroids     K-by-2 macro-cluster centroids (this agent's share)
/// @param rewards       K-by-1 information mass per cluster
/// @param startPos      launch point
/// @param budgetDist    max flown distance [m], or Inf
/// @param minTurnRadius Dubins turn radius [m]
MacroRouteSolution planBudgetedMacroRoute(const Path2& centroids, const VecX& rewards,
                                          const Vec2& startPos, double budgetDist,
                                          double minTurnRadius, const MacroRouteParams& opts,
                                          const OrienteeringParams& orienteeringOpts,
                                          const DubinsParams& dubinsOpts,
                                          const LengthFcn& lengthFcn = nullptr);

}  // namespace mtl::planning

#endif  // MTL_PLANNING_MACRO_ROUTE_HPP
