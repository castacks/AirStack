// =============================================================================
//  mtl_curve/planning/sector_orienteering.hpp
//
//  THE 'sector' SEED: COVERAGE ORIENTEERING OVER A SECTOR GRID.
//
//  The 'clusters' seed orders macro-clusters whose prizes are fixed and
//  additive.  That over-counts: two clusters a swath apart are both credited in
//  full although one pass sweeps both, and a cluster's prize ignores the belief
//  next to it that the same pass collects.  The sector seed instead
//
//    1. lays a sectorSize grid over the whole search area (every sector is a
//       candidate node, not only the cells above minimumBeliefMass),
//    2. credits a visited sector with the belief inside a box of half-side
//       boxFrac x the calibrated swath half-width around its centre (the area
//       one pass through it sweeps, plus a margin the optimiser can slide the
//       swath over),
//    3. scores a route by the belief in the UNION of its boxes, so every
//       insertion is valued by its MARGINAL gain: selecting a sector lowers the
//       value of its neighbours, and
//    4. solves that coverage orienteering problem with the same heuristic
//       family as solveBudgetedOrienteering - GRASP multi-start insertion (gain
//       / detour, gain^2 / detour, randomised restricted list), 2-opt and
//       Or-opt shortening, drop-and-refill replacement, ruin and recreate -
//       under a Euclidean budget of budgetFrac x the curve length.
//
//  The ordered sector centres are then flown by initParametricSpline exactly
//  like the cluster centroids.  Measured against the clusters + greedy
//  multi-start (58 + 60 single-agent maps, same optimiser): about -3% mean
//  residual for one optimisation instead of two (see sector_codebase/ and
//  "Claude outputs/seed_basin_report/sector_op/").
//
//  The weights may be any non-negative raster: the planner passes the prior
//  times the miss probability of the agents already planned, so a team spreads
//  out.  The random stream is a private Park-Miller generator (16807 x mod
//  2^31 - 1), the same one the MATLAB port (sectorOrienteering.m) uses.
// =============================================================================
#ifndef MTLC_PLANNING_SECTOR_ORIENTEERING_HPP
#define MTLC_PLANNING_SECTOR_ORIENTEERING_HPP

#include <optional>

#include "mtl_curve/params.hpp"
#include "mtl_curve/types.hpp"

namespace mtl::curve::planning {

struct SectorRoute {
    Path2  wp;                 ///< ordered sector centres (launch point excluded)
    double reward  = 0.0;      ///< weight inside the union of the visited boxes
    double length  = 0.0;      ///< [m] Euclidean route length (incl. the leg to the goal, if any)
    double budget  = 0.0;      ///< [m] budgetFrac * L
    Index  nCandidates = 0;    ///< sectors kept as nodes
    double seconds = 0.0;
};

/// @param W       ny-by-nx non-negative weights on a regular raster
/// @param x0, y0  [m] centre of pixel (0, 0); hg = pixel size [m]
/// @param bounds  [xmin xmax ymin ymax] the sector grid covers
/// @param swathHalfWidth  [m] calibrated P>=0.5 half-width of one pass
/// @param start   launch point; goal: forced end point (closed modes) or none
/// @param L       [m] the curve length (the route budget is budgetFrac * L)
SectorRoute solveSectorOrienteering(const MatX& W, double x0, double y0, double hg, const Eigen::Vector4d& bounds,
                                    double swathHalfWidth, const Vec2& start, const std::optional<Vec2>& goal,
                                    double L, const SectorParams& params);

}  // namespace mtl::curve::planning

#endif  // MTLC_PLANNING_SECTOR_ORIENTEERING_HPP
