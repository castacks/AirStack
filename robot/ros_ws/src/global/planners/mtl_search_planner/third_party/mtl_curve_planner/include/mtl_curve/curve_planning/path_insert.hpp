// =============================================================================
//  mtl_curve/curve_planning/path_insert.hpp
//
//  Reallocation seed that splices a claimed cluster into an agent's OPTIMISED
//  curve - the port of pathInsertSeed.m.
//
//  The plan's 'insert' seed splices the centroid into the agent's orienteering
//  waypoints and flies a fresh seed through them, discarding everything the
//  optimiser had done.  This one starts from the optimised curve:
//    1. waypoints every team.reallocPathWpSpacing metres along the curve, each
//       with the first-order yield of the arc leading up to it
//         Y_i = sum_j -ds_j sum_x W(x) k(a_j(x), d_j(x))   (W = team residual);
//    2. cheapest insertion of the centroid between consecutive waypoints (in
//       Open mode also after the last one);
//    3. BUDGET REPAIR.  The new path is measured as the old arcs it keeps plus
//       straight connectors, plus team.reallocPathMargin of turning slack per
//       join (and 2.5 pi Rmin of homing slack in the closed modes).  While it
//       overruns L, a window of consecutive waypoints (never the centroid, the
//       launch point or the goal) is shortcut - or, in Open mode, the tail
//       dropped - choosing the window that saves enough for the least lost
//       yield, else the best saving per unit of lost yield;
//    4. the result is a list of SPLICE PIECES - old arcs and the centroid -
//       that initParametricSpline flies: the first arc is copied exactly from
//       the launch point, each later arc is re-acquired after its connector and
//       then copied heading for heading (a small translation), and any length
//       left over goes to the greedy information pursuit.  The flown seed is
//       exactly L long and is projected by a least-squares heading fit, so the
//       optimiser's work on the kept arcs survives into the new seed.
// =============================================================================
#ifndef MTLC_CURVE_PLANNING_PATH_INSERT_HPP
#define MTLC_CURVE_PLANNING_PATH_INSERT_HPP

#include <vector>

#include "mtl_curve/curve_planning/init_spline.hpp"
#include "mtl_curve/params.hpp"
#include "mtl_curve/types.hpp"

namespace mtl::curve::curve_planning {

struct PathInsertSeed {
    std::vector<SplicePiece> pieces; ///< for initParametricSpline's Splice
    Index  prefixSamples = 0;    ///< old samples copied exactly from the launch point
    Index  insertAfter   = 0;    ///< waypoint index the centroid follows (0 = launch point)
    double detour   = 0.0;       ///< [m] Euclidean insertion detour
    double overrun0 = 0.0;       ///< [m] length overrun before the repair
    double overrun  = 0.0;       ///< [m] after (<= 0 when repaired; > 0: the claim does not fit)
    int    cuts     = 0;         ///< windows removed
    double lostYield = 0.0;      ///< first-order information given up by the cuts
    double sPrefix  = 0.0;       ///< [m] length of the exactly kept prefix
};

/// @param v, rep  the agent's current (optimised) curve
/// @param c       centroid of the claimed cluster
/// @param K, G    the agent's accurate kernel and the fast grid
/// @param W       team residual weights, prior .* exp(Lambda_team), on G
PathInsertSeed pathInsertSeed(const VecX& v, const CurveRep& rep, const Vec2& c, const SwathKernel& K,
                              const FastGrid& G, const MatX& W, const TeamParams& team);

}  // namespace mtl::curve::curve_planning

#endif  // MTLC_CURVE_PLANNING_PATH_INSERT_HPP
