// =============================================================================
//  mtl_curve/curve_planning/init_spline.hpp
//
//  Seed curve of EXACTLY length L - the port of initParametricSpline.m.
//
//  The plan fits a spline through the ordered centroids and "scales / extends"
//  it to L; scaling moves the start and breaks the curvature bound, and
//  extending it is the straight run-out this planner retires.  The seed is
//  FLOWN instead:
//    1. a unicycle at the optimisation step, turn rate bounded by kappa_max,
//       pursues the ordered centroids; one counts as captured within
//       captureFrac of the swath half-width;
//    2. with the list exhausted it pursues the grid point with the most
//       unswept belief within a half-width per metre of transit (belief swept
//       so far by this seed and by the other agents does not count);
//    3. in the closed modes it heads for the goal once only the distance to it
//       plus a turning margin remains, and orbits the goal with what is left;
//    4. the flown headings are projected onto the representation
//       (quasi-interpolation of the turn rate onto the hats, or least-squares
//       control points).
// =============================================================================
#ifndef MTLC_CURVE_PLANNING_INIT_SPLINE_HPP
#define MTLC_CURVE_PLANNING_INIT_SPLINE_HPP

#include <optional>

#include "mtl_curve/optimization/objective.hpp"
#include "mtl_curve/params.hpp"
#include "mtl_curve/types.hpp"

namespace mtl::curve::curve_planning {

struct InitResult {
    VecX     v0;
    CurveRep rep;
    Path2    path;                  ///< the flown seed polyline, start first
    std::vector<char> captured;     ///< per waypoint
    Path2    greedyTargets;
    Vec2     endPt = Vec2::Zero();
    Index    splicePrefix = 0;        ///< samples copied from the launch point before any deviation
    Index    splicePieceReached = 0;  ///< splice pieces completed (== count: all flown)
};

/// One piece of a spliced seed (reallocation Path seed, see path_insert.hpp):
///   via  fly to pt (counts once within captureFrac of the swath half-width);
///   arc  re-acquire the old curve at node j0 (0-based sample j starts at node
///        j), then copy the OLD headings of samples j0..j1 - the arc flown is
///        the old one, translated by the small offset left at acquisition.
struct SplicePiece {
    bool  via = false;
    Vec2  pt = Vec2::Zero();
    Index j0 = 0, j1 = -1;
};

/// An existing curve (curvature representation, same sampling) and the pieces
/// to fly.  A first arc starting at sample 0 is copied exactly from the launch
/// point; the hats wholly inside it keep their coefficients.  The flown seed is
/// projected by a least-squares heading fit instead of the quasi-interpolant,
/// so the copied arcs survive the projection.  Any length left after the last
/// piece goes to the usual greedy information pursuit.
struct Splice {
    const VecX*     v   = nullptr;
    const CurveRep* rep = nullptr;
    const std::vector<SplicePiece>* pieces = nullptr;
};

/// @param waypoints ordered cluster centroids (may be empty: pure greedy)
/// @param prob      grid, kernel and the other agents' log-miss
/// @param splice    optional: fly pieces of an existing curve first (Path seed)
InitResult initParametricSpline(const Vec2& pStart, const Path2& waypoints, double L,
                                EndpointMode mode, const std::optional<Vec2>& pDest,
                                const CurveParams& curve, const optimization::AgentProblem& prob,
                                const Splice* splice = nullptr);

}  // namespace mtl::curve::curve_planning

#endif  // MTLC_CURVE_PLANNING_INIT_SPLINE_HPP
