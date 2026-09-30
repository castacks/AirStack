// =============================================================================
//  mtl_curve/curve_planning/arc_length.hpp
//
//  Resample a curve at uniform arc-length steps - the port of
//  reparameterizeArcLength.m.  With ds = V*dt that is one sample per trajectory
//  time step (2 m at 20 m/s and 0.1 s).
//
//    Curvature  exact: the heading is integrated with the midpoint rule on the
//               ds grid itself, so the samples are exactly ds apart and the last
//               one is gamma(L).
//    BSpline    evaluated on a fine uniform-u grid (20 per ds), its cumulative
//               arc length computed, positions interpolated at uniform s.  The
//               plan's "rescale to budgetDist" is deliberately NOT done: scaling
//               moves the pinned start and breaks the curvature bound, so the
//               optimiser owns the length and this only reports it.
// =============================================================================
#ifndef MTLC_CURVE_PLANNING_ARC_LENGTH_HPP
#define MTLC_CURVE_PLANNING_ARC_LENGTH_HPP

#include "mtl_curve/types.hpp"

namespace mtl::curve::curve_planning {

struct ArcLengthSamples {
    Path2 pts;     ///< (M+1)-by-2, s = 0, ds, ..., L
    VecX  s;
    Path2 tan;     ///< unit tangents
    Path2 nrm;     ///< unit left normals
    VecX  kappa;
    double L = 0.0;  ///< measured length
};

ArcLengthSamples reparameterizeArcLength(const VecX& v, const CurveRep& rep, double ds);

}  // namespace mtl::curve::curve_planning

#endif  // MTLC_CURVE_PLANNING_ARC_LENGTH_HPP
