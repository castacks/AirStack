// =============================================================================
//  mtl_curve/curve_planning/swath_polygon.hpp
//
//  Lateral swath boundary and grid mask of one curve - the port of
//  computeSwathPolygon.m.  The plan's
//      Omega = { gamma(u) + w n(u) : |w| <= W_half }
//  is drawn with W_half = the kernel's halfWidth (one straight pass still
//  detects with P >= 0.5).  The boundary is a geometric offset of the ground
//  track; the mask is the exact fast-model coverage, so it also shows the fan a
//  tilted mount sweeps in a turn.
// =============================================================================
#ifndef MTLC_CURVE_PLANNING_SWATH_POLYGON_HPP
#define MTLC_CURVE_PLANNING_SWATH_POLYGON_HPP

#include "mtl_curve/types.hpp"

namespace mtl::curve::curve_planning {

struct SwathPolygon {
    Path2  center, left, right;   ///< ground track and its +/- halfWidth offsets
    double halfWidth = 0.0;
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> mask;  ///< on G: single-pass P >= 0.5
    double area = 0.0;            ///< [m^2] of the mask
};

/// @param halfWidth [m] boundary offset (AgentPlan::swathHalfWidth)
/// @param K, G      kernel and grid to rasterise the mask on (both or neither)
/// @param ds        [m] boundary sample spacing
SwathPolygon computeSwathPolygon(const VecX& v, const CurveRep& rep, double halfWidth,
                                 const SwathKernel* K = nullptr, const FastGrid* G = nullptr,
                                 double ds = 10.0);

}  // namespace mtl::curve::curve_planning

#endif  // MTLC_CURVE_PLANNING_SWATH_POLYGON_HPP
