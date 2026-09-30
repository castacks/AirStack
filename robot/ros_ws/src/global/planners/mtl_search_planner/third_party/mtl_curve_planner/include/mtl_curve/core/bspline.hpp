// =============================================================================
//  mtl_curve/core/bspline.hpp
//
//  B-spline basis functions and their first two derivatives (Cox-de Boor), the
//  port of functions/curve_planning/bsplineBasis.m.
//
//      N0(i, j) = N_{j,p}(u_i),   N1 = d/du,   N2 = d^2/du^2
//
//  Derivatives use  N'_{j,q} = q ( N_{j,q-1}/(t_{j+q}-t_j) - N_{j+1,q-1}/(t_{j+q+1}-t_{j+1}) ).
//  u == knots.back() is assigned to the last non-empty span, so a clamped curve
//  evaluates to its last control point there.
// =============================================================================
#ifndef MTLC_CORE_BSPLINE_HPP
#define MTLC_CORE_BSPLINE_HPP

#include "mtl_curve/types.hpp"

namespace mtl::curve::core {

/// Clamped uniform knot vector for nCtrl control points of degree p on [0, 1].
VecX clampedUniformKnots(Index nCtrl, int p);

/// Basis (and optionally its first / second derivatives) at the parameter
/// values `u`.  Each output is numel(u)-by-nCtrl; pass nullptr to skip one.
void bsplineBasis(const VecX& u, int p, const VecX& knots, MatX* N0, MatX* N1, MatX* N2);

}  // namespace mtl::curve::core

#endif  // MTLC_CORE_BSPLINE_HPP
