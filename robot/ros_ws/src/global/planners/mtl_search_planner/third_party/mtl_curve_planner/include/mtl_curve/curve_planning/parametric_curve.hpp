// =============================================================================
//  mtl_curve/curve_planning/parametric_curve.hpp
//
//  Points, tangents, normals, arc weights and curvature of a curve, and the
//  backward pass the optimiser needs - the port of evalParametricCurve.m.
//
//  Curvature representation: M segments of equal length ds = L/M; segment i
//  has heading theta_i = theta0 + Phi(s_i) c at its midpoint s_i, and sample i
//  sits at that midpoint:
//        q_i = P_{i-1} + ds/2 u_i,   P_i = P_{i-1} + ds u_i,   u = [cos, sin]
//  so the polyline has EXACTLY the budget length and |kappa| <= kappa_max
//  holds everywhere.
//
//  B-spline representation:
//        r(u) = sum N_i(u) P_i,   kappa = (x'y'' - y'x'')/|r'|^3,   ds = |r'| du
// =============================================================================
#ifndef MTLC_CURVE_PLANNING_PARAMETRIC_CURVE_HPP
#define MTLC_CURVE_PLANNING_PARAMETRIC_CURVE_HPP

#include "mtl_curve/types.hpp"

namespace mtl::curve::curve_planning {

/// Sample the curve.  M = 0 uses the representation's optimisation sampling
/// (rep.M segments / rep.Mu samples); M > 0 resamples densely (the flown
/// trajectory, final checks).
CurveSamples evalParametricCurve(const VecX& v, const CurveRep& rep, Index M = 0);

/// Vector-Jacobian product (backprop): gradient w.r.t. v of a scalar whose
/// gradient w.r.t. the sample positions is gp (M-by-2), w.r.t. the sample
/// headings atan2(tan) is gth (M; may be empty) and w.r.t. the arc weights is
/// gds (M; may be empty).  `C` must come from evalParametricCurve(v, rep) with
/// the same sampling.
VecX curveVJP(const CurveRep& rep, const CurveSamples& C, const Path2& gp, const VecX& gth,
              const VecX& gds);

/// 2-by-nv Jacobian of the terminal point (curvature representation only).
MatX curveEndJacobian(const CurveRep& rep, const CurveSamples& C);

/// All Nc control points of a B-spline curve (pinned + free).
Path2 bsplineControlPoints(const VecX& v, const CurveRep& rep);

/// Integrals of the piecewise-linear hat basis on uniform knots:
/// Phi(i, m) = int_0^{s_i} hat_m.
MatX hatIntegrals(const VecX& s, const VecX& knots);

/// Piecewise-linear interpolation of the knot values c at s (the curvature).
VecX hatInterp(const VecX& s, const VecX& knots, const VecX& c);

}  // namespace mtl::curve::curve_planning

#endif  // MTLC_CURVE_PLANNING_PARAMETRIC_CURVE_HPP
