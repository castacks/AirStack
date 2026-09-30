// =============================================================================
//  mtl_curve/curve_planning/endpoint_constraints.hpp
//
//  Decision vector, bounds and endpoint regime of one agent's curve - the port
//  of enforceEndpointConstraints.m.  Builds the CurveRep every other stage
//  works from.
//
//    Curvature  v = [c; theta0].  kappa(s) = sum_m c_m hat_m(s) on uniform
//               knots (curveParams.curvatureKnotSpacing), so the arc length is
//               EXACTLY L and, the hats being a non-negative partition of unity,
//               |c_m| <= kappa_max bounds |kappa(s)| everywhere.  The only
//               nonlinear constraint left is the endpoint in the closed modes.
//    BSpline    the plan's clamped uniform B-spline; v = the free control points.
//               P_1 = pStart always, P_Nc = goal in the closed modes, P_2 on the
//               launch heading when curveParams.initialHeading is set.
//
//  Endpoint modes: Open (gamma(0) = start), ReturnHome (gamma(1) = start),
//  FixedDest (gamma(1) = dest, which must be within L of the start).
// =============================================================================
#ifndef MTLC_CURVE_PLANNING_ENDPOINT_CONSTRAINTS_HPP
#define MTLC_CURVE_PLANNING_ENDPOINT_CONSTRAINTS_HPP

#include <optional>

#include "mtl_curve/params.hpp"
#include "mtl_curve/types.hpp"

namespace mtl::curve::curve_planning {

/// @throws std::invalid_argument for FixedDest without a destination, or a
///         destination farther than L.
CurveRep enforceEndpointConstraints(CurveRepresentation type, double L, const Vec2& pStart,
                                    EndpointMode mode, const std::optional<Vec2>& pDest,
                                    const CurveParams& curve);

}  // namespace mtl::curve::curve_planning

#endif  // MTLC_CURVE_PLANNING_ENDPOINT_CONSTRAINTS_HPP
