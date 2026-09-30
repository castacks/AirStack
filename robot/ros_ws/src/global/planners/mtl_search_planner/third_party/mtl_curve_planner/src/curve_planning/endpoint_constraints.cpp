#include "mtl_curve/curve_planning/endpoint_constraints.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include "mtl_curve/core/bspline.hpp"

namespace mtl::curve::curve_planning {

CurveRep enforceEndpointConstraints(CurveRepresentation type, double L, const Vec2& pStart,
                                    EndpointMode mode, const std::optional<Vec2>& pDest,
                                    const CurveParams& curve) {
    if (!(L > 0.0) || !std::isfinite(L))
        throw std::invalid_argument("enforceEndpointConstraints: the curve length must be finite and positive");

    CurveRep rep;
    rep.type = type;
    rep.L = L;
    rep.pStart = pStart;
    rep.mode = mode;
    rep.kappaMax = curve.maxCurvature;
    const double kMargin = curve.curvatureMargin;
    const double dsOpt = curve.sampleSpacing;

    switch (mode) {
        case EndpointMode::Open: rep.pGoal.reset(); break;
        case EndpointMode::ReturnHome: rep.pGoal = pStart; break;
        case EndpointMode::FixedDest:
            if (!pDest) throw std::invalid_argument("enforceEndpointConstraints: fixed_dest needs a destination");
            if ((*pDest - pStart).norm() > L) {
                throw std::invalid_argument(
                    "enforceEndpointConstraints: destination is " +
                    std::to_string((*pDest - pStart).norm()) + " m away but the budget is only " +
                    std::to_string(L) + " m");
            }
            rep.pGoal = *pDest;
            break;
    }

    if (type == CurveRepresentation::Curvature) {
        const Index Nk = std::max<Index>(4, static_cast<Index>(std::lround(L / curve.curvatureKnotSpacing)) + 1);
        rep.Nk = Nk;
        rep.knots = VecX::LinSpaced(Nk, 0.0, L);
        rep.M = std::max<Index>(10, static_cast<Index>(std::lround(L / dsOpt)));
        rep.nv = Nk + 1;
        rep.lb.resize(rep.nv);
        rep.ub.resize(rep.nv);
        rep.lb.head(Nk).setConstant(-kMargin * rep.kappaMax);
        rep.ub.head(Nk).setConstant(kMargin * rep.kappaMax);
        rep.lb(Nk) = -kInf;
        rep.ub(Nk) = kInf;
        if (curve.initialHeading) {
            rep.lb(Nk) = *curve.initialHeading;
            rep.ub(Nk) = *curve.initialHeading;
        }
        return rep;
    }

    // ---- BSpline --------------------------------------------------------------
    const Index Nc = curve.numControlPoints;
    const int   p  = curve.splineDegree;
    if (Nc < p + 1) throw std::invalid_argument("enforceEndpointConstraints: need at least degree+1 control points");
    rep.Nc = Nc;
    rep.p = p;
    rep.knotVec = core::clampedUniformKnots(Nc, p);
    rep.fixed.assign(static_cast<std::size_t>(Nc), 0);
    rep.Pfix = Path2::Zero(Nc, 2);
    rep.fixed[0] = 1;
    rep.Pfix.row(0) = pStart.transpose();
    if (rep.pGoal) {
        rep.fixed[static_cast<std::size_t>(Nc - 1)] = 1;
        rep.Pfix.row(Nc - 1) = rep.pGoal->transpose();
    }
    if (curve.initialHeading) {
        const double dKnot = L / static_cast<double>(Nc - 1);
        rep.fixed[1] = 1;
        rep.Pfix.row(1) = (pStart + dKnot * Vec2(std::cos(*curve.initialHeading),
                                                 std::sin(*curve.initialHeading))).transpose();
    }
    Index nFree = 0;
    for (const char f : rep.fixed) nFree += f ? 0 : 1;
    rep.nv = 2 * nFree;
    const Vec2 box = curve.controlPointBox.value_or(Vec2(-kInf, kInf));
    rep.lb = VecX::Constant(rep.nv, box.x());
    rep.ub = VecX::Constant(rep.nv, box.y());

    rep.Mu = std::max<Index>(20, static_cast<Index>(std::lround(L / dsOpt)));
    rep.du = 1.0 / static_cast<double>(rep.Mu);
    VecX u(rep.Mu);
    for (Index i = 0; i < rep.Mu; ++i) u(i) = (static_cast<double>(i) + 0.5) / static_cast<double>(rep.Mu);
    core::bsplineBasis(u, p, rep.knotVec, &rep.B0, &rep.B1, &rep.B2);

    // curvature constraint set: every ~5 m, so it cannot peak between samples
    const Index Mk = std::max<Index>(400, static_cast<Index>(std::lround(L / 5.0)));
    VecX uK(Mk);
    for (Index i = 0; i < Mk; ++i) uK(i) = (static_cast<double>(i) + 0.5) / static_cast<double>(Mk);
    core::bsplineBasis(uK, p, rep.knotVec, nullptr, &rep.BK1, &rep.BK2);
    rep.kappaCon = kMargin * rep.kappaMax;

    // composite Simpson for the length (plan: 500 intervals)
    const Index nS = 500;
    const VecX uS = VecX::LinSpaced(nS + 1, 0.0, 1.0);
    core::bsplineBasis(uS, p, rep.knotVec, nullptr, &rep.B1s, nullptr);
    rep.wS.resize(nS + 1);
    for (Index i = 0; i <= nS; ++i) rep.wS(i) = (i == 0 || i == nS) ? 1.0 : (i % 2 == 1 ? 4.0 : 2.0);
    rep.wS /= 3.0 * static_cast<double>(nS);
    rep.curvaturePenaltyWeight = curve.curvaturePenaltyWeight;
    return rep;
}

}  // namespace mtl::curve::curve_planning
