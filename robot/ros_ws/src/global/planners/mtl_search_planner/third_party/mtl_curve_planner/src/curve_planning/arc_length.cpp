#include "mtl_curve/curve_planning/arc_length.hpp"

#include <algorithm>
#include <cmath>

#include "mtl_curve/core/bspline.hpp"
#include "mtl_curve/curve_planning/parametric_curve.hpp"

namespace mtl::curve::curve_planning {

ArcLengthSamples reparameterizeArcLength(const VecX& v, const CurveRep& rep, double ds) {
    ArcLengthSamples R;
    if (rep.type == CurveRepresentation::Curvature) {
        const double L = rep.L;
        const Index M = std::max<Index>(1, static_cast<Index>(std::lround(L / ds)));
        const double dsu = L / static_cast<double>(M);
        const CurveSamples C = evalParametricCurve(v, rep, M);
        R.pts.resize(M + 1, 2);
        R.pts.row(0) = rep.pStart.transpose();
        for (Index i = 0; i < M; ++i) R.pts.row(i + 1) = R.pts.row(i) + dsu * C.tan.row(i);
        R.s.resize(M + 1);
        for (Index i = 0; i <= M; ++i) R.s(i) = static_cast<double>(i) * dsu;
        VecX th(M + 1);
        th(0) = C.theta(0) - 0.5 * dsu * C.kappa(0);
        for (Index i = 1; i < M; ++i) th(i) = 0.5 * (C.theta(i - 1) + C.theta(i));
        th(M) = C.theta(M - 1) + 0.5 * dsu * C.kappa(M - 1);
        R.tan.resize(M + 1, 2);
        for (Index i = 0; i <= M; ++i) {
            R.tan(i, 0) = std::cos(th(i));
            R.tan(i, 1) = std::sin(th(i));
        }
        R.kappa = hatInterp(R.s, rep.knots, v.head(rep.Nk));
        R.L = L;
    } else {
        const double Lest = std::max(1.0, rep.L);
        const Index nF = std::max<Index>(200, static_cast<Index>(std::lround(20.0 * Lest / ds)));
        const VecX uF = VecX::LinSpaced(nF + 1, 0.0, 1.0);
        const Path2 Pc = bsplineControlPoints(v, rep);
        MatX B1;
        core::bsplineBasis(uF, rep.p, rep.knotVec, nullptr, &B1, nullptr);
        const Path2 r1 = B1 * Pc;
        VecX sc(nF + 1);
        sc(0) = 0.0;
        for (Index i = 1; i <= nF; ++i)
            sc(i) = sc(i - 1) + 0.5 * (r1.row(i - 1).norm() + r1.row(i).norm()) * (uF(i) - uF(i - 1));
        const double L = sc(nF);
        const Index M = std::max<Index>(1, static_cast<Index>(std::lround(L / ds)));
        R.s = VecX::LinSpaced(M + 1, 0.0, L);
        VecX uS(M + 1);
        Index j = 0;
        for (Index i = 0; i <= M; ++i) {
            while (j + 1 < nF && sc(j + 1) < R.s(i)) ++j;
            const double span = sc(j + 1) - sc(j);
            const double f = span > 0.0 ? std::min(std::max((R.s(i) - sc(j)) / span, 0.0), 1.0) : 0.0;
            uS(i) = uF(j) + f * (uF(j + 1) - uF(j));
        }
        MatX b0, b1, b2;
        core::bsplineBasis(uS, rep.p, rep.knotVec, &b0, &b1, &b2);
        R.pts = b0 * Pc;
        const Path2 q1 = b1 * Pc, q2 = b2 * Pc;
        R.tan.resize(M + 1, 2);
        R.kappa.resize(M + 1);
        for (Index i = 0; i <= M; ++i) {
            const double sp = std::max(q1.row(i).norm(), 1e-9);
            R.tan(i, 0) = q1(i, 0) / sp;
            R.tan(i, 1) = q1(i, 1) / sp;
            R.kappa(i) = (q1(i, 0) * q2(i, 1) - q1(i, 1) * q2(i, 0)) / (sp * sp * sp);
        }
        R.L = L;
    }
    R.nrm.resize(R.tan.rows(), 2);
    R.nrm.col(0) = -R.tan.col(1);
    R.nrm.col(1) = R.tan.col(0);
    return R;
}

}  // namespace mtl::curve::curve_planning
