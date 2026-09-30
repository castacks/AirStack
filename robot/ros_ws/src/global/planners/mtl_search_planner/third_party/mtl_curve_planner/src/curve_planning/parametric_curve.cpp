#include "mtl_curve/curve_planning/parametric_curve.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "mtl_curve/core/bspline.hpp"

namespace mtl::curve::curve_planning {

MatX hatIntegrals(const VecX& s, const VecX& knots) {
    const Index Nk = knots.size();
    const double h = knots(1) - knots(0);
    const double L = knots(Nk - 1);
    MatX Phi(s.size(), Nk);
    for (Index m = 0; m < Nk; ++m) {
        const double km = knots(m);
        const double lL = std::max(0.0, km - h);
        const double rR = std::min(L, km + h);
        const double IL0 = (lL - km + h) * (lL - km + h);
        for (Index i = 0; i < s.size(); ++i) {
            const double xl = std::min(std::max(s(i), lL), km);
            const double IL = ((xl - km + h) * (xl - km + h) - IL0) / (2.0 * h);
            const double xr = std::min(std::max(s(i), km), rR);
            const double IR = (h * h - (km + h - xr) * (km + h - xr)) / (2.0 * h);
            Phi(i, m) = IL + IR;
        }
    }
    return Phi;
}

VecX hatInterp(const VecX& s, const VecX& knots, const VecX& c) {
    const Index Nk = knots.size();
    const double h = knots(1) - knots(0);
    VecX out(s.size());
    for (Index i = 0; i < s.size(); ++i) {
        const double x = std::min(std::max(s(i), knots(0)), knots(Nk - 1));
        Index j = static_cast<Index>(std::floor((x - knots(0)) / h));
        j = std::min<Index>(std::max<Index>(j, 0), Nk - 2);
        const double f = (x - knots(j)) / h;
        out(i) = (1.0 - f) * c(j) + f * c(j + 1);
    }
    return out;
}

Path2 bsplineControlPoints(const VecX& v, const CurveRep& rep) {
    Path2 Pc = rep.Pfix;
    Index nf = 0;
    for (const char f : rep.fixed) nf += f ? 0 : 1;
    Index k = 0;
    for (Index i = 0; i < rep.Nc; ++i) {
        if (rep.fixed[static_cast<std::size_t>(i)]) continue;
        Pc(i, 0) = v(k);
        Pc(i, 1) = v(nf + k);
        ++k;
    }
    return Pc;
}

CurveSamples evalParametricCurve(const VecX& v, const CurveRep& rep, Index M) {
    CurveSamples C;
    if (rep.type == CurveRepresentation::Curvature) {
        if (M <= 0) M = rep.M;
        const double L = rep.L;
        const double ds = L / static_cast<double>(M);
        VecX s(M);
        for (Index i = 0; i < M; ++i) s(i) = (static_cast<double>(i) + 0.5) * ds;
        C.Phi = hatIntegrals(s, rep.knots);
        const VecX c = v.head(rep.Nk);
        const double th0 = v(rep.Nk);
        C.theta = (C.Phi * c).array() + th0;
        C.pts.resize(M, 2);
        C.tan.resize(M, 2);
        C.nrm.resize(M, 2);
        Vec2 P = rep.pStart;
        for (Index i = 0; i < M; ++i) {
            const double cs = std::cos(C.theta(i)), sn = std::sin(C.theta(i));
            C.tan(i, 0) = cs;
            C.tan(i, 1) = sn;
            C.nrm(i, 0) = -sn;
            C.nrm(i, 1) = cs;
            C.pts(i, 0) = P.x() + 0.5 * ds * cs;
            C.pts(i, 1) = P.y() + 0.5 * ds * sn;
            P.x() += ds * cs;
            P.y() += ds * sn;
        }
        C.ds = VecX::Constant(M, ds);
        C.kappa = hatInterp(s, rep.knots, c);
        C.L = L;
        C.endPt = P;
        C.s = s;
        C.dsSeg = ds;
        return C;
    }

    // ---- BSpline --------------------------------------------------------------
    const Path2 Pc = bsplineControlPoints(v, rep);
    MatX B0, B1, B2;
    double du = rep.du;
    const MatX* pB0 = &rep.B0;
    const MatX* pB1 = &rep.B1;
    const MatX* pB2 = &rep.B2;
    if (M > 0) {
        VecX uu(M);
        for (Index i = 0; i < M; ++i) uu(i) = (static_cast<double>(i) + 0.5) / static_cast<double>(M);
        core::bsplineBasis(uu, rep.p, rep.knotVec, &B0, &B1, &B2);
        pB0 = &B0; pB1 = &B1; pB2 = &B2;
        du = 1.0 / static_cast<double>(M);
    }
    const Path2 r0 = (*pB0) * Pc;
    C.r1 = (*pB1) * Pc;
    C.r2 = (*pB2) * Pc;
    const Index n = r0.rows();
    C.sp.resize(n);
    C.tan.resize(n, 2);
    C.nrm.resize(n, 2);
    C.kappa.resize(n);
    C.ds.resize(n);
    for (Index i = 0; i < n; ++i) {
        const double sp = std::max(C.r1.row(i).norm(), 1e-9);
        C.sp(i) = sp;
        C.tan(i, 0) = C.r1(i, 0) / sp;
        C.tan(i, 1) = C.r1(i, 1) / sp;
        C.nrm(i, 0) = -C.tan(i, 1);
        C.nrm(i, 1) = C.tan(i, 0);
        C.ds(i) = C.r1.row(i).norm() * du;
        C.kappa(i) = (C.r1(i, 0) * C.r2(i, 1) - C.r1(i, 1) * C.r2(i, 0)) / (sp * sp * sp);
    }
    C.pts = r0;
    const Path2 d1s = rep.B1s * Pc;
    C.L = rep.wS.dot(d1s.rowwise().norm());
    C.endPt = Pc.row(rep.Nc - 1).transpose();
    C.s.resize(n);
    double acc = 0.0;
    for (Index i = 0; i < n; ++i) {
        C.s(i) = acc + 0.5 * C.ds(i);
        acc += C.ds(i);
    }
    C.du = du;
    C.Pc = Pc;
    return C;
}

VecX curveVJP(const CurveRep& rep, const CurveSamples& C, const Path2& gp, const VecX& gth,
              const VecX& gds) {
    const Index M = C.pts.rows();
    const bool hasTh = gth.size() == M;
    const bool hasDs = gds.size() == M;
    if (rep.type == CurveRepresentation::Curvature) {
        const double ds = C.dsSeg;
        // sample i sits at P_{i-1} + ds/2 u_i, P_{i-1} = p0 + ds sum_{k<i} u_k:
        //   d q_i / d theta_k = ds n_k (k < i),  ds/2 n_i (k = i)
        VecX gSeg(M);
        Vec2 after = Vec2::Zero();  // sum_{i > k} gp_i
        for (Index k = M - 1; k >= 0; --k) {
            const Vec2 g(gp(k, 0), gp(k, 1));
            const Vec2 nk(C.nrm(k, 0), C.nrm(k, 1));
            gSeg(k) = nk.dot(ds * after + 0.5 * ds * g) + (hasTh ? gth(k) : 0.0);
            after += g;
        }
        VecX g(rep.nv);
        g.head(rep.Nk) = C.Phi.transpose() * gSeg;
        g(rep.Nk) = gSeg.sum();
        return g;
    }
    // BSpline: theta = atan2(r'): d theta / d r' = n / |r'|;  ds = |r'| du
    Path2 gr1(M, 2);
    for (Index i = 0; i < M; ++i) {
        const double a = hasTh ? gth(i) / C.sp(i) : 0.0;
        const double b = hasDs ? gds(i) * C.du : 0.0;
        gr1(i, 0) = a * C.nrm(i, 0) + b * C.tan(i, 0);
        gr1(i, 1) = a * C.nrm(i, 1) + b * C.tan(i, 1);
    }
    const MatX& B0 = rep.B0;
    const MatX& B1 = rep.B1;
    if (B0.rows() != M) throw std::invalid_argument("curveVJP: B-spline VJP needs the optimisation sampling");
    const MatX GP = B0.transpose() * gp + B1.transpose() * gr1;  // Nc-by-2
    Index nf = 0;
    for (const char f : rep.fixed) nf += f ? 0 : 1;
    VecX g(2 * nf);
    Index k = 0;
    for (Index i = 0; i < rep.Nc; ++i) {
        if (rep.fixed[static_cast<std::size_t>(i)]) continue;
        g(k) = GP(i, 0);
        g(nf + k) = GP(i, 1);
        ++k;
    }
    return g;
}

MatX curveEndJacobian(const CurveRep& rep, const CurveSamples& C) {
    if (rep.type != CurveRepresentation::Curvature)
        throw std::invalid_argument("curveEndJacobian: curvature representation only");
    const double ds = C.dsSeg;
    const VecX gx = -ds * C.theta.array().sin();
    const VecX gy = ds * C.theta.array().cos();
    MatX Je(2, rep.nv);
    Je.row(0).head(rep.Nk) = (C.Phi.transpose() * gx).transpose();
    Je.row(1).head(rep.Nk) = (C.Phi.transpose() * gy).transpose();
    Je(0, rep.Nk) = gx.sum();
    Je(1, rep.Nk) = gy.sum();
    return Je;
}

}  // namespace mtl::curve::curve_planning
