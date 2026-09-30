// The curve representations: B-spline basis, exact length and curvature bound
// of the intrinsic curve, pinned endpoints, the backward pass (VJP) and the
// endpoint Jacobian against finite differences, and uniform arc-length
// resampling.
#include <cmath>
#include <random>

#include "mtl_curve/core/bspline.hpp"
#include "mtl_curve/curve_planning/arc_length.hpp"
#include "mtl_curve/curve_planning/endpoint_constraints.hpp"
#include "mtl_curve/curve_planning/parametric_curve.hpp"
#include "test_util.hpp"

using namespace mtl::curve;
using namespace mtl::curve::curve_planning;

namespace {

CurveParams curveParams() {
    CurveParams c;
    c.maxCurvature = 0.01;
    c.sampleSpacing = 25.0;
    c.curvatureKnotSpacing = 100.0;
    c.controlPointBox = Vec2(-1000, 6000);
    return c;
}

VecX randomCurvature(const CurveRep& rep, std::mt19937_64& rng) {
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    VecX v(rep.nv);
    for (Index i = 0; i < rep.Nk; ++i) v(i) = 0.8 * rep.ub(i) * u(rng);
    v(rep.Nk) = 3.0 * u(rng);
    return v;
}

VecX randomBspline(const CurveRep& rep, std::mt19937_64& rng, double noise = 40.0) {
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    // a gentle curve: control points along a wavy line from the start
    const Path2 P = [&] {
        Path2 p(rep.Nc, 2);
        for (Index i = 0; i < rep.Nc; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(rep.Nc - 1);
            p(i, 0) = rep.pStart.x() + 900.0 * std::sin(kPi * t) + noise * u(rng);
            p(i, 1) = rep.pStart.y() + 1500.0 * t + 200.0 * std::sin(2.0 * kPi * t) + noise * u(rng);
        }
        return p;
    }();
    Index nf = 0;
    for (const char f : rep.fixed) nf += f ? 0 : 1;
    VecX v(2 * nf);
    Index k = 0;
    for (Index i = 0; i < rep.Nc; ++i) {
        if (rep.fixed[static_cast<std::size_t>(i)]) continue;
        v(k) = P(i, 0);
        v(nf + k) = P(i, 1);
        ++k;
    }
    return v;
}

/// A scalar of the samples: sum_i w_i . p_i + a_i theta_i + b_i ds_i, whose
/// gradient w.r.t. v the VJP must reproduce.
double functional(const CurveSamples& C, const Path2& Wp, const VecX& a, const VecX& b) {
    double s = 0.0;
    for (Index i = 0; i < C.pts.rows(); ++i) {
        s += Wp(i, 0) * C.pts(i, 0) + Wp(i, 1) * C.pts(i, 1);
        s += a(i) * std::atan2(C.tan(i, 1), C.tan(i, 0)) + b(i) * C.ds(i);
    }
    return s;
}

}  // namespace

int main() {
    std::mt19937_64 rng(3);

    // --- B-spline basis: partition of unity, clamped ends, derivatives ------
    {
        const VecX t = core::clampedUniformKnots(12, 3);
        CHECK(t.size() == 16);
        const VecX u = VecX::LinSpaced(401, 0.0, 1.0);
        MatX N0, N1, N2;
        core::bsplineBasis(u, 3, t, &N0, &N1, &N2);
        double pou = 0.0;
        for (Index i = 0; i < u.size(); ++i) pou = std::max(pou, std::abs(N0.row(i).sum() - 1.0));
        CHECK(pou < 1e-12);
        CHECK_NEAR(N0(0, 0), 1.0, 1e-12);
        CHECK_NEAR(N0(400, 11), 1.0, 1e-12);
        // derivative against a central difference at interior points
        const double h = 1e-6;
        double e1 = 0.0, e2 = 0.0;
        for (const double uu : {0.13, 0.37, 0.52, 0.81}) {
            VecX q(3);
            q << uu - h, uu, uu + h;
            MatX A0, A1, A2;
            core::bsplineBasis(q, 3, t, &A0, &A1, &A2);
            e1 = std::max(e1, ((A0.row(2) - A0.row(0)) / (2 * h) - A1.row(1)).cwiseAbs().maxCoeff());
            e2 = std::max(e2, ((A1.row(2) - A1.row(0)) / (2 * h) - A2.row(1)).cwiseAbs().maxCoeff());
        }
        CHECK(e1 < 1e-5);
        CHECK(e2 < 1e-3);
    }

    const CurveParams cp = curveParams();
    const Vec2 start(2000, 2000);

    // --- curvature representation: exact length, bounded curvature ---------
    {
        const CurveRep rep = enforceEndpointConstraints(CurveRepresentation::Curvature, 5000.0, start,
                                                        EndpointMode::Open, std::nullopt, cp);
        CHECK(rep.Nk == 51);
        CHECK(rep.nv == 52);
        CHECK(!rep.hasEq());
        for (int trial = 0; trial < 5; ++trial) {
            const VecX v = randomCurvature(rep, rng);
            for (const Index M : {Index(0), Index(2500)}) {
                const CurveSamples C = evalParametricCurve(v, rep, M);
                CHECK_NEAR(C.ds.sum(), 5000.0, 1e-9);
                CHECK(C.kappa.cwiseAbs().maxCoeff() <= cp.curvatureMargin * cp.maxCurvature + 1e-12);
                CHECK_NEAR(C.pts(0, 0) - 0.5 * C.ds(0) * C.tan(0, 0), start.x(), 1e-9);
            }
            // the 25 m optimisation polyline and the 2 m flown one agree on the
            // end point to well under the 1 m endpoint tolerance
            const CurveSamples a = evalParametricCurve(v, rep), b = evalParametricCurve(v, rep, 2500);
            CHECK((a.endPt - b.endPt).norm() < 3.0);
        }
    }

    // --- VJP and endpoint Jacobian against finite differences ---------------
    for (const CurveRepresentation type : {CurveRepresentation::Curvature, CurveRepresentation::BSpline}) {
        const CurveRep rep = enforceEndpointConstraints(type, 5000.0, start, EndpointMode::FixedDest,
                                                        Vec2(3000, 3500), cp);
        CHECK(rep.hasEq());
        const VecX v = type == CurveRepresentation::Curvature ? randomCurvature(rep, rng) : randomBspline(rep, rng);
        const CurveSamples C = evalParametricCurve(v, rep);
        std::normal_distribution<double> nd(0.0, 1.0);
        Path2 Wp(C.pts.rows(), 2);
        VecX a(C.pts.rows()), b(C.pts.rows());
        for (Index i = 0; i < C.pts.rows(); ++i) {
            Wp(i, 0) = nd(rng);
            Wp(i, 1) = nd(rng);
            a(i) = 100.0 * nd(rng);
            b(i) = nd(rng);
        }
        const VecX g = curveVJP(rep, C, Wp, a, b);
        CHECK(g.size() == rep.nv);
        double worst = 0.0;
        for (int trial = 0; trial < 4; ++trial) {
            VecX d(rep.nv);
            for (Index i = 0; i < rep.nv; ++i) d(i) = nd(rng);
            if (type == CurveRepresentation::Curvature) d.head(rep.Nk) *= 1e-3;
            else d *= 10.0;
            const double e = 1e-5;
            const double fp = functional(evalParametricCurve(v + e * d, rep), Wp, a, b);
            const double fm = functional(evalParametricCurve(v - e * d, rep), Wp, a, b);
            const double fd = (fp - fm) / (2 * e);
            worst = std::max(worst, std::abs(fd - g.dot(d)) / std::max(1.0, std::abs(fd)));
        }
        std::printf("  VJP (%s): worst relative error %.2e\n", toString(type), worst);
        CHECK(worst < 1e-5);

        if (type == CurveRepresentation::Curvature) {
            const MatX Je = curveEndJacobian(rep, C);
            VecX d(rep.nv);
            for (Index i = 0; i < rep.nv; ++i) d(i) = nd(rng);
            d.head(rep.Nk) *= 1e-3;
            const double e = 1e-6;
            const Vec2 fd = (evalParametricCurve(v + e * d, rep).endPt - evalParametricCurve(v - e * d, rep).endPt) / (2 * e);
            CHECK((fd - Je * d).norm() < 1e-4 * std::max(1.0, fd.norm()));
        } else {
            // pinned control points really are pinned
            const Path2 Pc = bsplineControlPoints(v, rep);
            CHECK_NEAR((Pc.row(0).transpose() - start).norm(), 0.0, 1e-12);
            CHECK_NEAR((Pc.row(rep.Nc - 1).transpose() - Vec2(3000, 3500)).norm(), 0.0, 1e-12);
            CHECK_NEAR((C.endPt - Vec2(3000, 3500)).norm(), 0.0, 1e-12);
        }
    }

    // --- initial heading pins theta0 (curvature) and P_2 (B-spline) ----------
    {
        CurveParams c = cp;
        c.initialHeading = 0.7;
        const CurveRep rc = enforceEndpointConstraints(CurveRepresentation::Curvature, 4000, start, EndpointMode::Open, std::nullopt, c);
        CHECK(rc.lb(rc.Nk) == 0.7 && rc.ub(rc.Nk) == 0.7);
        const CurveRep rb = enforceEndpointConstraints(CurveRepresentation::BSpline, 4000, start, EndpointMode::Open, std::nullopt, c);
        CHECK(rb.fixed[1] == 1);
        CHECK_NEAR(std::atan2(rb.Pfix(1, 1) - start.y(), rb.Pfix(1, 0) - start.x()), 0.7, 1e-12);
    }

    // --- infeasible destination is rejected at construction -----------------
    {
        bool threw = false;
        try {
            enforceEndpointConstraints(CurveRepresentation::Curvature, 1000, start, EndpointMode::FixedDest,
                                       Vec2(4000, 4000), cp);
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        CHECK(threw);
    }

    // --- uniform arc-length resampling ---------------------------------------
    // (a smooth B-spline: on a kinked one the 2 m CHORD of a 2 m arc is shorter)
    for (const CurveRepresentation type : {CurveRepresentation::Curvature, CurveRepresentation::BSpline}) {
        const CurveRep rep = enforceEndpointConstraints(type, 5000.0, start, EndpointMode::Open, std::nullopt, cp);
        const VecX v = type == CurveRepresentation::Curvature ? randomCurvature(rep, rng) : randomBspline(rep, rng, 0.0);
        const ArcLengthSamples R = reparameterizeArcLength(v, rep, 2.0);
        double sMin = kInf, sMax = 0.0;
        for (Index i = 1; i < R.pts.rows(); ++i) {
            const double s = (R.pts.row(i) - R.pts.row(i - 1)).norm();
            sMin = std::min(sMin, s);
            sMax = std::max(sMax, s);
        }
        std::printf("  resample (%s): step %.4f .. %.4f m\n", toString(type), sMin, sMax);
        CHECK(sMax - sMin < 0.02);
        CHECK_NEAR((R.pts.row(0).transpose() - start).norm(), 0.0, 1e-9);
        if (type == CurveRepresentation::Curvature) CHECK_NEAR(R.L, 5000.0, 1e-9);
        CHECK(R.tan.rowwise().norm().maxCoeff() < 1.0 + 1e-12);
    }

    return test::report("test_curve_geometry");
}
