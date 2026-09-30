#include "mtl_curve/optimization/objective.hpp"

#include <algorithm>
#include <cmath>

#include "mtl_curve/curve_planning/parametric_curve.hpp"
#include "mtl_curve/optimization/swath_kernel.hpp"

namespace mtl::curve::optimization {
namespace {

/// d kappa_j / d v at the samples defined by (B1, B2) (rows), for the free
/// control points - the chain rule through r' and r''.
MatX kappaJacobian(const CurveRep& rep, const MatX& B1, const MatX& B2, const Path2& Pc,
                   VecX* kappaOut) {
    const Path2 r1 = B1 * Pc, r2 = B2 * Pc;
    const Index n = r1.rows();
    Index nf = 0;
    for (const char f : rep.fixed) nf += f ? 0 : 1;
    MatX Jk = MatX::Zero(n, 2 * nf);
    if (kappaOut) kappaOut->resize(n);
    for (Index i = 0; i < n; ++i) {
        const double sp = std::max(r1.row(i).norm(), 1e-9);
        const double sp3 = sp * sp * sp, sp5 = sp3 * sp * sp;
        const double cr = r1(i, 0) * r2(i, 1) - r1(i, 1) * r2(i, 0);
        if (kappaOut) (*kappaOut)(i) = cr / sp3;
        const double dk1x = r2(i, 1) / sp3 - 3.0 * cr * r1(i, 0) / sp5;
        const double dk1y = -r2(i, 0) / sp3 - 3.0 * cr * r1(i, 1) / sp5;
        const double dk2x = -r1(i, 1) / sp3;
        const double dk2y = r1(i, 0) / sp3;
        Index k = 0;
        for (Index c = 0; c < rep.Nc; ++c) {
            if (rep.fixed[static_cast<std::size_t>(c)]) continue;
            Jk(i, k)      = B1(i, c) * dk1x + B2(i, c) * dk2x;
            Jk(i, nf + k) = B1(i, c) * dk1y + B2(i, c) * dk2y;
            ++k;
        }
    }
    return Jk;
}

}  // namespace

MatX agentDeposit(const VecX& v, const CurveRep& rep, const FastGrid& G, const SwathKernel& K) {
    const CurveSamples C = curve_planning::evalParametricCurve(v, rep);
    return swathKernelDeposit(C.pts, C.tan, C.ds, G, K);
}

double objectiveResidualBelief(const VecX& v, const CurveRep& rep, const AgentProblem& prob,
                               VecX* grad, MatX* lambdaOut) {
    const FastGrid& G = *prob.G;
    const CurveSamples C = curve_planning::evalParametricCurve(v, rep);
    const MatX Lam = swathKernelDeposit(C.pts, C.tan, C.ds, G, *prob.K);
    MatX W;
    if (prob.lambdaOther) W = (G.prior.array() * (*prob.lambdaOther + Lam).array().exp()).matrix();
    else W = (G.prior.array() * Lam.array().exp()).matrix();
    double J = W.sum();

    double lamK = 0.0;
    if (rep.type == CurveRepresentation::BSpline) lamK = rep.curvaturePenaltyWeight;
    VecX ex;
    if (lamK > 0.0) {
        ex = (C.kappa.cwiseAbs().array() - rep.kappaMax).cwiseMax(0.0).matrix();
        J += lamK * (ex.array().square() * C.ds.array()).sum();
    }

    if (grad) {
        Path2 gp;
        VecX gth, gds;
        swathKernelGradient(C.pts, C.tan, C.ds, G, *prob.K, W, gp, gth, gds);
        *grad = curve_planning::curveVJP(rep, C, gp, gth, gds);
        if (lamK > 0.0 && ex.maxCoeff() > 0.0) {
            // penalty: through kappa (r', r'') and through ds (|r'|)
            const MatX Jk = kappaJacobian(rep, rep.B1, rep.B2, C.Pc, nullptr);
            VecX wk(C.kappa.size());
            for (Index i = 0; i < wk.size(); ++i)
                wk(i) = lamK * 2.0 * ex(i) * (C.kappa(i) > 0 ? 1.0 : (C.kappa(i) < 0 ? -1.0 : 0.0)) * C.ds(i);
            *grad += Jk.transpose() * wk;
            const VecX gdsPen = lamK * ex.array().square().matrix();
            *grad += curve_planning::curveVJP(rep, C, Path2::Zero(C.pts.rows(), 2), VecX(), gdsPen);
        }
    }
    if (lambdaOut) *lambdaOut = Lam;
    return J;
}

void constraintCurvatureLength(const VecX& v, const CurveRep& rep, VecX& cin, VecX& ceq, MatX* gcin,
                               MatX* gceq) {
    if (rep.type == CurveRepresentation::Curvature) {
        cin.resize(0);
        if (gcin) gcin->resize(rep.nv, 0);
        if (!rep.hasEq()) {
            ceq.resize(0);
            if (gceq) gceq->resize(rep.nv, 0);
            return;
        }
        const CurveSamples C = curve_planning::evalParametricCurve(v, rep);
        ceq = (C.endPt - *rep.pGoal) / rep.L;
        if (gceq) *gceq = (curve_planning::curveEndJacobian(rep, C) / rep.L).transpose();
        return;
    }
    // ---- BSpline --------------------------------------------------------------
    const Path2 Pc = curve_planning::bsplineControlPoints(v, rep);
    const bool dense = rep.BK1.rows() > 0;
    const MatX& B1 = dense ? rep.BK1 : rep.B1;
    const MatX& B2 = dense ? rep.BK2 : rep.B2;
    const double km = dense ? rep.kappaCon : rep.kappaMax;
    VecX kap;
    const MatX Jk = kappaJacobian(rep, B1, B2, Pc, &kap);
    const Index n = kap.size();
    cin.resize(2 * n);
    cin.head(n) = kap / km - VecX::Ones(n);
    cin.tail(n) = -kap / km - VecX::Ones(n);
    const Path2 d1s = rep.B1s * Pc;
    const double L = rep.wS.dot(d1s.rowwise().norm());
    ceq.resize(1);
    ceq(0) = L / rep.L - 1.0;
    if (gcin) {
        gcin->resize(rep.nv, 2 * n);
        gcin->leftCols(n) = (Jk / km).transpose();
        gcin->rightCols(n) = (-Jk / km).transpose();
    }
    if (gceq) {
        Path2 ts(d1s.rows(), 2);
        for (Index i = 0; i < d1s.rows(); ++i) {
            const double nrm = std::max(d1s.row(i).norm(), 1e-9);
            ts(i, 0) = rep.wS(i) * d1s(i, 0) / nrm;
            ts(i, 1) = rep.wS(i) * d1s(i, 1) / nrm;
        }
        const MatX GL = rep.B1s.transpose() * ts;  // Nc-by-2
        Index nf = 0;
        for (const char f : rep.fixed) nf += f ? 0 : 1;
        gceq->resize(rep.nv, 1);
        Index k = 0;
        for (Index c = 0; c < rep.Nc; ++c) {
            if (rep.fixed[static_cast<std::size_t>(c)]) continue;
            (*gceq)(k, 0) = GL(c, 0) / rep.L;
            (*gceq)(nf + k, 0) = GL(c, 1) / rep.L;
            ++k;
        }
    }
}

}  // namespace mtl::curve::optimization
