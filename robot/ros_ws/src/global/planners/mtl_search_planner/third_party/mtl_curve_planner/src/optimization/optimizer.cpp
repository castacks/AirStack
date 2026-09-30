#include "mtl_curve/optimization/optimizer.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <deque>
#include <functional>
#include <string>

#include <Eigen/Cholesky>

#include "mtl_curve/curve_planning/parametric_curve.hpp"

namespace mtl::curve::optimization {
namespace {

// --- the smooth box transform ------------------------------------------------
struct Transform {
    std::vector<char> fixed, box;
    VecX mid, half, lb;

    explicit Transform(const CurveRep& rep) {
        const Index n = rep.nv;
        fixed.assign(static_cast<std::size_t>(n), 0);
        box.assign(static_cast<std::size_t>(n), 0);
        mid = VecX::Zero(n);
        half = VecX::Ones(n);
        lb = rep.lb;
        for (Index i = 0; i < n; ++i) {
            const auto k = static_cast<std::size_t>(i);
            fixed[k] = rep.lb(i) == rep.ub(i) ? 1 : 0;
            box[k] = (!fixed[k] && std::isfinite(rep.lb(i)) && std::isfinite(rep.ub(i))) ? 1 : 0;
            if (box[k]) {
                mid(i) = 0.5 * (rep.lb(i) + rep.ub(i));
                half(i) = 0.5 * (rep.ub(i) - rep.lb(i));
            }
        }
    }
    VecX toZ(const VecX& v) const {
        VecX z = v;
        for (Index i = 0; i < v.size(); ++i) {
            const auto k = static_cast<std::size_t>(i);
            if (box[k]) z(i) = std::asin(std::min(std::max((v(i) - mid(i)) / half(i), -0.995), 0.995));
            if (fixed[k]) z(i) = 0.0;
        }
        return z;
    }
    VecX toV(const VecX& z, VecX* dvdz = nullptr) const {
        VecX v = z;
        if (dvdz) *dvdz = VecX::Ones(z.size());
        for (Index i = 0; i < z.size(); ++i) {
            const auto k = static_cast<std::size_t>(i);
            if (box[k]) {
                v(i) = mid(i) + half(i) * std::sin(z(i));
                if (dvdz) (*dvdz)(i) = half(i) * std::cos(z(i));
            }
            if (fixed[k]) {
                v(i) = lb(i);
                if (dvdz) (*dvdz)(i) = 0.0;
            }
        }
        return v;
    }
};

using Fun = std::function<double(const VecX&, VecX&)>;

struct LbfgsOut {
    int iters = 0, fevals = 0;
    std::string msg;
};

LbfgsOut lbfgs(const Fun& fun, VecX& z, int maxIter, const OptimizerParams& o) {
    LbfgsOut out;
    VecX g;
    double F = fun(z, g);
    out.fevals = 1;
    std::deque<VecX> S, Y;
    out.msg = "maxIter";
    int small = 0;
    for (int it = 1; it <= maxIter; ++it) {
        out.iters = it;
        VecX q = g;
        const std::size_t k = S.size();
        std::vector<double> al(k), rho(k);
        for (std::size_t ii = k; ii-- > 0;) {
            rho[ii] = 1.0 / Y[ii].dot(S[ii]);
            al[ii] = rho[ii] * S[ii].dot(q);
            q -= al[ii] * Y[ii];
        }
        if (k > 0) {
            q *= S[k - 1].dot(Y[k - 1]) / Y[k - 1].dot(Y[k - 1]);
        } else {
            q *= 0.05 / std::max(q.cwiseAbs().maxCoeff(), 1e-300);   // first step: 0.05 rad max
        }
        for (std::size_t ii = 0; ii < k; ++ii) {
            const double b = rho[ii] * Y[ii].dot(q);
            q += S[ii] * (al[ii] - b);
        }
        VecX d = -q;
        double gd = g.dot(d);
        if (!(gd < 0.0)) {                                   // not a descent direction
            S.clear();
            Y.clear();
            d = -g * (0.05 / std::max(g.cwiseAbs().maxCoeff(), 1e-300));
            gd = g.dot(d);
        }
        double step = 1.0;
        bool ok = false;
        VecX zn, gn;
        double Fn = F;
        for (int ls = 0; ls < 25; ++ls) {
            zn = z + step * d;
            Fn = fun(zn, gn);
            ++out.fevals;
            if (Fn <= F + 1e-4 * step * gd) {
                ok = true;
                break;
            }
            step *= 0.4;
        }
        if (!ok) {
            out.msg = "line search failed";
            break;
        }
        const VecX s = zn - z, y = gn - g;
        if (s.dot(y) > 1e-12 * s.norm() * y.norm()) {
            S.push_back(s);
            Y.push_back(y);
            if (static_cast<int>(S.size()) > o.lbfgsMemory) {
                S.pop_front();
                Y.pop_front();
            }
        }
        const double rel = (F - Fn) / std::max(1e-12, std::abs(F));
        z = zn;
        F = Fn;
        g = gn;
        small = rel < o.ftol ? small + 1 : 0;
        if (small >= 4) {
            out.msg = "ftol";
            break;
        }
    }
    return out;
}

bool hasConstraints(const CurveRep& rep) {
    return rep.type == CurveRepresentation::BSpline || rep.hasEq();
}

/// Augmented-Lagrangian + L-BFGS (optimizeAgentCurve.m's internalAL).
VecX internalAL(const VecX& v0, const CurveRep& rep, const AgentProblem& prob, const OptimizerParams& o,
                int maxIter, LbfgsOut& st) {
    const Transform T(rep);
    VecX z = T.toZ(v0);
    st = LbfgsOut{};

    auto alObj = [&](const VecX& lam, const VecX& nu, double mu) -> Fun {
        return [&, lam, nu, mu](const VecX& zz, VecX& g) {
            VecX dvdz;
            const VecX v = T.toV(zz, &dvdz);
            VecX gv;
            double F = objectiveResidualBelief(v, rep, prob, &gv);
            if (mu > 0.0) {
                VecX cin, ceq;
                MatX gcin, gceq;
                constraintCurvatureLength(v, rep, cin, ceq, &gcin, &gceq);
                if (ceq.size() > 0) {
                    F += lam.dot(ceq) + 0.5 * mu * ceq.squaredNorm();
                    gv += gceq * (lam + mu * ceq);
                }
                if (cin.size() > 0) {
                    const VecX a = (nu + mu * cin).cwiseMax(0.0);
                    F += (a.squaredNorm() - nu.squaredNorm()) / (2.0 * mu);
                    gv += gcin * a;
                }
            }
            g = gv.cwiseProduct(dvdz);
            for (Index i = 0; i < g.size(); ++i)
                if (T.fixed[static_cast<std::size_t>(i)]) g(i) = 0.0;
            return F;
        };
    };

    if (!hasConstraints(rep)) {
        st = lbfgs(alObj(VecX(), VecX(), 0.0), z, maxIter, o);
        return T.toV(z);
    }

    VecX cin, ceq;
    constraintCurvatureLength(T.toV(z), rep, cin, ceq);
    VecX lam = VecX::Zero(ceq.size()), nu = VecX::Zero(cin.size());
    double mu = o.alMu0;
    double prevViol = kInf;
    for (int outer = 1; outer <= o.alOuter; ++outer) {
        const int nIt = outer == 1 ? maxIter : std::max(20, static_cast<int>(std::lround(maxIter / 3.0)));
        const LbfgsOut r = lbfgs(alObj(lam, nu, mu), z, nIt, o);
        st.iters += r.iters;
        st.fevals += r.fevals;
        st.msg = r.msg;
        const VecX v = T.toV(z);
        constraintCurvatureLength(v, rep, cin, ceq);
        double viol = 0.0;
        if (ceq.size() > 0) viol = std::max(viol, ceq.cwiseAbs().maxCoeff());
        if (cin.size() > 0) viol = std::max(viol, cin.maxCoeff());
        lam += mu * ceq;
        nu = (nu + mu * cin).cwiseMax(0.0);
        if (viol < 1e-6) break;
        if (viol > 0.25 * prevViol) mu *= 10.0;
        prevViol = viol;
    }
    return T.toV(z);
}

/// Gauss-Newton minimum-norm steps onto the equality constraints, on the
/// dense (flown) discretisation.
VecX projectFeasible(VecX v, const CurveRep& rep, double denseStep, OptimizeInfo& info) {
    const Index Md = std::max<Index>(10, static_cast<Index>(std::lround(rep.L / denseStep)));
    if (rep.type == CurveRepresentation::Curvature) {
        if (!rep.hasEq()) return v;
        VecX w = VecX::Constant(rep.nv, rep.kappaMax * rep.kappaMax);
        w(rep.Nk) = 1.0;
        for (Index i = 0; i < rep.nv; ++i)
            if (rep.lb(i) == rep.ub(i)) w(i) = 0.0;
        for (int k = 1; k <= 30; ++k) {
            const CurveSamples C = curve_planning::evalParametricCurve(v, rep, Md);
            const Vec2 h = C.endPt - *rep.pGoal;
            if (k == 1) info.projErrBefore = h.norm();
            if (h.norm() < 1e-3) break;
            const MatX Je = curve_planning::curveEndJacobian(rep, C);
            const MatX WJt = w.asDiagonal() * Je.transpose();
            const Eigen::Matrix2d A = Je * WJt;
            const VecX dv = -WJt * A.ldlt().solve(h);
            v = (v + dv).cwiseMax(rep.lb).cwiseMin(rep.ub);
            info.projSteps = k;
        }
        const CurveSamples C = curve_planning::evalParametricCurve(v, rep, Md);
        info.projErrAfter = (C.endPt - *rep.pGoal).norm();
        return v;
    }
    // BSpline: length only (the endpoints are pinned control points)
    for (int k = 1; k <= 20; ++k) {
        VecX cin, ceq;
        MatX gceq;
        constraintCurvatureLength(v, rep, cin, ceq, nullptr, &gceq);
        if (k == 1) info.projErrBefore = std::abs(ceq(0)) * rep.L;
        if (std::abs(ceq(0)) < 2e-4) break;
        VecX g = gceq.col(0);
        for (Index i = 0; i < rep.nv; ++i)
            if (rep.lb(i) == rep.ub(i)) g(i) = 0.0;
        v -= g * (ceq(0) / g.squaredNorm());
        info.projSteps = k;
    }
    VecX cin, ceq;
    constraintCurvatureLength(v, rep, cin, ceq);
    info.projErrAfter = std::abs(ceq(0)) * rep.L;
    return v;
}

}  // namespace

VecX optimizeAgentCurve(const VecX& v0in, const CurveRep& rep, const AgentProblem& prob,
                        const AgentProblem* explore, const OptimizerParams& opts, int exploreIter,
                        double denseStep, OptimizeInfo* infoOut) {
    const auto t0 = std::chrono::steady_clock::now();
    OptimizeInfo info;
    VecX v0 = v0in.cwiseMax(rep.lb).cwiseMin(rep.ub);
    info.J0 = objectiveResidualBelief(v0, rep, prob);

    if (explore && exploreIter > 0) {
        // graduated optimisation: the optimistic long-tailed kernel first
        OptimizerParams ox = opts;
        ox.verbose = false;
        ox.maxIter = exploreIter;
        v0 = optimizeAgentCurve(v0, rep, *explore, nullptr, ox, 0, denseStep, nullptr);
    }

    LbfgsOut st;
    VecX v = internalAL(v0, rep, prob, opts, opts.maxIter, st);
    info.iters = st.iters;
    info.fevals = st.fevals + 1;
    info.exitMsg = st.msg;

    v = projectFeasible(v, rep, denseStep, info);
    info.J = objectiveResidualBelief(v, rep, prob);
    VecX cin, ceq;
    constraintCurvatureLength(v, rep, cin, ceq);
    info.ceqMax = ceq.size() ? ceq.cwiseAbs().maxCoeff() : 0.0;
    info.cinMax = cin.size() ? std::max(0.0, cin.maxCoeff()) : 0.0;
    info.timeSec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (opts.verbose) {
        std::fprintf(stderr,
                     "    optimise (%s): J %.5f -> %.5f in %d it, %d evals, %.2f s  |ceq| %.1e  max cin %.1e\n",
                     toString(rep.type), info.J0, info.J, info.iters, info.fevals, info.timeSec,
                     info.ceqMax, info.cinMax);
    }
    if (infoOut) *infoOut = info;
    return v;
}

}  // namespace mtl::curve::optimization
