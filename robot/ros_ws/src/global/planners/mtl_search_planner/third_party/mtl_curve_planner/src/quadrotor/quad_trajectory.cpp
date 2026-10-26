#include "mtl_curve/quadrotor/quad_trajectory.hpp"

#include <Eigen/SparseCholesky>
#include <Eigen/SparseCore>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "mtl_curve/core/numeric.hpp"
#include "mtl_curve/quadrotor/flatness.hpp"

namespace mtl::curve::quadrotor {

using SpMat = Eigen::SparseMatrix<double>;
using Trip = Eigen::Triplet<double>;

// =============================================================================
//  reference
// =============================================================================
QuadReference quadReference(const Path3& drone, const Path2& sensor, Index nSteps, double Vq) {
    const Index n = std::max<Index>(2, std::min({nSteps, drone.rows(), sensor.rows()}));
    // keep strictly increasing arc length (drop padding / hover repeats)
    std::vector<Index> keep{0};
    std::vector<double> sv{0.0};
    double s = 0.0;
    for (Index k = 1; k < n; ++k) {
        const double seg = (drone.row(k).head<2>() - drone.row(k - 1).head<2>()).norm();
        s += seg;
        if (seg > 1e-9) {
            keep.push_back(k);
            sv.push_back(s);
        }
    }
    const Index m = static_cast<Index>(keep.size());
    QuadReference R;
    R.s.resize(m);
    R.P.resize(m, 2);
    R.B.resize(m, 2);
    for (Index i = 0; i < m; ++i) {
        const Index k = keep[static_cast<std::size_t>(i)];
        R.s(i) = sv[static_cast<std::size_t>(i)];
        R.P.row(i) = drone.row(k).head<2>();
        R.B.row(i) = sensor.row(k);
    }
    VecX psi(m);
    for (Index i = 0; i < m; ++i) {
        Vec2 d;
        if (m < 2) d = Vec2(1.0, 0.0);
        else if (i == 0) d = (R.P.row(1) - R.P.row(0)).transpose();
        else if (i == m - 1) d = (R.P.row(m - 1) - R.P.row(m - 2)).transpose();
        else d = (R.P.row(i + 1) - R.P.row(i - 1)).transpose();
        psi(i) = std::atan2(d.y(), d.x());
    }
    R.psi = core::unwrap(psi);
    R.h = drone(0, 2);
    R.Vq = Vq;
    R.T = R.s(m - 1) / Vq;
    return R;
}

QuadReferenceSample sampleReference(const QuadReference& ref, const VecX& t) {
    const Index K = t.size();
    VecX sq(K);
    for (Index k = 0; k < K; ++k) sq(k) = std::min(std::max(ref.Vq * t(k), 0.0), ref.s(ref.s.size() - 1));
    QuadReferenceSample R;
    R.s = sq;
    R.P.resize(K, 2);
    R.B.resize(K, 2);
    R.tan.resize(K, 2);
    R.P.col(0) = core::interp1(ref.s, ref.P.col(0), sq);
    R.P.col(1) = core::interp1(ref.s, ref.P.col(1), sq);
    R.B.col(0) = core::interp1(ref.s, ref.B.col(0), sq);
    R.B.col(1) = core::interp1(ref.s, ref.B.col(1), sq);
    R.psi = core::interp1(ref.s, ref.psi, sq);
    R.bearing.resize(K);
    for (Index k = 0; k < K; ++k) {
        R.tan(k, 0) = std::cos(R.psi(k));
        R.tan(k, 1) = std::sin(R.psi(k));
        const double br = std::atan2(R.B(k, 1) - R.P(k, 1), R.B(k, 0) - R.P(k, 0));
        R.bearing(k) = R.psi(k) + std::atan2(std::sin(br - R.psi(k)), std::cos(br - R.psi(k)));
    }
    return R;
}

// =============================================================================
//  the optimisation
// =============================================================================
namespace {

struct Limits {
    double tilt, rate, yawRate, yawAcc, speed, look;
};
struct Weights {
    double tilt = 1, rate = 1, yawRate = 1, yawAcc = 1, speed = 1, look = 1;
};

/// Everything fixed during one agent's solve.
struct Ctx {
    const QuadParams* q = nullptr;
    UniformBspline B;
    Index n = 0, K = 0, nv = 0;
    double h = 0.0;
    QuadReferenceSample R;
    Path2 nrm;
    Limits lim{};
    double m = 0.97;
    VecX cpsiFixed;
};

void unpack(const VecX& x, const Ctx& c, VecX& cx, VecX& cy, VecX& cpsi) {
    cx = x.segment(0, c.n);
    cy = x.segment(c.n, c.n);
    cpsi = c.q->useYaw ? VecX(x.segment(2 * c.n, c.n)) : c.cpsiFixed;
}

/// Row-wise sparse assembly: a residual row at sample k is a combination of
/// the B-spline derivative rows of the x, y and psi coefficients.
struct Assembler {
    const Ctx& c;
    bool wantJ;
    std::vector<double> r;
    std::vector<Trip> trips;
    Index row = 0;
    Assembler(const Ctx& ctx, bool wj) : c(ctx), wantJ(wj) {
        r.reserve(static_cast<std::size_t>(16 * ctx.K));
        if (wj) trips.reserve(static_cast<std::size_t>(16 * ctx.K * 3 * (ctx.B.p + 1)));
    }
    /// cX[d], cY[d], cP[d]: coefficient of the d-th derivative row of each block
    void add(Index k, double value, const std::array<double, 5>& cX, const std::array<double, 5>& cY,
             const std::array<double, 5>& cP) {
        r.push_back(value);
        if (wantJ) {
            const Index m0 = c.B.m[static_cast<std::size_t>(k)];
            const int p = c.B.p;
            const int nd = static_cast<int>(c.B.b.size());
            const bool yaw = c.q->useYaw;
            for (int blk = 0; blk < (yaw ? 3 : 2); ++blk) {
                const std::array<double, 5>& cc = blk == 0 ? cX : (blk == 1 ? cY : cP);
                bool any = false;
                for (int d = 0; d < nd; ++d) any = any || cc[static_cast<std::size_t>(d)] != 0.0;
                if (!any) continue;
                for (int i = 0; i <= p; ++i) {
                    double v = 0.0;
                    for (int d = 0; d < nd; ++d) {
                        const double w = cc[static_cast<std::size_t>(d)];
                        if (w != 0.0) v += w * c.B.b[static_cast<std::size_t>(d)](k, i);
                    }
                    trips.emplace_back(row, static_cast<Index>(blk) * c.n + m0 + i, v);
                }
            }
        }
        ++row;
    }
};

constexpr std::array<double, 5> Z5{0, 0, 0, 0, 0};
std::array<double, 5> at(int d, double v) {
    std::array<double, 5> a = Z5;
    a[static_cast<std::size_t>(d)] = v;
    return a;
}

/// Stacked residuals and (optionally) the sparse Jacobian.
void residuals(const VecX& x, const Ctx& c, const Weights& w, VecX& rOut, SpMat* Jout) {
    const QuadParams& q = *c.q;
    const bool wantJ = Jout != nullptr;
    VecX cx, cy, cpsi;
    unpack(x, c, cx, cy, cpsi);
    const Index K = c.K;
    const double h = c.h;
    const QuadReferenceSample& R = c.R;
    const bool yaw = q.useYaw;
    Assembler as(c, wantJ);

    for (Index k = 0; k < K; ++k) {
        const Vec2 P(c.B.eval(0, k, cx), c.B.eval(0, k, cy));
        const Vec2 V(c.B.eval(1, k, cx), c.B.eval(1, k, cy));
        const Vec2 A(c.B.eval(2, k, cx), c.B.eval(2, k, cy));
        const Vec2 Jk(c.B.eval(3, k, cx), c.B.eval(3, k, cy));
        const double psi = c.B.eval(0, k, cpsi), psid = c.B.eval(1, k, cpsi), psidd = c.B.eval(2, k, cpsi);
        const FlatState L = flatMap(V, A, Jk, psi, 0.0, h, q, FlatMode::Look);

        // per-sample derivatives of (bore, tilt, look) w.r.t. (vx, vy, ax, ay, psi)
        std::array<Vec2, 5> dB{};
        std::array<double, 5> dT{}, dL{};
        if (wantJ) {
            for (int i = 0; i < 5; ++i) {
                Vec2 Vp = V, Vm = V, Ap = A, Am = A;
                double pp = psi, pm = psi, ep;
                if (i < 2) {
                    ep = 1e-6 * std::max(1.0, std::abs(V(i)));
                    Vp(i) += ep; Vm(i) -= ep;
                } else if (i < 4) {
                    ep = 1e-6 * std::max(1.0, std::abs(A(i - 2)));
                    Ap(i - 2) += ep; Am(i - 2) -= ep;
                } else {
                    ep = 1e-7;
                    pp += ep; pm -= ep;
                }
                const FlatState Lp = flatMap(Vp, Ap, Jk, pp, 0.0, h, q, FlatMode::Look);
                const FlatState Lm = flatMap(Vm, Am, Jk, pm, 0.0, h, q, FlatMode::Look);
                dB[static_cast<std::size_t>(i)] = (Lp.bore - Lm.bore) / (2.0 * ep);
                dT[static_cast<std::size_t>(i)] = (Lp.tilt - Lm.tilt) / (2.0 * ep);
                dL[static_cast<std::size_t>(i)] = (Lp.look - Lm.look) / (2.0 * ep);
            }
        }

        // boresight tracking
        for (int cc = 0; cc < 2; ++cc) {
            const double sb = q.sigmaBore;
            const double val = (P(cc) + L.bore(cc) - R.B(k, cc)) / sb;
            std::array<double, 5> cX = Z5, cY = Z5, cP = Z5;
            if (wantJ) {
                cX = {cc == 0 ? 1.0 / sb : 0.0, dB[0](cc) / sb, dB[2](cc) / sb, 0, 0};
                cY = {cc == 1 ? 1.0 / sb : 0.0, dB[1](cc) / sb, dB[3](cc) / sb, 0, 0};
                cP = {dB[4](cc) / sb, 0, 0, 0, 0};
            }
            as.add(k, val, cX, cY, cP);
        }
        // path tracking (cross / along the reference tangent)
        const Vec2 e = P - R.P.row(k).transpose();
        const Vec2 nr = c.nrm.row(k).transpose(), tg = R.tan.row(k).transpose();
        as.add(k, e.dot(nr) / q.sigmaCross, at(0, nr.x() / q.sigmaCross), at(0, nr.y() / q.sigmaCross), Z5);
        as.add(k, e.dot(tg) / q.sigmaAlong, at(0, tg.x() / q.sigmaAlong), at(0, tg.y() / q.sigmaAlong), Z5);
        // smoothness
        as.add(k, c.B.eval(4, k, cx) / q.sigmaSnap, at(4, 1.0 / q.sigmaSnap), Z5, Z5);
        as.add(k, c.B.eval(4, k, cy) / q.sigmaSnap, Z5, at(4, 1.0 / q.sigmaSnap), Z5);
        if (yaw) {
            as.add(k, (psi - R.psi(k)) / q.sigmaYawReg, Z5, Z5, at(0, 1.0 / q.sigmaYawReg));
            as.add(k, psidd / q.sigmaYawAcc, Z5, Z5, at(2, 1.0 / q.sigmaYawAcc));
        }
        // start state
        if (k == 0) {
            const Vec2 v0 = q.speed * R.tan.row(0).transpose();
            as.add(0, (P.x() - R.P(0, 0)) / q.sigmaStartPos, at(0, 1.0 / q.sigmaStartPos), Z5, Z5);
            as.add(0, (P.y() - R.P(0, 1)) / q.sigmaStartPos, Z5, at(0, 1.0 / q.sigmaStartPos), Z5);
            as.add(0, (V.x() - v0.x()) / q.sigmaStartVel, at(1, 1.0 / q.sigmaStartVel), Z5, Z5);
            as.add(0, (V.y() - v0.y()) / q.sigmaStartVel, Z5, at(1, 1.0 / q.sigmaStartVel), Z5);
        }

        // --- hinges --------------------------------------------------------
        {   // tilt
            const double gv = L.tilt - c.m * c.lim.tilt;
            if (gv > 0.0) {
                const double s = w.tilt / q.hingeScaleAngle;
                as.add(k, s * gv, {0, s * dT[0], s * dT[2], 0, 0}, {0, s * dT[1], s * dT[3], 0, 0}, Z5);
            }
        }
        {   // look
            const double gv = L.look - (c.lim.look - q.lookPlanMargin);
            if (gv > 0.0) {
                const double s = w.look / q.hingeScaleAngle;
                as.add(k, s * gv, {0, s * dL[0], s * dL[2], 0, 0}, {0, s * dL[1], s * dL[3], 0, 0},
                       {s * dL[4], 0, 0, 0, 0});
            }
        }
        {   // roll/pitch rate |(p, q)|
            const double rxy = flatMap(V, A, Jk, 0.0, 0.0, h, q, FlatMode::Rate).rateXY;
            const double gv = rxy - c.m * c.lim.rate;
            if (gv > 0.0) {
                const double s = w.rate / q.hingeScaleRate;
                std::array<double, 6> dR{};
                if (wantJ) {
                    for (int i = 0; i < 6; ++i) {
                        Vec2 Vp = V, Vm = V, Ap = A, Am = A, Jp = Jk, Jm = Jk;
                        const int col = i % 2;
                        double ep;
                        switch (i / 2) {
                            case 0: ep = 1e-6 * std::max(1.0, std::abs(V(col))); Vp(col) += ep; Vm(col) -= ep; break;
                            case 1: ep = 1e-6 * std::max(1.0, std::abs(A(col))); Ap(col) += ep; Am(col) -= ep; break;
                            default: ep = 1e-6 * std::max(1.0, std::abs(Jk(col))); Jp(col) += ep; Jm(col) -= ep; break;
                        }
                        dR[static_cast<std::size_t>(i)] = (flatMap(Vp, Ap, Jp, 0.0, 0.0, h, q, FlatMode::Rate).rateXY -
                                                           flatMap(Vm, Am, Jm, 0.0, 0.0, h, q, FlatMode::Rate).rateXY) /
                                                          (2.0 * ep);
                    }
                }
                as.add(k, s * gv, {0, s * dR[0], s * dR[2], s * dR[4], 0}, {0, s * dR[1], s * dR[3], s * dR[5], 0}, Z5);
            }
        }
        {   // speed
            const double sp = std::max(V.norm(), 1e-9);
            const double gv = sp - c.m * c.lim.speed;
            if (gv > 0.0) {
                const double s = w.speed / q.hingeScaleSpeed;
                as.add(k, s * gv, at(1, s * V.x() / sp), at(1, s * V.y() / sp), Z5);
            }
        }
        if (yaw) {
            const double g1 = std::abs(psid) - c.m * c.lim.yawRate;
            if (g1 > 0.0) {
                const double s = w.yawRate / q.hingeScaleRate;
                as.add(k, s * g1, Z5, Z5, at(1, s * (psid > 0 ? 1.0 : -1.0)));
            }
            const double g2 = std::abs(psidd) - c.m * c.lim.yawAcc;
            if (g2 > 0.0) {
                const double s = w.yawAcc / q.hingeScaleRate;
                as.add(k, s * g2, Z5, Z5, at(2, s * (psidd > 0 ? 1.0 : -1.0)));
            }
        }
    }
    rOut = Eigen::Map<const VecX>(as.r.data(), static_cast<Index>(as.r.size()));
    if (wantJ) {
        Jout->resize(as.row, c.nv);
        Jout->setFromTriplets(as.trips.begin(), as.trips.end());
    }
}

int levenbergMarquardt(VecX& x, const Ctx& c, const Weights& w, const QuadParams& q, std::vector<double>& costHist) {
    VecX r;
    SpMat J;
    residuals(x, c, w, r, &J);
    double cost = 0.5 * r.squaredNorm();
    costHist.push_back(cost);
    double lambda = q.lambda0;
    int small = 0, it = 0;
    Eigen::SimplicialLDLT<SpMat> solver;
    for (it = 1; it <= q.maxIter; ++it) {
        const SpMat H = SpMat(J.transpose()) * J;
        const VecX g = J.transpose() * r;
        const VecX dH = H.diagonal();
        const double dHmax = dH.maxCoeff();
        bool accepted = false;
        VecX xn;
        double cn = 0.0;
        for (int tries = 0; tries < 12; ++tries) {
            SpMat Ad = H;
            for (Index i = 0; i < c.nv; ++i) Ad.coeffRef(i, i) += lambda * (dH(i) + 1e-9 * dHmax) + 1e-12;
            solver.compute(Ad);
            if (solver.info() == Eigen::Success) {
                const VecX dx = -solver.solve(g);
                xn = x + dx;
                VecX rn;
                residuals(xn, c, w, rn, nullptr);
                cn = 0.5 * rn.squaredNorm();
                if (std::isfinite(cn) && cn < cost) {
                    accepted = true;
                    break;
                }
            }
            lambda *= 4.0;
        }
        if (!accepted) break;
        const double rel = (cost - cn) / std::max(cost, 1e-300);
        x = xn;
        cost = cn;
        lambda = std::max(lambda / 3.0, 1e-9);
        residuals(x, c, w, r, &J);
        costHist.push_back(cost);
        if (rel < q.tolFun) {
            if (++small >= 2) break;
        } else {
            small = 0;
        }
    }
    return std::min(it, q.maxIter);
}

VecX uniformGrid(double T, double step, bool appendEnd) {
    const Index n = static_cast<Index>(std::floor(T / step + 1e-9)) + 1;
    VecX t(n);
    for (Index i = 0; i < n; ++i) t(i) = static_cast<double>(i) * step;
    if (appendEnd && T - t(n - 1) > 1e-9) {
        t.conservativeResize(n + 1);
        t(n) = T;
    }
    return t;
}

/// Everything the solution implies on a time grid.
struct Sampled {
    Path2 P, V, A, J;
    VecX psi, yawRate, yawAcc, speed;
    std::vector<FlatState> S;
};

Sampled sampleSolution(const VecX& x, const Ctx& c, const VecX& t) {
    const UniformBspline B = uniformBspline(t, c.B.delta, c.B.T, c.B.p, 3);
    VecX cx, cy, cpsi;
    unpack(x, c, cx, cy, cpsi);
    Sampled O;
    const Index K = t.size();
    O.P.resize(K, 2); O.V.resize(K, 2); O.A.resize(K, 2); O.J.resize(K, 2);
    O.psi = B.eval(0, cpsi);
    O.yawRate = B.eval(1, cpsi);
    O.yawAcc = B.eval(2, cpsi);
    O.speed.resize(K);
    O.S.resize(static_cast<std::size_t>(K));
    for (Index k = 0; k < K; ++k) {
        O.P(k, 0) = B.eval(0, k, cx); O.P(k, 1) = B.eval(0, k, cy);
        O.V(k, 0) = B.eval(1, k, cx); O.V(k, 1) = B.eval(1, k, cy);
        O.A(k, 0) = B.eval(2, k, cx); O.A(k, 1) = B.eval(2, k, cy);
        O.J(k, 0) = B.eval(3, k, cx); O.J(k, 1) = B.eval(3, k, cy);
        O.speed(k) = O.V.row(k).norm();
        O.S[static_cast<std::size_t>(k)] = flatMap(O.V.row(k).transpose(), O.A.row(k).transpose(),
                                                   O.J.row(k).transpose(), O.psi(k), O.yawRate(k), c.h, *c.q);
    }
    return O;
}

SpMat sparseBasis(const UniformBspline& B, int d) {
    std::vector<Trip> tr;
    tr.reserve(static_cast<std::size_t>(B.samples() * (B.p + 1)));
    for (Index k = 0; k < B.samples(); ++k)
        for (int i = 0; i <= B.p; ++i)
            tr.emplace_back(k, B.m[static_cast<std::size_t>(k)] + i, B.b[static_cast<std::size_t>(d)](k, i));
    SpMat M(B.samples(), B.nCP);
    M.setFromTriplets(tr.begin(), tr.end());
    return M;
}

VecX regularisedFit(const UniformBspline& B, int dReg, double mu, const VecX& y) {
    const SpMat B0 = sparseBasis(B, 0), Br = sparseBasis(B, dReg);
    SpMat A = SpMat(B0.transpose()) * B0 + mu * (SpMat(Br.transpose()) * Br);
    for (Index i = 0; i < A.rows(); ++i) A.coeffRef(i, i) += 1e-9;
    Eigen::SimplicialLDLT<SpMat> s(A);
    if (s.info() != Eigen::Success) throw std::runtime_error("quadrotor: spline fit failed");
    return s.solve(VecX(B0.transpose() * y));
}

double maxAbs(const VecX& v) { return v.size() ? v.cwiseAbs().maxCoeff() : 0.0; }

}  // namespace

QuadTrajectory optimizeQuadFlatTrajectory(const QuadReference& ref, double beta, const QuadParams& q, double dtOut) {
    if (!q.resolved) throw std::invalid_argument("optimizeQuadFlatTrajectory: resolve the QuadParams first");
    const auto t0 = std::chrono::steady_clock::now();
    const double h = ref.h, T = ref.T;
    const int p = q.splineDegree;
    const double Dl = q.knotSpacing;

    Ctx c;
    c.q = &q;
    c.h = h;
    c.m = q.limitMargin;
    const double lookMax = std::min(q.maxLookAngle, std::acos(std::min(h / (q.lookRangeMargin * beta), 1.0)));
    c.lim = {q.maxTilt, q.maxRollPitchRate, q.maxYawRate, q.maxYawAcc, q.maxSpeed, lookMax};

    const VecX tk = uniformGrid(T, q.dtOpt, true);
    c.K = tk.size();
    c.B = uniformBspline(tk, Dl, T, p, 4);
    c.n = c.B.nCP;
    c.R = sampleReference(ref, tk);
    c.nrm.resize(c.K, 2);
    c.nrm.col(0) = -c.R.tan.col(1);
    c.nrm.col(1) = c.R.tan.col(0);

    // initial guess: the curve itself, yaw toward the reference ground point
    const double mu = 1e-6;
    const VecX cx = regularisedFit(c.B, 4, mu, c.R.P.col(0));
    const VecX cy = regularisedFit(c.B, 4, mu, c.R.P.col(1));
    const VecX cpsi = regularisedFit(c.B, 2, mu, q.useYaw ? c.R.bearing : c.R.psi);
    c.cpsiFixed = cpsi;
    c.nv = (q.useYaw ? 3 : 2) * c.n;
    VecX x(c.nv);
    x.segment(0, c.n) = cx;
    x.segment(c.n, c.n) = cy;
    if (q.useYaw) x.segment(2 * c.n, c.n) = cpsi;

    Weights w;
    std::vector<double> costHist;
    int totalIter = 0;
    bool feasible = false;
    const VecX tChk = uniformGrid(T, std::min(q.dtOpt, 0.1), true);
    for (int rnd = 0; rnd <= q.penaltyRounds; ++rnd) {
        totalIter += levenbergMarquardt(x, c, w, q, costHist);
        const Sampled O = sampleSolution(x, c, tChk);
        double mt = 0, mr = 0, ml = 0;
        for (const FlatState& s : O.S) {
            mt = std::max(mt, s.tilt);
            mr = std::max(mr, s.rateXY);
            ml = std::max(ml, s.look);
        }
        bool anyV = false;
        auto esc = [&](bool v, double& wt) {
            if (v) {
                wt *= q.penaltyGrowth;
                anyV = true;
            }
        };
        esc(mt > c.lim.tilt, w.tilt);
        esc(mr > c.lim.rate, w.rate);
        esc(q.useYaw && maxAbs(O.yawRate) > c.lim.yawRate, w.yawRate);
        esc(q.useYaw && maxAbs(O.yawAcc) > c.lim.yawAcc, w.yawAcc);
        esc(O.speed.maxCoeff() > c.lim.speed, w.speed);
        esc(ml > c.lim.look, w.look);
        if (q.verbose)
            std::fprintf(stderr,
                         "    quad LM round %d: cost %.4g, tilt %.2f / rate %.1f / yawRate %.1f deg(/s), look %.1f deg%s\n",
                         rnd, costHist.back(), rad2deg(mt), rad2deg(mr), rad2deg(maxAbs(O.yawRate)), rad2deg(ml),
                         anyV ? "  -> escalate" : "");
        if (!anyV) {
            feasible = true;
            break;
        }
    }

    // --- output grid ------------------------------------------------------------
    const VecX tOut = uniformGrid(T, dtOut, false);
    const Sampled O = sampleSolution(x, c, tOut);
    const QuadReferenceSample Ro = sampleReference(ref, tOut);
    const Index N = tOut.size();

    QuadTrajectory Q;
    Q.t = tOut;
    Q.altitude = h;
    Q.vel = Path3::Zero(N, 3); Q.acc = Path3::Zero(N, 3); Q.jerk = Path3::Zero(N, 3);
    Q.vel.leftCols(2) = O.V; Q.acc.leftCols(2) = O.A; Q.jerk.leftCols(2) = O.J;
    Q.yaw = O.psi;
    Q.yawRate = O.yawRate;
    Q.rates.resize(N, 3);
    Q.thrust.resize(N); Q.thrustToWeight.resize(N); Q.tilt.resize(N); Q.look.resize(N);
    Q.boreErr.resize(N); Q.crossErr.resize(N); Q.alongErr.resize(N);
    Q.boreRef = Ro.B;
    Q.pathRef = Ro.P;
    Q.trackHeading = Ro.psi;
    AgentTrajectory& A = Q.traj;
    A.drone.resize(N, 3);
    A.sensor.resize(N, 2);
    A.rpy.resize(N, 3);
    A.gimbalAngle = VecX::Zero(N);
    A.gimbalCmd = VecX::Zero(N);
    double maxRoll = 0, maxPitch = 0, maxRxy = 0, len = 0, pathSq = 0;
    for (Index k = 0; k < N; ++k) {
        const FlatState& s = O.S[static_cast<std::size_t>(k)];
        A.drone(k, 0) = O.P(k, 0);
        A.drone(k, 1) = O.P(k, 1);
        A.drone(k, 2) = h;
        A.sensor.row(k) = O.P.row(k) + s.bore.transpose();
        A.rpy.row(k) = s.rpy.transpose();
        Q.rates.row(k) = s.rates.transpose();
        Q.thrust(k) = s.f;
        Q.thrustToWeight(k) = s.f / q.gravity;
        Q.tilt(k) = s.tilt;
        Q.look(k) = s.look;
        Q.boreErr(k) = (A.sensor.row(k) - Ro.B.row(k)).norm();
        const Vec2 e = (O.P.row(k) - Ro.P.row(k)).transpose();
        const Vec2 tg = Ro.tan.row(k).transpose();
        Q.crossErr(k) = e.dot(Vec2(-tg.y(), tg.x()));
        Q.alongErr(k) = e.dot(tg);
        pathSq += e.squaredNorm();
        maxRoll = std::max(maxRoll, std::abs(s.rpy(0)));
        maxPitch = std::max(maxPitch, std::abs(s.rpy(1)));
        maxRxy = std::max(maxRxy, s.rateXY);
        if (k > 0) len += (O.P.row(k) - O.P.row(k - 1)).norm();
    }
    A.steps = N;
    A.length = len;
    A.maxRollDeg = rad2deg(maxRoll);
    A.startPt = O.P.row(0).transpose();
    A.endPt = O.P.row(N - 1).transpose();

    VecX cxo, cyo, cpsio;
    unpack(x, c, cxo, cyo, cpsio);
    Q.cx = cxo; Q.cy = cyo; Q.cpsi = cpsio;
    Q.knotSpacing = Dl; Q.horizon = T; Q.degree = p;

    QuadStats& st = Q.stats;
    std::vector<double> be(Q.boreErr.data(), Q.boreErr.data() + N);
    st.boreRms = std::sqrt(Q.boreErr.squaredNorm() / static_cast<double>(N));
    st.boreP95 = core::quantile(be, 0.95);
    st.boreMax = Q.boreErr.maxCoeff();
    st.pathRms = std::sqrt(pathSq / static_cast<double>(N));
    st.crossMax = maxAbs(Q.crossErr);
    st.alongMax = maxAbs(Q.alongErr);
    st.maxTiltDeg = rad2deg(Q.tilt.maxCoeff());
    st.maxRollDeg = rad2deg(maxRoll);
    st.maxPitchDeg = rad2deg(maxPitch);
    st.maxRateXYDegS = rad2deg(maxRxy);
    st.maxYawRateDegS = rad2deg(maxAbs(O.yawRate));
    st.maxYawAccDegS2 = rad2deg(maxAbs(O.yawAcc));
    st.maxSpeed = O.speed.maxCoeff();
    st.minSpeed = O.speed.minCoeff();
    st.maxLookDeg = rad2deg(Q.look.maxCoeff());
    st.lookLimitDeg = rad2deg(lookMax);
    st.maxThrustToWeight = Q.thrustToWeight.maxCoeff();
    st.thrustOK = st.maxThrustToWeight <= q.thrustToWeight && Q.thrustToWeight.minCoeff() >= q.minThrustFrac;
    st.feasible = feasible && st.thrustOK;
    st.iterations = totalIter;
    st.costHist = std::move(costHist);
    st.nCoef = c.nv;
    st.length = len;
    st.solveTime = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    return Q;
}

// =============================================================================
//  team
// =============================================================================
namespace {

template <typename M>
void padRows(M& m, Index rows) {
    const Index n = m.rows();
    if (n >= rows || n == 0) return;
    M p(rows, m.cols());
    p.topRows(n) = m;
    p.bottomRows(rows - n).rowwise() = m.row(n - 1);
    m = std::move(p);
}

void padVec(VecX& v, Index rows) {
    const Index n = v.size();
    if (n >= rows || n == 0) return;
    VecX p(rows);
    p.head(n) = v;
    p.tail(rows - n).setConstant(v(n - 1));
    v = std::move(p);
}

}  // namespace

std::vector<QuadTrajectory> generateQuadTrajectories(const std::vector<AgentTrajectory>& fixedWing, double Vfw,
                                                     double dt, double fwTiltAngle, double beta,
                                                     const QuadParams& params, VecX& timeVec) {
    (void)Vfw;
    const QuadParams q = resolveQuadParams(params, fwTiltAngle);
    const double dtOut = q.dtOut ? *q.dtOut : dt;
    std::vector<QuadTrajectory> out;
    out.reserve(fixedWing.size());
    Index maxSteps = 1;
    for (std::size_t a = 0; a < fixedWing.size(); ++a) {
        const AgentTrajectory& fw = fixedWing[a];
        const Index steps = fw.steps > 0 ? fw.steps : fw.drone.rows();
        const QuadReference ref = quadReference(fw.drone, fw.sensor, steps, q.speed);
        out.push_back(optimizeQuadFlatTrajectory(ref, beta, q, dtOut));
        const QuadTrajectory& Q = out.back();
        maxSteps = std::max(maxSteps, Q.t.size());
        if (q.verbose)
            std::fprintf(stderr,
                         "  quad %zu: T %.0f s, boresight err rms %.1f m (p95 %.1f, max %.1f), path rms %.1f m, "
                         "tilt %.1f deg, %s (%.2f s)\n",
                         a + 1, Q.t(Q.t.size() - 1), Q.stats.boreRms, Q.stats.boreP95, Q.stats.boreMax,
                         Q.stats.pathRms, Q.stats.maxTiltDeg, Q.stats.feasible ? "feasible" : "LIMITS VIOLATED",
                         Q.stats.solveTime);
    }
    for (QuadTrajectory& Q : out) {
        padRows(Q.traj.drone, maxSteps);
        padRows(Q.traj.sensor, maxSteps);
        padRows(Q.traj.rpy, maxSteps);
        padVec(Q.traj.gimbalAngle, maxSteps);
        padVec(Q.traj.gimbalCmd, maxSteps);
    }
    timeVec.resize(maxSteps);
    for (Index i = 0; i < maxSteps; ++i) timeVec(i) = static_cast<double>(i) * dtOut;
    return out;
}

std::vector<AgentTrajectory> quadAgentTrajectories(const std::vector<QuadTrajectory>& quad) {
    std::vector<AgentTrajectory> out;
    out.reserve(quad.size());
    for (const QuadTrajectory& Q : quad) out.push_back(Q.traj);
    return out;
}

std::vector<AgentTrajectory> quadMatchedLooks(const std::vector<QuadTrajectory>& quad,
                                              const std::vector<AgentTrajectory>& fixedWing, double Vfw,
                                              double dt, double Vq, VecX& timeVec) {
    std::vector<AgentTrajectory> out(quad.size());
    Index maxSteps = 1;
    for (std::size_t a = 0; a < quad.size(); ++a) {
        const QuadTrajectory& Q = quad[a];
        const Index steps = fixedWing[a].steps > 0 ? fixedWing[a].steps : fixedWing[a].drone.rows();
        const Index nQ = Q.t.size();
        VecX tk(steps);
        for (Index k = 0; k < steps; ++k)
            tk(k) = std::min(static_cast<double>(k) * Vfw * dt / Vq, Q.t(nQ - 1));
        AgentTrajectory& A = out[a];
        A.drone.resize(steps, 3);
        A.sensor.resize(steps, 2);
        A.rpy.resize(steps, 3);
        for (int cc = 0; cc < 3; ++cc) {
            A.drone.col(cc) = core::interp1(Q.t, VecX(Q.traj.drone.col(cc).head(nQ)), tk);
            A.rpy.col(cc) = core::interp1(Q.t, VecX(Q.traj.rpy.col(cc).head(nQ)), tk);
        }
        for (int cc = 0; cc < 2; ++cc) A.sensor.col(cc) = core::interp1(Q.t, VecX(Q.traj.sensor.col(cc).head(nQ)), tk);
        A.gimbalAngle = VecX::Zero(steps);
        A.gimbalCmd = VecX::Zero(steps);
        A.steps = steps;
        maxSteps = std::max(maxSteps, steps);
    }
    for (AgentTrajectory& A : out) {
        padRows(A.drone, maxSteps);
        padRows(A.sensor, maxSteps);
        padRows(A.rpy, maxSteps);
        padVec(A.gimbalAngle, maxSteps);
        padVec(A.gimbalCmd, maxSteps);
    }
    timeVec.resize(maxSteps);
    for (Index i = 0; i < maxSteps; ++i) timeVec(i) = static_cast<double>(i) * dt;
    return out;
}

}  // namespace mtl::curve::quadrotor
