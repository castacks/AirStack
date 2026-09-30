#include "mtl_curve/optimization/swath_kernel.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace mtl::curve::optimization {
namespace {

double pDetect(double d3, const SensorModelParams& s, double pOut) {
    if (d3 > s.beta) return pOut;
    return 1.0 / (s.a + std::exp(s.b * (d3 - s.c)));
}

/// linear interpolation with a constant fill outside [x0, xn] (MATLAB
/// interp1(..., 'linear', fill)); x must be increasing and uniform-or-not.
double interpFill(const VecX& x, const VecX& y, double xq, double fill) {
    const Index n = x.size();
    if (n == 0 || xq < x(0) || xq > x(n - 1)) return fill;
    Index hi = static_cast<Index>(std::upper_bound(x.data(), x.data() + n, xq) - x.data());
    if (hi >= n) return y(n - 1);
    if (hi == 0) return y(0);
    const Index lo = hi - 1;
    const double f = (xq - x(lo)) / (x(hi) - x(lo));
    return (1.0 - f) * y(lo) + f * y(hi);
}

/// max(y_lo, y_hi) of the two samples bracketing xq (0 outside the range).
double conservativeAt(const VecX& x, const VecX& y, double xq) {
    const Index n = x.size();
    if (n == 0 || xq < x(0) || xq > x(n - 1)) return 0.0;
    Index hi = static_cast<Index>(std::upper_bound(x.data(), x.data() + n, xq) - x.data());
    if (hi >= n) return y(n - 1);
    if (hi == 0) return y(0);
    if (x(hi - 1) == xq) return y(hi - 1);
    return std::max(y(hi - 1), y(hi));
}

/// 'same' 1-D convolution of every column of A with the symmetric kernel g.
MatX convColsSame(const MatX& A, const VecX& g) {
    const Index n = A.rows(), m = g.size(), c = m / 2;
    MatX out = MatX::Zero(A.rows(), A.cols());
    for (Index j = 0; j < A.cols(); ++j)
        for (Index i = 0; i < n; ++i) {
            double acc = 0.0;
            for (Index k = 0; k < m; ++k) {
                const Index src = i + c - k;
                if (src >= 0 && src < n) acc += g(k) * A(src, j);
            }
            out(i, j) = acc;
        }
    return out;
}

/// Exact (hard-edged) straight pass, averaged along track in probability.
void straightProfile(double h, const SweepParams& sw, const SensorModelParams& sensor, double pOut,
                     double tanHalf, double V, double dt, const KernelParams& o, VecX& dOut,
                     VecX& lamEff) {
    const Index nT = static_cast<Index>(std::floor(o.trackLen / V / dt + 1e-9)) + 1;
    const double xLo = 0.35 * o.trackLen, xHi = 0.65 * o.trackLen;
    const double yMax = 1.2 * sensor.beta;
    const double gs = o.gridStep;
    const Index nx = static_cast<Index>(std::floor((xHi - xLo) / gs + 1e-9)) + 1;
    const Index ny = static_cast<Index>(std::floor(2.0 * yMax / gs + 1e-9)) + 1;
    MatX Lam = MatX::Zero(ny, nx);
    const double tau = sw.tiltAngle;
    for (Index q = 0; q < nT; ++q) {
        const double t = static_cast<double>(q) * dt;
        const double px = V * t;
        const double alpha = sw.alphaMax * std::sin(2.0 * kPi * sw.freq * t);
        const double lx = px + h * std::tan(tau);
        const double ly = h * std::tan(alpha) / std::cos(tau);
        const double R = std::sqrt((px - lx) * (px - lx) + ly * ly + h * h) * tanHalf;
        const Index c1 = std::max<Index>(0, static_cast<Index>(std::ceil((lx - R - xLo) / gs)));
        const Index c2 = std::min<Index>(nx - 1, static_cast<Index>(std::floor((lx + R - xLo) / gs)));
        const Index r1 = std::max<Index>(0, static_cast<Index>(std::ceil((ly - R + yMax) / gs)));
        const Index r2 = std::min<Index>(ny - 1, static_cast<Index>(std::floor((ly + R + yMax) / gs)));
        if (c1 > c2 || r1 > r2) continue;
        for (Index c = c1; c <= c2; ++c) {
            const double X = xLo + static_cast<double>(c) * gs;
            for (Index r = r1; r <= r2; ++r) {
                const double Y = -yMax + static_cast<double>(r) * gs;
                if ((Y - ly) * (Y - ly) + (X - lx) * (X - lx) > R * R) continue;
                const double d3 = std::sqrt(Y * Y + (X - px) * (X - px) + h * h);
                Lam(r, c) += std::log1p(-pDetect(d3, sensor, pOut));
            }
        }
    }
    VecX E = Lam.array().exp().rowwise().mean();
    VecX Es = E;
    for (Index r = 0; r < ny; ++r) Es(r) = 0.5 * (E(r) + E(ny - 1 - r));
    // rows with Y >= 0
    Index r0 = 0;
    while (r0 < ny && -yMax + static_cast<double>(r0) * gs < -1e-9) ++r0;
    dOut.resize(ny - r0);
    lamEff.resize(ny - r0);
    for (Index r = r0; r < ny; ++r) {
        dOut(r - r0) = -yMax + static_cast<double>(r) * gs;
        lamEff(r - r0) = std::log(std::max(Es(r), 1e-300));
    }
}

/// Bounding box (in metres, padded one cell) of the non-zero table entries.
void setSupportBox(SwathKernel& K) {
    Index c1 = K.T.cols(), c2 = -1, r1 = K.T.rows(), r2 = -1;
    for (Index c = 0; c < K.T.cols(); ++c)
        for (Index r = 0; r < K.T.rows(); ++r)
            if (K.T(r, c) != 0.0) {
                c1 = std::min(c1, c); c2 = std::max(c2, c);
                r1 = std::min(r1, r); r2 = std::max(r2, r);
            }
    if (c2 < 0) { K.aLo = K.aHi = K.dLo = K.dHi = 0.0; return; }
    K.aLo = K.a0 + static_cast<double>(c1 - 1) * K.step;
    K.aHi = K.a0 + static_cast<double>(c2 + 1) * K.step;
    K.dLo = K.d0 + static_cast<double>(r1 - 1) * K.step;
    K.dHi = K.d0 + static_cast<double>(r2 + 1) * K.step;
}

}  // namespace

SwathKernelSet calibrateSwathKernel(double h, const SweepParams& sweep, const SensorModelParams& sensor,
                                    double fov, double V, double dt, const KernelParams& o) {
    const double pOut = sensor.pOutOfRangeMulti;
    const double tanHalf = std::tan(fov / 2.0);
    const double reach = std::sqrt(std::max(sensor.beta * sensor.beta - h * h, 0.0));
    const double tau = sweep.tiltAngle;
    const double f0 = h * std::tan(tau);
    const double st = o.tableStep;

    // --- 1+2. aircraft-frame kernel with smooth, inset edges ------------------
    const double ext = std::ceil((reach + 3.0 * st) / st) * st;
    const Index n = static_cast<Index>(std::lround(2.0 * ext / st)) + 1;
    auto axis = [&](Index i) { return -ext + static_cast<double>(i) * st; };
    auto sm = [&](double x) { return 1.0 / (1.0 + std::exp(-x / o.edgeWidth)); };
    MatX acc = MatX::Zero(n, n);  // rows d, cols a
    for (int q = 0; q < o.nPhase; ++q) {
        const double ph = static_cast<double>(q) / static_cast<double>(o.nPhase) * 2.0 * kPi;
        const double al = sweep.alphaMax * std::sin(ph);
        const double lx = f0, ly = h * std::tan(al) / std::cos(tau);
        const double R = std::sqrt(lx * lx + ly * ly + h * h) * tanHalf;
        for (Index ja = 0; ja < n; ++ja) {
            const double A = axis(ja);
            for (Index id = 0; id < n; ++id) {
                const double D = axis(id);
                const double d3 = std::sqrt(A * A + D * D + h * h);
                const double pSig = 1.0 / (sensor.a + std::exp(sensor.b * (d3 - sensor.c)));
                const double inRange = sm((sensor.beta - o.edgeInset) - d3);
                const double inFp = sm((R - o.edgeInset) - std::sqrt((A - lx) * (A - lx) + (D - ly) * (D - ly)));
                const double p = inFp * (inRange * pSig + (1.0 - inRange) * pOut);
                acc(id, ja) += std::log1p(-p);
            }
        }
    }
    MatX kTab = acc / static_cast<double>(o.nPhase) / (V * dt);
    // Drop the numerically negligible tail (pOut-level looks beyond the range,
    // logistic crumbs outside the footprint): below 1e-7 log-miss per metre it
    // cannot move a pixel's miss probability by 0.01% over a kilometre, and
    // trimming it lets the deposit skip the part of the stencil the sensor
    // never sees (behind a forward-tilted mount).
    kTab = kTab.unaryExpr([](double x) { return x > -1e-7 ? 0.0 : x; });

    // --- 3. straight-line exact profile and row correction ------------------
    VecX d, lamEff;
    straightProfile(h, sweep, sensor, pOut, tanHalf, V, dt, o, d, lamEff);
    lamEff = lamEff.cwiseMax(-o.lambdaCap);
    for (Index id = 0; id < n; ++id) {
        const double rowInt = kTab.row(id).sum() * st;
        // Lambda_eff at the row, taken CONSERVATIVELY across the profile samples
        // either side (the less negative of the two): the profile falls off a
        // cliff at the reach, and a linear interpolant across that cliff would
        // credit the row just outside it with half the coverage of the row
        // just inside.
        const double leff = conservativeAt(d, lamEff, std::abs(axis(id)));
        double cfac = 0.0;
        if (rowInt < -1e-9) cfac = std::min(1.0, leff / rowInt);
        kTab.row(id) *= cfac;
    }
    {
        const MatX flipped = kTab.colwise().reverse();
        kTab = 0.5 * (kTab + flipped);
    }
    VecX dAx(n), rowSum(n);
    for (Index i = 0; i < n; ++i) {
        dAx(i) = axis(i);
        rowSum(i) = kTab.row(i).sum() * st;
    }
    VecX lamModel(d.size());
    for (Index i = 0; i < d.size(); ++i) lamModel(i) = interpFill(dAx, rowSum, d(i), 0.0);

    double hw = 0.0;
    for (Index i = 0; i < d.size(); ++i)
        if (1.0 - std::exp(lamEff(i)) >= 0.5) hw = d(i);

    SwathKernelSet out;
    SwathKernel& K = out.accurate;
    K.T = kTab;
    K.a0 = -ext;
    K.d0 = -ext;
    K.step = st;
    K.Rk = ext;
    K.reach = reach;
    K.halfWidth = hw;
    K.d = d;
    K.lambdaEff = lamEff;
    K.lambdaModel = lamModel;
    K.fitMaxErrP = (lamModel.array().exp() - lamEff.array().exp()).abs().maxCoeff();
    K.h = h;
    K.sweep = sweep;
    K.explore = false;

    // --- exploration kernel: accurate + blurred tail ------------------------
    SwathKernel X = K;
    X.d.resize(0); X.lambdaEff.resize(0); X.lambdaModel.resize(0);
    X.explore = true;
    if (o.tailSigma > 0.0 && o.tailWeight > 0.0) {
        const Index pad = static_cast<Index>(std::ceil(3.0 * o.tailSigma / st));
        MatX Tp = MatX::Zero(n + 2 * pad, n + 2 * pad);
        Tp.block(pad, pad, n, n) = kTab;
        VecX g(2 * pad + 1);
        for (Index i = 0; i < g.size(); ++i) {
            const double x = static_cast<double>(i - pad) * st / o.tailSigma;
            g(i) = std::exp(-0.5 * x * x);
        }
        g /= g.sum();
        const MatX blurred = convColsSame(convColsSame(Tp, g).transpose(), g).transpose();
        X.T = Tp + o.tailWeight * blurred;
        X.a0 = -ext - static_cast<double>(pad) * st;
        X.d0 = X.a0;
        X.Rk = ext + static_cast<double>(pad) * st;
    }
    setSupportBox(out.accurate);
    setSupportBox(X);
    out.explore = X;
    return out;
}

namespace {

/// One stencil entry's bilinear lookup.  Returns false outside the table.
struct Lookup {
    double k = 0.0, ka = 0.0, kd = 0.0;
};

inline bool lookup(const SwathKernel& K, double av, double dv, bool wantDeriv, Lookup& out) {
    const double ia = (av - K.a0) / K.step;
    const double id = (dv - K.d0) / K.step;
    const Index na = K.T.cols(), nd = K.T.rows();
    if (!(ia >= 0.0) || !(id >= 0.0) || ia >= static_cast<double>(na - 1) ||
        id >= static_cast<double>(nd - 1))
        return false;
    const Index i0 = static_cast<Index>(ia);
    const Index j0 = static_cast<Index>(id);
    const double fa = ia - static_cast<double>(i0);
    const double fd = id - static_cast<double>(j0);
    const double T00 = K.T(j0, i0), T10 = K.T(j0 + 1, i0);
    const double T01 = K.T(j0, i0 + 1), T11 = K.T(j0 + 1, i0 + 1);
    out.k = (1.0 - fa) * (1.0 - fd) * T00 + fa * (1.0 - fd) * T01 + (1.0 - fa) * fd * T10 + fa * fd * T11;
    if (wantDeriv) {
        out.ka = ((1.0 - fd) * (T01 - T00) + fd * (T11 - T10)) / K.step;
        out.kd = ((1.0 - fa) * (T10 - T00) + fa * (T11 - T01)) / K.step;
    }
    return true;
}

}  // namespace

MatX swathKernelDeposit(const Path2& pts, const Path2& tan, const VecX& ds, const FastGrid& G,
                        const SwathKernel& K) {
    MatX Lam = MatX::Zero(G.ny, G.nx);
    const std::size_t nS = G.six.size();
    for (Index j = 0; j < pts.rows(); ++j) {
        const double cx = pts(j, 0), cy = pts(j, 1), tx = tan(j, 0), ty = tan(j, 1);
        const Index ix0 = static_cast<Index>(std::lround((cx - G.x0) / G.hg));
        const Index iy0 = static_cast<Index>(std::lround((cy - G.y0) / G.hg));
        for (std::size_t s = 0; s < nS; ++s) {
            const Index IX = ix0 + G.six[s], IY = iy0 + G.siy[s];
            if (IX < 0 || IX >= G.nx || IY < 0 || IY >= G.ny) continue;
            const double rx = G.x(IX) - cx, ry = G.y(IY) - cy;
            const double av = rx * tx + ry * ty;
            if (av < K.aLo || av > K.aHi) continue;
            const double dv = ry * tx - rx * ty;
            if (dv < K.dLo || dv > K.dHi) continue;
            Lookup lk;
            if (!lookup(K, av, dv, false, lk)) continue;
            Lam(IY, IX) += ds(j) * lk.k;
        }
    }
    return Lam;
}

VecX swathKernelAtPoints(const Path2& pts, const Path2& tan, const VecX& ds, const SwathKernel& K,
                         const Path2& query) {
    VecX out = VecX::Zero(query.rows());
    const double R2 = K.Rk * K.Rk;
    for (Index q = 0; q < query.rows(); ++q) {
        double acc = 0.0;
        for (Index j = 0; j < pts.rows(); ++j) {
            const double rx = query(q, 0) - pts(j, 0), ry = query(q, 1) - pts(j, 1);
            if (rx * rx + ry * ry > R2) continue;
            const double av = rx * tan(j, 0) + ry * tan(j, 1);
            const double dv = ry * tan(j, 0) - rx * tan(j, 1);
            Lookup lk;
            if (lookup(K, av, dv, false, lk)) acc += ds(j) * lk.k;
        }
        out(q) = acc;
    }
    return out;
}

void swathKernelGradient(const Path2& pts, const Path2& tan, const VecX& ds, const FastGrid& G,
                         const SwathKernel& K, const MatX& W, Path2& gp, VecX& gth, VecX& gds,
                         VecX* yield) {
    const Index M = pts.rows();
    gp = Path2::Zero(M, 2);
    gth = VecX::Zero(M);
    gds = VecX::Zero(M);
    const std::size_t nS = G.six.size();
    for (Index j = 0; j < M; ++j) {
        const double cx = pts(j, 0), cy = pts(j, 1), tx = tan(j, 0), ty = tan(j, 1);
        const Index ix0 = static_cast<Index>(std::lround((cx - G.x0) / G.hg));
        const Index iy0 = static_cast<Index>(std::lround((cy - G.y0) / G.hg));
        double Sa = 0.0, Sd = 0.0, St = 0.0, Sk = 0.0;
        for (std::size_t s = 0; s < nS; ++s) {
            const Index IX = ix0 + G.six[s], IY = iy0 + G.siy[s];
            if (IX < 0 || IX >= G.nx || IY < 0 || IY >= G.ny) continue;
            const double w = W(IY, IX);
            if (w == 0.0) continue;
            const double rx = G.x(IX) - cx, ry = G.y(IY) - cy;
            const double av = rx * tx + ry * ty;
            if (av < K.aLo || av > K.aHi) continue;
            const double dv = ry * tx - rx * ty;
            if (dv < K.dLo || dv > K.dHi) continue;
            Lookup lk;
            if (!lookup(K, av, dv, true, lk)) continue;
            Sa += w * lk.ka;
            Sd += w * lk.kd;
            St += w * (lk.ka * dv - lk.kd * av);
            Sk += w * lk.k;
        }
        // n = [-ty, tx]
        gp(j, 0) = -ds(j) * (Sa * tx - Sd * ty);
        gp(j, 1) = -ds(j) * (Sa * ty + Sd * tx);
        gth(j) = ds(j) * St;
        gds(j) = Sk;
    }
    if (yield) *yield = -(ds.array() * gds.array()).matrix();
}

}  // namespace mtl::curve::optimization
