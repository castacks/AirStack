#include "mtl/core/numeric.hpp"

#include <numeric>
#include <stdexcept>

namespace mtl::core {

double polylineLength(const Path2& p) {
    if (p.rows() < 2) return 0.0;
    return (p.bottomRows(p.rows() - 1) - p.topRows(p.rows() - 1)).rowwise().norm().sum();
}

double polylineLength(const Path3& p) {
    if (p.rows() < 2) return 0.0;
    const auto a = p.block(1, 0, p.rows() - 1, 2);
    const auto b = p.block(0, 0, p.rows() - 1, 2);
    return (a - b).rowwise().norm().sum();
}

void arcOf(const Path2& xy, VecX& ds, VecX& arc) {
    const Index n = xy.rows();
    ds  = VecX::Zero(n);
    arc = VecX::Zero(n);
    for (Index k = 1; k < n; ++k) {
        ds(k)  = (xy.row(k) - xy.row(k - 1)).norm();
        arc(k) = arc(k - 1) + ds(k);
    }
}

VecX movingAverage(const VecX& x, int window) {
    const int win = std::max(1, window);
    if (win <= 1) return x;
    const Index n = x.size();
    if (n == 0) return x;

    // MATLAB: conv(x, ones(win,1)/win, 'same') ./ conv(ones(n,1), ones(win,1)/win, 'same').
    // The 'same' part of a length-(n+win-1) full convolution starts at
    // offset floor(win/2) (0-based).
    const Index full   = n + win - 1;
    const Index offset = win / 2;
    VecX num = VecX::Zero(full);
    VecX den = VecX::Zero(full);
    for (Index i = 0; i < n; ++i) {
        for (int j = 0; j < win; ++j) {
            num(i + j) += x(i);
            den(i + j) += 1.0;
        }
    }
    VecX y(n);
    for (Index i = 0; i < n; ++i) {
        const Index k = i + offset;
        y(i) = den(k) > 0.0 ? num(k) / den(k) : x(i);
    }
    return y;
}

VecX unwrap(const VecX& x) {
    VecX y = x;
    double offset = 0.0;
    for (Index k = 1; k < y.size(); ++k) {
        const double d = x(k) - x(k - 1);
        if (d > kPi) {
            offset -= 2.0 * kPi;
        } else if (d < -kPi) {
            offset += 2.0 * kPi;
        }
        y(k) = x(k) + offset;
    }
    return y;
}

VecX headingOf(const Path2& xy, const VecX& ds) {
    const Index n = xy.rows();
    VecX psi = VecX::Zero(std::max<Index>(n, 1));
    if (n == 0) return psi;
    if (n == 1) return psi;

    Path2 d(n, 2);
    d.row(0) = xy.row(1) - xy.row(0);
    for (Index k = 1; k < n; ++k) d.row(k) = xy.row(k) - xy.row(k - 1);

    double last = std::atan2(d(0, 1), d(0, 0));
    for (Index k = 0; k < n; ++k) {
        if (ds(k) > 1e-9 || k == 0) {
            if (d.row(k).norm() > 1e-9) last = std::atan2(d(k, 1), d(k, 0));
        }
        psi(k) = last;
    }
    psi = unwrap(psi);
    return movingAverage(psi, 5);
}

VecX slopeOf(const VecX& h, const VecX& ds) {
    const Index n = h.size();
    VecX dhds = VecX::Zero(n);
    if (n == 0) return dhds;
    for (Index k = 0; k + 1 < n; ++k) {
        if (ds(k + 1) > 1e-9) dhds(k) = (h(k + 1) - h(k)) / ds(k + 1);
    }
    dhds(n - 1) = dhds(std::max<Index>(n - 2, 0));
    return dhds;
}

VecX interp1(const VecX& x, const VecX& y, const VecX& xq) {
    const Index n = x.size();
    VecX out(xq.size());
    if (n == 0) {
        out.setZero();
        return out;
    }
    if (n == 1) {
        out.setConstant(y(0));
        return out;
    }
    Index lo = 0;
    for (Index q = 0; q < xq.size(); ++q) {
        const double v = xq(q);
        if (v <= x(0)) {
            out(q) = y(0);
            continue;
        }
        if (v >= x(n - 1)) {
            out(q) = y(n - 1);
            continue;
        }
        // Queries arrive monotonically in every call site, so walking the index
        // forward is O(n) overall rather than O(n log n).
        if (x(lo) > v) lo = 0;
        while (lo + 1 < n - 1 && x(lo + 1) < v) ++lo;
        const double x0 = x(lo), x1 = x(lo + 1);
        const double t  = (x1 - x0) > 0.0 ? (v - x0) / (x1 - x0) : 0.0;
        out(q) = y(lo) + t * (y(lo + 1) - y(lo));
    }
    return out;
}

Path2 resampleAlongPath(const Path2& path, Index nSteps) {
    Path2 pts(std::max<Index>(nSteps, 0), 2);
    if (nSteps <= 0) return pts;
    if (path.rows() == 0) {
        pts.setZero();
        return pts;
    }

    // Drop repeated arc-length values, as MATLAB's unique(arc,'stable') does.
    std::vector<double> arc;
    std::vector<Vec2>   pt;
    arc.reserve(path.rows());
    pt.reserve(path.rows());
    double acc = 0.0;
    arc.push_back(0.0);
    pt.emplace_back(path(0, 0), path(0, 1));
    for (Index i = 1; i < path.rows(); ++i) {
        acc += (path.row(i) - path.row(i - 1)).norm();
        if (acc > arc.back()) {
            arc.push_back(acc);
            pt.emplace_back(path(i, 0), path(i, 1));
        }
    }

    if (arc.size() < 2 || arc.back() <= 0.0) {
        for (Index i = 0; i < nSteps; ++i) pts.row(i) = path.row(0);
        return pts;
    }

    VecX a = Eigen::Map<VecX>(arc.data(), static_cast<Index>(arc.size()));
    VecX px(static_cast<Index>(pt.size())), py(static_cast<Index>(pt.size()));
    for (std::size_t i = 0; i < pt.size(); ++i) {
        px(static_cast<Index>(i)) = pt[i].x();
        py(static_cast<Index>(i)) = pt[i].y();
    }
    VecX q(nSteps);
    if (nSteps == 1) {
        q(0) = 0.0;
    } else {
        for (Index i = 0; i < nSteps; ++i)
            q(i) = arc.back() * static_cast<double>(i) / static_cast<double>(nSteps - 1);
    }
    pts.col(0) = interp1(a, px, q);
    pts.col(1) = interp1(a, py, q);
    return pts;
}

double quantile(std::vector<double> x, double p) {
    if (x.empty()) return 0.0;
    std::sort(x.begin(), x.end());
    const auto n = static_cast<double>(x.size());
    auto idx = static_cast<long>(std::ceil(p * n));
    idx = std::max<long>(1, std::min<long>(static_cast<long>(x.size()), idx));
    return x[static_cast<std::size_t>(idx - 1)];
}

VecX coordinatedRoll(const VecX& yaw, const VecX& ds, double dt, double maxRoll,
                     double rollSign, double rollSmoothSec) {
    const Index n = yaw.size();
    VecX yr = VecX::Zero(n);
    for (Index k = 1; k < n; ++k) yr(k) = (yaw(k) - yaw(k - 1)) / dt;

    const int win = std::max(3, static_cast<int>(std::lround(rollSmoothSec / dt)));
    yr = movingAverage(yr, win);

    VecX roll(n);
    for (Index k = 0; k < n; ++k) {
        const double v = ds(k) / dt;
        roll(k) = -rollSign * std::atan2(v * yr(k), 9.81);
    }
    roll = movingAverage(roll, win);
    return clampAbs(roll, maxRoll);
}

Vec2 projectPointOnSegment(const Vec2& pt, const Vec2& a, const Vec2& b) {
    const Vec2   ab = b - a;
    const double l2 = ab.squaredNorm();
    if (l2 < 1e-18) return a;
    double t = (pt - a).dot(ab) / l2;
    t = std::min(std::max(t, 0.0), 1.0);
    return a + t * ab;
}

VecX smoothstepProfile(const std::vector<Index>& knots, const std::vector<double>& values,
                       Index n) {
    VecX y = VecX::Zero(std::max<Index>(n, 0));
    if (n <= 0 || knots.empty()) return y;
    if (knots.size() == 1) {
        y.setConstant(values[0]);
        return y;
    }
    const Index head = std::max<Index>(0, std::min<Index>(n - 1, knots.front()));
    for (Index i = 0; i <= head; ++i) y(i) = values.front();

    for (std::size_t i = 0; i + 1 < knots.size(); ++i) {
        const Index k1 = std::max<Index>(0, knots[i]);
        const Index k2 = std::min<Index>(n - 1, knots[i + 1]);
        if (k2 <= k1) continue;
        const double span = static_cast<double>(k2 - k1);
        for (Index k = k1; k <= k2; ++k) {
            const double u = static_cast<double>(k - k1) / span;
            const double s = u * u * (3.0 - 2.0 * u);  // S(0)=0 S(1)=1 S'=0 at both ends
            y(k) = values[i] + (values[i + 1] - values[i]) * s;
        }
    }
    if (knots.back() < n - 1) {
        for (Index k = std::max<Index>(0, knots.back()); k < n; ++k) y(k) = values.back();
    }
    return y;
}

VecX rateLimit(const VecX& x, double rate) {
    VecX y = x;
    for (Index k = 1; k < y.size(); ++k) y(k) = y(k - 1) + clampAbs(y(k) - y(k - 1), rate);
    return y;
}

}  // namespace mtl::core
