#include "mtl/core/dubins.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

#include "mtl/core/numeric.hpp"

namespace mtl::core {
namespace {

constexpr double kTwoPi = 2.0 * kPi;

inline double mod2pi(double a) {
    double m = std::fmod(a, kTwoPi);
    if (m < 0.0) m += kTwoPi;
    return m;
}

/// The standard normalised Dubins formulation (Shkel & Lumelsky): the problem
/// is reduced to a start at the origin heading alpha, an end at distance d
/// heading beta, with unit turning radius.  Each word returns its three
/// normalised segment lengths, or false when that word does not exist.
struct Normalised {
    double d, alpha, beta;
    double sa, sb, ca, cb, cab;
};

bool wordLSL(const Normalised& n, std::array<double, 3>& t) {
    const double tmp = n.d + n.sa - n.sb;
    const double p2  = 2.0 + n.d * n.d - 2.0 * n.cab + 2.0 * n.d * (n.sa - n.sb);
    if (p2 < 0.0) return false;
    const double th = std::atan2(n.cb - n.ca, tmp);
    t = {mod2pi(th - n.alpha), std::sqrt(p2), mod2pi(n.beta - th)};
    return true;
}

bool wordRSR(const Normalised& n, std::array<double, 3>& t) {
    const double tmp = n.d - n.sa + n.sb;
    const double p2  = 2.0 + n.d * n.d - 2.0 * n.cab + 2.0 * n.d * (n.sb - n.sa);
    if (p2 < 0.0) return false;
    const double th = std::atan2(n.ca - n.cb, tmp);
    t = {mod2pi(n.alpha - th), std::sqrt(p2), mod2pi(th - n.beta)};
    return true;
}

bool wordLSR(const Normalised& n, std::array<double, 3>& t) {
    const double p2 = -2.0 + n.d * n.d + 2.0 * n.cab + 2.0 * n.d * (n.sa + n.sb);
    if (p2 < 0.0) return false;
    const double p  = std::sqrt(p2);
    const double th = std::atan2(-n.ca - n.cb, n.d + n.sa + n.sb) - std::atan2(-2.0, p);
    t = {mod2pi(th - n.alpha), p, mod2pi(th - n.beta)};
    return true;
}

bool wordRSL(const Normalised& n, std::array<double, 3>& t) {
    const double p2 = -2.0 + n.d * n.d + 2.0 * n.cab - 2.0 * n.d * (n.sa + n.sb);
    if (p2 < 0.0) return false;
    const double p  = std::sqrt(p2);
    const double th = std::atan2(n.ca + n.cb, n.d - n.sa - n.sb) - std::atan2(2.0, p);
    t = {mod2pi(n.alpha - th), p, mod2pi(n.beta - th)};
    return true;
}

bool wordRLR(const Normalised& n, std::array<double, 3>& t) {
    const double tmp =
        (6.0 - n.d * n.d + 2.0 * n.cab + 2.0 * n.d * (n.sa - n.sb)) / 8.0;
    if (std::abs(tmp) > 1.0) return false;
    const double p  = mod2pi(kTwoPi - std::acos(tmp));
    const double th = std::atan2(n.ca - n.cb, n.d - n.sa + n.sb);
    const double t0 = mod2pi(n.alpha - th + 0.5 * p);
    t = {t0, p, mod2pi(n.alpha - n.beta - t0 + p)};
    return true;
}

bool wordLRL(const Normalised& n, std::array<double, 3>& t) {
    const double tmp =
        (6.0 - n.d * n.d + 2.0 * n.cab + 2.0 * n.d * (n.sb - n.sa)) / 8.0;
    if (std::abs(tmp) > 1.0) return false;
    const double p  = mod2pi(kTwoPi - std::acos(tmp));
    const double th = std::atan2(-n.ca + n.cb, n.d + n.sa - n.sb);
    const double t0 = mod2pi(th - n.alpha + 0.5 * p);
    t = {t0, p, mod2pi(n.beta - n.alpha - t0 + p)};
    return true;
}

/// Segment turn direction per word: +1 left, -1 right, 0 straight.
std::array<int, 3> directionsOf(DubinsWord w) {
    switch (w) {
        case DubinsWord::LSL: return {+1, 0, +1};
        case DubinsWord::LSR: return {+1, 0, -1};
        case DubinsWord::RSL: return {-1, 0, +1};
        case DubinsWord::RSR: return {-1, 0, -1};
        case DubinsWord::RLR: return {-1, +1, -1};
        case DubinsWord::LRL: return {+1, -1, +1};
        default: return {0, 0, 0};
    }
}

}  // namespace

DubinsPath DubinsPath::connect(const Pose2& from, const Pose2& to, double minTurnRadius) {
    DubinsPath path;
    path.start_  = from;
    path.radius_ = std::max(minTurnRadius, 1e-6);

    const Vec2   delta = to.xy - from.xy;
    const double dist  = delta.norm();
    const double d     = dist / path.radius_;
    const double theta = (dist > 1e-12) ? mod2pi(std::atan2(delta.y(), delta.x())) : 0.0;

    Normalised n{};
    n.d     = d;
    n.alpha = mod2pi(from.theta - theta);
    n.beta  = mod2pi(to.theta - theta);
    n.sa    = std::sin(n.alpha);
    n.sb    = std::sin(n.beta);
    n.ca    = std::cos(n.alpha);
    n.cb    = std::cos(n.beta);
    n.cab   = std::cos(n.alpha - n.beta);

    struct Candidate {
        DubinsWord word;
        bool (*fn)(const Normalised&, std::array<double, 3>&);
    };
    static const Candidate kCandidates[] = {
        {DubinsWord::LSL, &wordLSL}, {DubinsWord::RSR, &wordRSR},
        {DubinsWord::LSR, &wordLSR}, {DubinsWord::RSL, &wordRSL},
        {DubinsWord::RLR, &wordRLR}, {DubinsWord::LRL, &wordLRL},
    };

    double                best = kInf;
    std::array<double, 3> bestSeg{{0.0, 0.0, 0.0}};
    DubinsWord            bestWord = DubinsWord::Invalid;

    for (const auto& c : kCandidates) {
        std::array<double, 3> t{{0.0, 0.0, 0.0}};
        if (!c.fn(n, t)) continue;
        if (!std::isfinite(t[0]) || !std::isfinite(t[1]) || !std::isfinite(t[2])) continue;
        const double len = t[0] + t[1] + t[2];
        if (len < best) {
            best     = len;
            bestSeg  = t;
            bestWord = c.word;
        }
    }

    if (bestWord == DubinsWord::Invalid) {
        // Cannot happen for a positive radius, but a degenerate pose pair must
        // still return something usable rather than a NaN track.
        path.word_   = DubinsWord::LSL;
        path.seg_    = {0.0, dist / path.radius_, 0.0};
        path.length_ = dist;
        return path;
    }

    path.word_   = bestWord;
    path.seg_    = {bestSeg[0] * path.radius_, bestSeg[1] * path.radius_,
                    bestSeg[2] * path.radius_};
    path.length_ = best * path.radius_;
    return path;
}

Pose2 DubinsPath::interpolate(double s) const {
    Pose2 p = start_;
    if (!valid()) return p;
    s = std::min(std::max(s, 0.0), length_);

    const auto dirs = directionsOf(word_);
    double     rem  = s;
    for (int i = 0; i < 3; ++i) {
        const double seg = std::min(rem, seg_[i]);
        if (seg > 0.0) {
            if (dirs[i] == 0) {
                p.xy += seg * Vec2(std::cos(p.theta), std::sin(p.theta));
            } else {
                const double sign  = static_cast<double>(dirs[i]);
                const double dth   = sign * seg / radius_;
                const double cx    = p.xy.x() - sign * radius_ * std::sin(p.theta);
                const double cy    = p.xy.y() + sign * radius_ * std::cos(p.theta);
                const double th2   = p.theta + dth;
                p.xy   = Vec2(cx + sign * radius_ * std::sin(th2),
                              cy - sign * radius_ * std::cos(th2));
                p.theta = th2;
            }
        }
        rem -= seg;
        if (rem <= 1e-12) break;
    }
    return p;
}

Path2 DubinsPath::sample(double stepSize) const {
    const double step = std::max(stepSize, 1e-6);
    std::vector<double> lengths;
    for (double s = 0.0; s < length_; s += step) lengths.push_back(s);
    if (lengths.empty() || (length_ - lengths.back()) > 1e-5) lengths.push_back(length_);
    lengths.back() = std::min(lengths.back(), length_);

    Path2 pts(static_cast<Index>(lengths.size()), 2);
    for (std::size_t i = 0; i < lengths.size(); ++i) {
        const Pose2 q = interpolate(lengths[i]);
        pts(static_cast<Index>(i), 0) = q.xy.x();
        pts(static_cast<Index>(i), 1) = q.xy.y();
    }
    return pts;
}

std::string DubinsPath::wordName() const {
    switch (word_) {
        case DubinsWord::LSL: return "LSL";
        case DubinsWord::LSR: return "LSR";
        case DubinsWord::RSL: return "RSL";
        case DubinsWord::RSR: return "RSR";
        case DubinsWord::RLR: return "RLR";
        case DubinsWord::LRL: return "LRL";
        default: return "invalid";
    }
}

void computeDubinsWaypoints(const Path2& waypoints, double minTurnRadius,
                            const DubinsParams& opts, Path2& track, VecX& arc) {
    const Index nPts = waypoints.rows();
    track.resize(0, 2);
    arc.resize(0);
    if (nPts == 0) return;
    if (nPts == 1) {
        track = waypoints;
        arc   = VecX::Zero(1);
        return;
    }

    // --- leg directions, with a degenerate leg inheriting the previous one ---
    Path2 dirs = Path2::Zero(nPts - 1, 2);
    for (Index i = 0; i + 1 < nPts; ++i) {
        const Vec2   d = waypoints.row(i + 1) - waypoints.row(i);
        const double n = d.norm();
        if (n < 1e-6) {
            dirs.row(i) = (i > 0) ? dirs.row(i - 1) : Eigen::RowVector2d(1.0, 0.0);
        } else {
            dirs.row(i) = d.transpose() / n;
        }
    }

    // --- heading at each waypoint = bisector of the incoming and outgoing legs ---
    VecX headings(nPts);
    headings(0) = std::atan2(dirs(0, 1), dirs(0, 0));
    for (Index i = 1; i + 1 < nPts; ++i) {
        Eigen::RowVector2d b = dirs.row(i - 1) + dirs.row(i);
        if (b.norm() < 1e-6) b = dirs.row(i - 1);  // exact reversal: keep the inbound
        headings(i) = std::atan2(b(1), b(0));
    }
    headings(nPts - 1) = std::atan2(dirs(nPts - 2, 1), dirs(nPts - 2, 0));

    // --- draw and sample each segment ---
    std::vector<Vec2> pts;
    pts.reserve(static_cast<std::size_t>(nPts) * 32);
    for (Index i = 0; i + 1 < nPts; ++i) {
        const Pose2 a{Vec2(waypoints(i, 0), waypoints(i, 1)), headings(i)};
        const Pose2 b{Vec2(waypoints(i + 1, 0), waypoints(i + 1, 1)), headings(i + 1)};
        const DubinsPath seg  = DubinsPath::connect(a, b, minTurnRadius);
        const Path2      samp = seg.sample(opts.stepSize);
        // Avoid duplicating the exact waypoint at segment junctions.
        const Index first = (i > 0 && samp.rows() > 0) ? 1 : 0;
        for (Index k = first; k < samp.rows(); ++k) pts.emplace_back(samp(k, 0), samp(k, 1));
    }

    if (pts.empty()) {
        track = waypoints.row(0);
        arc   = VecX::Zero(1);
        return;
    }

    // Drop duplicated samples: a zero-length step makes the arc length
    // non-monotonic, which the trajectory interpolation cannot use.
    std::vector<Vec2> kept;
    kept.reserve(pts.size());
    kept.push_back(pts.front());
    for (std::size_t i = 1; i < pts.size(); ++i) {
        if ((pts[i] - kept.back()).norm() > 1e-9) kept.push_back(pts[i]);
    }

    track.resize(static_cast<Index>(kept.size()), 2);
    arc.resize(static_cast<Index>(kept.size()));
    arc(0)      = 0.0;
    track.row(0) = kept[0].transpose();
    for (std::size_t i = 1; i < kept.size(); ++i) {
        const auto k = static_cast<Index>(i);
        track.row(k) = kept[i].transpose();
        arc(k)       = arc(k - 1) + (kept[i] - kept[i - 1]).norm();
    }
}

double dubinsLength(const Path2& waypoints, double minTurnRadius, const DubinsParams& opts) {
    if (waypoints.rows() < 2) return 0.0;
    // Measured on the SAMPLED track, not on the analytic arc, because the
    // sampled track is what generateTrajectories flies and what the outer
    // reserve loop measures.  Using the analytic length here would make the
    // budget bisection chase a number the aircraft never produces.
    Path2 track;
    VecX  arc;
    computeDubinsWaypoints(waypoints, minTurnRadius, opts, track, arc);
    return arc.size() > 0 ? arc(arc.size() - 1) : 0.0;
}

Path2 dubinsSegment(const Pose2& from, const Pose2& to, double minTurnRadius, double stepSize) {
    if ((to.xy - from.xy).norm() < 1e-9 &&
        std::abs(core::wrapPi(to.theta - from.theta)) < 1e-9) {
        return Path2(0, 2);
    }
    return DubinsPath::connect(from, to, minTurnRadius).sample(stepSize);
}

}  // namespace mtl::core
