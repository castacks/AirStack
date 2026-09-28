// =============================================================================
//  dubins.cpp — the six Dubins words in closed form (normalised radius 1).
// =============================================================================
#include "tigris_search_planner/dubins.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace tigris_search {
namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

enum Seg { L_SEG, S_SEG, R_SEG };
constexpr Seg kWords[6][3] = {{L_SEG, S_SEG, L_SEG}, {L_SEG, S_SEG, R_SEG}, {R_SEG, S_SEG, L_SEG},
                              {R_SEG, S_SEG, R_SEG}, {R_SEG, L_SEG, R_SEG}, {L_SEG, R_SEG, L_SEG}};

/// Normalised word solutions; false when the word does not exist.
bool solve(int word, double d, double a, double b, double out[3]) {
    const double sa = std::sin(a), sb = std::sin(b), ca = std::cos(a), cb = std::cos(b);
    const double cab = std::cos(a - b);
    const double d2 = d * d;
    switch (word) {
        case DubinsPath::LSL: {
            const double psq = 2.0 + d2 - 2.0 * cab + 2.0 * d * (sa - sb);
            if (psq < 0.0) return false;
            const double tmp = std::atan2(cb - ca, d + sa - sb);
            out[0] = mod2pi(tmp - a);
            out[1] = std::sqrt(psq);
            out[2] = mod2pi(b - tmp);
            return true;
        }
        case DubinsPath::RSR: {
            const double psq = 2.0 + d2 - 2.0 * cab + 2.0 * d * (sb - sa);
            if (psq < 0.0) return false;
            const double tmp = std::atan2(ca - cb, d - sa + sb);
            out[0] = mod2pi(a - tmp);
            out[1] = std::sqrt(psq);
            out[2] = mod2pi(tmp - b);
            return true;
        }
        case DubinsPath::LSR: {
            const double psq = -2.0 + d2 + 2.0 * cab + 2.0 * d * (sa + sb);
            if (psq < 0.0) return false;
            const double p = std::sqrt(psq);
            const double tmp = std::atan2(-ca - cb, d + sa + sb) - std::atan2(-2.0, p);
            out[0] = mod2pi(tmp - a);
            out[1] = p;
            out[2] = mod2pi(tmp - mod2pi(b));
            return true;
        }
        case DubinsPath::RSL: {
            const double psq = -2.0 + d2 + 2.0 * cab - 2.0 * d * (sa + sb);
            if (psq < 0.0) return false;
            const double p = std::sqrt(psq);
            const double tmp = std::atan2(ca + cb, d - sa - sb) - std::atan2(2.0, p);
            out[0] = mod2pi(a - tmp);
            out[1] = p;
            out[2] = mod2pi(b - tmp);
            return true;
        }
        case DubinsPath::RLR: {
            const double tmp = (6.0 - d2 + 2.0 * cab + 2.0 * d * (sa - sb)) / 8.0;
            if (std::fabs(tmp) > 1.0) return false;
            const double p = mod2pi(kTwoPi - std::acos(tmp));
            const double t = mod2pi(a - std::atan2(ca - cb, d - sa + sb) + p / 2.0);
            out[0] = t;
            out[1] = p;
            out[2] = mod2pi(a - b - t + p);
            return true;
        }
        case DubinsPath::LRL: {
            const double tmp = (6.0 - d2 + 2.0 * cab + 2.0 * d * (sb - sa)) / 8.0;
            if (std::fabs(tmp) > 1.0) return false;
            const double p = mod2pi(kTwoPi - std::acos(tmp));
            const double t = mod2pi(-a - std::atan2(ca - cb, d + sa - sb) + p / 2.0);
            out[0] = t;
            out[1] = p;
            out[2] = mod2pi(mod2pi(b) - a - t + p);
            return true;
        }
        default:
            return false;
    }
}

/// Advance a normalised pose (x, y, th) by t along one segment type.
void advance(Seg type, double t, double& x, double& y, double& th) {
    switch (type) {
        case L_SEG:
            x += std::sin(th + t) - std::sin(th);
            y += -std::cos(th + t) + std::cos(th);
            th += t;
            break;
        case R_SEG:
            x += -std::sin(th - t) + std::sin(th);
            y += std::cos(th - t) - std::cos(th);
            th -= t;
            break;
        case S_SEG:
            x += std::cos(th) * t;
            y += std::sin(th) * t;
            break;
    }
}

}  // namespace

double mod2pi(double a) {
    double v = std::fmod(a, kTwoPi);
    if (v < 0.0) v += kTwoPi;
    return v;
}

double wrapPi(double a) {
    double v = std::fmod(a + kPi, kTwoPi);
    if (v < 0.0) v += kTwoPi;
    return v - kPi;
}

Pose2 DubinsPath::sample(double s) const {
    double t = std::max(0.0, std::min(s, length())) / rho;  // normalised arc
    double x = 0.0, y = 0.0, th = q0.yaw;
    const Seg* types = kWords[word];
    for (int i = 0; i < 3 && t > 0.0; ++i) {
        const double step = std::min(t, seg[i]);
        advance(types[i], step, x, y, th);
        t -= step;
    }
    Pose2 p;
    p.x = q0.x + x * rho;
    p.y = q0.y + y * rho;
    p.yaw = wrapPi(th);
    return p;
}

bool dubinsShortest(const Pose2& a, const Pose2& b, double rho, DubinsPath* out) {
    if (!(rho > 0.0) || out == nullptr) return false;
    const double dx = b.x - a.x, dy = b.y - a.y;
    const double d = std::hypot(dx, dy) / rho;
    const double theta = (d > 0.0) ? mod2pi(std::atan2(dy, dx)) : 0.0;
    const double alpha = mod2pi(a.yaw - theta);
    const double beta = mod2pi(b.yaw - theta);
    double best = std::numeric_limits<double>::infinity();
    bool found = false;
    for (int w = 0; w < 6; ++w) {
        double p[3];
        if (!solve(w, d, alpha, beta, p)) continue;
        const double len = p[0] + p[1] + p[2];
        if (len < best) {
            best = len;
            found = true;
            out->word = w;
            out->seg[0] = p[0];
            out->seg[1] = p[1];
            out->seg[2] = p[2];
        }
    }
    out->q0 = a;
    out->rho = rho;
    return found;
}

}  // namespace tigris_search
