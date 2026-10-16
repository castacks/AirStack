#include "mtl_curve/curve_planning/path_insert.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <vector>

#include "mtl_curve/curve_planning/parametric_curve.hpp"
#include "mtl_curve/optimization/swath_kernel.hpp"

namespace mtl::curve::curve_planning {
namespace {

struct Node {
    Vec2   p = Vec2::Zero();
    double th = 0.0;        ///< old heading leaving this node (old nodes)
    Index  orig = 0;        ///< 0 = launch point, i = i-th waypoint along the old curve, -1 = centroid
    Index  b0 = 0, b1 = -1; ///< old samples of the arc ending here (b0..b1)
    double s = 0.0;         ///< arc length on the old curve at this node
    double y = 0.0;         ///< yield of the old arc ending here
    bool   fixed = false;   ///< never cut (launch point, centroid)
    bool   removed = false;
};

double bearing(const Vec2& a, const Vec2& b) { return std::atan2(b.y() - a.y(), b.x() - a.x()); }

/// Length of a connector from a (leaving with heading h) to the point b: the
/// chord plus the turn into it, R (D - sin D) for a heading change D (the
/// turn-then-straight path to a far point).
double connLen(const Vec2& a, double h, const Vec2& b, double Rmin) {
    const double D = std::abs(std::atan2(std::sin(bearing(a, b) - h), std::cos(bearing(a, b) - h)));
    return (b - a).norm() + Rmin * (D - std::sin(D));
}

/// Heading leaving the kept node k[i]: the old tangent at an old node, the
/// arrival bearing at the centroid.
double headingOut(const std::vector<Node>& P, const std::vector<std::size_t>& k, std::size_t i) {
    const Node& nd = P[k[i]];
    if (nd.orig >= 0 || i == 0) return nd.th;
    return bearing(P[k[i - 1]].p, nd.p);
}

/// Edge k[i-1] -> k[i] of the new path: the OLD ARC when the second node
/// directly follows the first on the old curve, else a connector.
double edgeLen(const std::vector<Node>& P, const std::vector<std::size_t>& k, std::size_t i, double Rmin) {
    const Node& a = P[k[i - 1]];
    const Node& b = P[k[i]];
    if (a.orig >= 0 && b.orig == a.orig + 1) return b.s - a.s;
    return connLen(a.p, headingOut(P, k, i - 1), b.p, Rmin);
}

}  // namespace

PathInsertSeed pathInsertSeed(const VecX& v, const CurveRep& rep, const Vec2& c, const SwathKernel& K,
                              const FastGrid& G, const MatX& W, const TeamParams& team) {
    PathInsertSeed out;
    const double Rmin = 1.0 / rep.kappaMax;
    const double joinMargin = team.reallocPathMargin ? *team.reallocPathMargin : 0.5 * kPi * Rmin;
    const double homeMargin = rep.hasEq() ? 2.5 * kPi * Rmin : 0.0;
    const std::optional<Vec2> goal = rep.pGoal;

    // ---- 1. waypoints along the optimised curve, and their yields ------------------
    const CurveSamples C = evalParametricCurve(v, rep);
    const Index M = C.pts.rows();
    Path2 gp;
    VecX gth, gds, y;
    optimization::swathKernelGradient(C.pts, C.tan, C.ds, G, K, W, gp, gth, gds, &y);
    y = y.cwiseMax(0.0);
    const Index stride =
        std::max<Index>(1, static_cast<Index>(std::lround(team.reallocPathWpSpacing / C.ds.mean())));

    std::vector<Node> P;
    {
        Node n0;
        n0.p = rep.pStart;
        n0.th = std::atan2(C.tan(0, 1), C.tan(0, 0));
        n0.fixed = true;
        P.push_back(n0);
        Vec2 p = rep.pStart;
        double s = 0.0, yAcc = 0.0;
        Index b0 = 0, orig = 0;
        for (Index j = 0; j < M; ++j) {
            s += C.ds(j);
            yAcc += y(j);
            p = rep.type == CurveRepresentation::Curvature ? Vec2(p + C.ds(j) * Vec2(C.tan(j, 0), C.tan(j, 1)))
                                                            : Vec2(C.pts.row(j).transpose());
            if ((j + 1) % stride == 0 || j == M - 1) {
                Node n;
                n.p = p;
                n.th = std::atan2(C.tan(std::min<Index>(j + 1, M - 1), 1), C.tan(std::min<Index>(j + 1, M - 1), 0));
                n.orig = ++orig;
                n.b0 = b0;
                n.b1 = j;
                n.s = s;
                n.y = yAcc;
                P.push_back(n);
                b0 = j + 1;
                yAcc = 0.0;
            }
        }
    }
    const std::size_t n = P.size() - 1;

    // ---- 2. cheapest insertion -------------------------------------------------------
    double best = kInf;
    std::size_t pos = n;
    for (std::size_t i = 0; i <= n; ++i) {
        double d;
        const double in = connLen(P[i].p, P[i].th, c, Rmin);
        if (i < n) {
            d = in + connLen(c, bearing(P[i].p, c), P[i + 1].p, Rmin) - (P[i + 1].s - P[i].s);
        } else if (!goal) {
            d = in;                                             // append after the end (Open)
        } else {
            continue;
        }
        if (d < best) {
            best = d;
            pos = i;
        }
    }
    {
        Node nc;
        nc.p = c;
        nc.orig = -1;
        nc.fixed = true;
        P.insert(P.begin() + static_cast<std::ptrdiff_t>(pos + 1), nc);
    }
    out.insertAfter = static_cast<Index>(pos);
    out.detour = best;

    // ---- 3. budget repair ------------------------------------------------------------
    int interiorCuts = 0;
    auto keptRows = [&]() {
        std::vector<std::size_t> k;
        for (std::size_t i = 0; i < P.size(); ++i)
            if (!P[i].removed) k.push_back(i);
        return k;
    };
    auto overrun = [&]() {
        const std::vector<std::size_t> k = keptRows();
        double len = 0.0;
        for (std::size_t i = 1; i < k.size(); ++i) len += edgeLen(P, k, i, Rmin);
        if (goal) len += connLen(P[k.back()].p, headingOut(P, k, k.size() - 1), *goal, Rmin);
        return len + joinMargin * (1 + interiorCuts) + homeMargin - rep.L;
    };
    double over = overrun();
    out.overrun0 = over;
    for (int it = 0; it < 50 && over > 0.0; ++it) {
        const std::vector<std::size_t> keep = keptRows();
        const std::size_t nk = keep.size();
        std::vector<double> cs(nk, 0.0), cy(nk + 1, 0.0);
        for (std::size_t i = 1; i < nk; ++i) cs[i] = cs[i - 1] + edgeLen(P, keep, i, Rmin);
        for (std::size_t i = 0; i < nk; ++i) cy[i + 1] = cy[i] + P[keep[i]].y;
        const double endLeg = goal ? connLen(P[keep[nk - 1]].p, headingOut(P, keep, nk - 1), *goal, Rmin) : 0.0;

        double bestFit = kInf, bestRatio = -kInf;
        std::size_t fp = 0, fq = 0, rp = 0, rq = 0;
        bool haveRatio = false;
        for (std::size_t p = 1; p < nk; ++p) {
            if (P[keep[p]].fixed) continue;
            for (std::size_t q = p; q < nk; ++q) {
                if (P[keep[q]].fixed) break;                    // a window never spans the centroid
                const bool interior = q + 1 < nk;
                // once q is gone the old arc into q+1 becomes a connector: lost too
                const double lost = cy[q + 1] - cy[p] + (interior ? P[keep[q + 1]].y : 0.0);
                double sv;
                if (interior) {
                    sv = cs[q + 1] - cs[p - 1] -
                         connLen(P[keep[p - 1]].p, headingOut(P, keep, p - 1), P[keep[q + 1]].p, Rmin) - joinMargin;
                } else if (!goal) {
                    sv = cs[nk - 1] - cs[p - 1];                // drop the tail (Open)
                } else {
                    sv = cs[nk - 1] - cs[p - 1] + endLeg -
                         connLen(P[keep[p - 1]].p, headingOut(P, keep, p - 1), *goal, Rmin);
                }
                if (sv <= 1e-6) continue;
                if (sv >= over) {
                    if (lost < bestFit) {
                        bestFit = lost;
                        fp = p;
                        fq = q;
                    }
                } else if (!std::isfinite(bestFit)) {
                    const double r = sv / std::max(lost, 1e-12);
                    if (r > bestRatio) {
                        bestRatio = r;
                        rp = p;
                        rq = q;
                        haveRatio = true;
                    }
                }
            }
        }
        std::size_t wp, wq;
        if (std::isfinite(bestFit)) {
            wp = fp;
            wq = fq;
        } else if (haveRatio) {
            wp = rp;
            wq = rq;
        } else {
            break;
        }
        for (std::size_t i = wp; i <= wq; ++i) P[keep[i]].removed = true;
        if (wq + 1 < nk) ++interiorCuts;
        out.lostYield += cy[wq + 1] - cy[wp];
        ++out.cuts;
        over = overrun();
    }
    out.overrun = over;

    // ---- 4. pieces: copied old arcs, joined by connectors ------------------------------
    Index arcStart = 0, arcEnd = -1;   // the current old arc (samples); open from the launch point
    Index prevOrig = 0;
    auto closeArc = [&]() {
        if (arcEnd >= arcStart) {
            SplicePiece a;
            a.j0 = arcStart;
            a.j1 = arcEnd;
            out.pieces.push_back(a);
        }
        arcStart = 0;
        arcEnd = -1;
    };
    for (std::size_t i = 1; i < P.size(); ++i) {
        const Node& nd = P[i];
        if (nd.removed) continue;
        if (nd.orig < 0) {                                      // the centroid
            closeArc();
            SplicePiece via;
            via.via = true;
            via.pt = nd.p;
            out.pieces.push_back(via);
            prevOrig = -1;
            continue;
        }
        if (prevOrig >= 0 && nd.orig == prevOrig + 1) {
            arcEnd = nd.b1;                                     // the old arc continues
        } else {
            closeArc();                                         // a connector to this node,
            arcStart = nd.b1 + 1;                               // then the old arc after it
            arcEnd = nd.b1;
        }
        prevOrig = nd.orig;
    }
    closeArc();
    out.prefixSamples = (!out.pieces.empty() && !out.pieces[0].via && out.pieces[0].j0 == 0)
                            ? out.pieces[0].j1 + 1 : 0;
    if (rep.type != CurveRepresentation::Curvature) out.prefixSamples = 0;
    out.sPrefix = static_cast<double>(out.prefixSamples) * C.ds(0);
    return out;
}

}  // namespace mtl::curve::curve_planning
