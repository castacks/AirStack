// The visit-every-set GCS - port of gcsTour.m / gcsFixedOrder.m (see there for
// the formulation; the variable layout and row order are the same).
#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <numeric>
#include <random>
#include <set>
#include <string>

#include "mtl_curve/gcs/gcs.hpp"

namespace mtl::curve::gcs {
namespace {

using Trip = Eigen::Triplet<double>;

/// Rows of one cone block, accumulated as triplets.
struct Block {
    std::vector<Trip> t;
    std::vector<double> b;
    int m = 0;
    void row(const std::vector<Index>& cols, const std::vector<double>& vals, double rhs) {
        for (std::size_t i = 0; i < cols.size(); ++i) t.emplace_back(m, static_cast<int>(cols[i]), vals[i]);
        b.push_back(rhs);
        ++m;
    }
};

ConicProblem assemble(const VecX& c, const Block& Z, const Block& L, const Block& Q, const std::vector<int>& qs,
                      Index nv) {
    ConicProblem P;
    P.c = c;
    const int m = Z.m + L.m + Q.m;
    std::vector<Trip> all;
    all.reserve(Z.t.size() + L.t.size() + Q.t.size());
    for (const Trip& x : Z.t) all.push_back(x);
    for (const Trip& x : L.t) all.emplace_back(x.row() + Z.m, x.col(), x.value());
    for (const Trip& x : Q.t) all.emplace_back(x.row() + Z.m + L.m, x.col(), x.value());
    P.A.resize(m, nv);
    P.A.setFromTriplets(all.begin(), all.end());
    P.b.resize(m);
    Index r = 0;
    for (double v : Z.b) P.b(r++) = v;
    for (double v : L.b) P.b(r++) = v;
    for (double v : Q.b) P.b(r++) = v;
    P.z = Z.m;
    P.l = L.m;
    P.q = qs;
    return P;
}

double routeLen(const std::vector<Vec2>& P, const Vec2& s, const std::optional<Vec2>& g) {
    double L = 0.0;
    Vec2 prev = s;
    for (const Vec2& p : P) {
        L += (p - prev).norm();
        prev = p;
    }
    if (g) L += (*g - prev).norm();
    return L;
}

std::vector<Vec2> rowsOf(const Path2& X, const std::vector<Index>& o) {
    std::vector<Vec2> P;
    for (const Index i : o) P.emplace_back(X.row(i).transpose());
    return P;
}

/// Edmonds-Karp on a dense capacity matrix; reach = source side of the min cut.
double maxflow(const MatX& C, int s, int t, std::vector<char>& reach) {
    const int N = static_cast<int>(C.rows());
    MatX R = C;
    double f = 0.0;
    while (true) {
        std::vector<int> prev(static_cast<std::size_t>(N), -1);
        prev[static_cast<std::size_t>(s)] = s;
        std::deque<int> q{s};
        while (!q.empty() && prev[static_cast<std::size_t>(t)] < 0) {
            const int u = q.front();
            q.pop_front();
            for (int v = 0; v < N; ++v)
                if (prev[static_cast<std::size_t>(v)] < 0 && R(u, v) > 1e-9) {
                    prev[static_cast<std::size_t>(v)] = u;
                    q.push_back(v);
                }
        }
        if (prev[static_cast<std::size_t>(t)] < 0) break;
        double b = kInf;
        for (int v = t; v != s; v = prev[static_cast<std::size_t>(v)]) b = std::min(b, R(prev[static_cast<std::size_t>(v)], v));
        for (int v = t; v != s; v = prev[static_cast<std::size_t>(v)]) {
            const int u = prev[static_cast<std::size_t>(v)];
            R(u, v) -= b;
            R(v, u) += b;
        }
        f += b;
        if (f >= 1.0) break;
    }
    reach.assign(static_cast<std::size_t>(N), 0);
    reach[static_cast<std::size_t>(s)] = 1;
    std::deque<int> q{s};
    while (!q.empty()) {
        const int u = q.front();
        q.pop_front();
        for (int v = 0; v < N; ++v)
            if (!reach[static_cast<std::size_t>(v)] && R(u, v) > 1e-9) {
                reach[static_cast<std::size_t>(v)] = 1;
                q.push_back(v);
            }
    }
    return f;
}

std::vector<Index> walk(const MatX& Y, Index n, bool random, const Path2& X, const Vec2& pStart,
                        std::mt19937_64& rng) {
    std::vector<Index> o;
    std::vector<char> vis(static_cast<std::size_t>(n), 0);
    Index u = 0;
    Vec2 pos = pStart;
    std::uniform_real_distribution<double> U(0.0, 1.0);
    for (Index k = 0; k < n; ++k) {
        std::vector<Index> cand;
        std::vector<double> w;
        for (Index v = 0; v < n; ++v)
            if (!vis[static_cast<std::size_t>(v)]) {
                cand.push_back(v);
                w.push_back(Y(u, v + 1) + 1e-6 / (1.0 + (Vec2(X.row(v).transpose()) - pos).norm()));
            }
        std::size_t j = 0;
        if (random) {
            const double tot = std::accumulate(w.begin(), w.end(), 0.0);
            const double r = U(rng) * tot;
            double acc = 0.0;
            j = w.size() - 1;
            for (std::size_t i = 0; i < w.size(); ++i) {
                acc += w[i];
                if (r <= acc) {
                    j = i;
                    break;
                }
            }
        } else {
            j = static_cast<std::size_t>(std::max_element(w.begin(), w.end()) - w.begin());
        }
        const Index v = cand[j];
        o.push_back(v);
        vis[static_cast<std::size_t>(v)] = 1;
        u = v + 1;
        pos = X.row(v).transpose();
    }
    return o;
}

std::vector<Index> nearestOrder(const Path2& X, const Vec2& pStart) {
    const Index n = X.rows();
    std::vector<Index> o;
    std::vector<char> vis(static_cast<std::size_t>(n), 0);
    Vec2 pos = pStart;
    for (Index k = 0; k < n; ++k) {
        double bd = kInf;
        Index bv = 0;
        for (Index v = 0; v < n; ++v)
            if (!vis[static_cast<std::size_t>(v)]) {
                const double d = (Vec2(X.row(v).transpose()) - pos).norm();
                if (d < bd) {
                    bd = d;
                    bv = v;
                }
            }
        o.push_back(bv);
        vis[static_cast<std::size_t>(bv)] = 1;
        pos = X.row(bv).transpose();
    }
    return o;
}

/// 2-opt and or-opt (single relocation) on fixed points X (row v = point of set v).
std::vector<Index> localSearch(std::vector<Index> o, const Path2& X, const Vec2& s, const std::optional<Vec2>& g) {
    const std::size_t n = o.size();
    double cur = routeLen(rowsOf(X, o), s, g);
    bool improved = true;
    while (improved) {
        improved = false;
        for (std::size_t i = 0; i + 1 < n; ++i)
            for (std::size_t j = i + 1; j < n; ++j) {
                std::vector<Index> o2 = o;
                std::reverse(o2.begin() + static_cast<std::ptrdiff_t>(i), o2.begin() + static_cast<std::ptrdiff_t>(j) + 1);
                const double L2 = routeLen(rowsOf(X, o2), s, g);
                if (L2 < cur - 1e-6) {
                    o = o2;
                    cur = L2;
                    improved = true;
                }
            }
        for (std::size_t i = 0; i < n; ++i)
            for (std::size_t j = 0; j < n; ++j) {
                if (i == j) continue;
                std::vector<Index> o2 = o;
                const Index v = o2[i];
                o2.erase(o2.begin() + static_cast<std::ptrdiff_t>(i));
                o2.insert(o2.begin() + static_cast<std::ptrdiff_t>(std::min(j, o2.size())), v);
                const double L2 = routeLen(rowsOf(X, o2), s, g);
                if (L2 < cur - 1e-6) {
                    o = o2;
                    cur = L2;
                    improved = true;
                }
            }
    }
    return o;
}

Vec2 projectToSet(const ConvexSet& S, Vec2 p) {
    const VecX viol = S.H * p - S.h;
    if (viol.maxCoeff() <= 1e-6) return p;
    if (viol.maxCoeff() <= 1.0) p -= S.H.transpose() * viol.cwiseMax(0.0);
    if ((S.H * p - S.h).maxCoeff() > 1e-3) p = S.V.colwise().mean().transpose();
    return p;
}

}  // namespace

double routeLength(const Path2& P, const Vec2& pStart, const std::optional<Vec2>& pGoal) {
    std::vector<Vec2> v;
    for (Index i = 0; i < P.rows(); ++i) v.emplace_back(P.row(i).transpose());
    return routeLen(v, pStart, pGoal);
}

double turnOverhead(const Path2& P, const Vec2& pStart, const std::optional<Vec2>& pGoal, double Rmin) {
    std::vector<Vec2> Q{pStart};
    for (Index i = 0; i < P.rows(); ++i) Q.emplace_back(P.row(i).transpose());
    if (pGoal) Q.push_back(*pGoal);
    std::vector<double> th;
    for (std::size_t i = 1; i < Q.size(); ++i) {
        const Vec2 d = Q[i] - Q[i - 1];
        if (d.norm() > 1e-6) th.push_back(std::atan2(d.y(), d.x()));
    }
    double o = 0.0;
    for (std::size_t i = 1; i < th.size(); ++i) {
        const double D = std::abs(std::atan2(std::sin(th[i] - th[i - 1]), std::cos(th[i] - th[i - 1])));
        o += Rmin * (D - std::sin(D));
    }
    return o;
}

FixedOrderResult fixedOrderRoute(const std::vector<ConvexSet>& sets, const std::vector<Index>& order,
                                 const Vec2& pStart, const std::optional<Vec2>& pGoal,
                                 const std::vector<double>& centerWeights, const GcsParams& opts) {
    FixedOrderResult R;
    const double S = opts.scale;
    const Index n = static_cast<Index>(order.size());
    const bool hasGoal = pGoal.has_value();
    if (n == 0) {
        R.pts.resize(0, 2);
        R.len = hasGoal ? (*pGoal - pStart).norm() : 0.0;
        return R;
    }
    const Index nl = n + (hasGoal ? 1 : 0);
    std::vector<Index> wk;
    std::vector<double> wv;
    for (Index k = 0; k < n; ++k) {
        const double w = centerWeights.empty() ? 0.0 : centerWeights[static_cast<std::size_t>(order[static_cast<std::size_t>(k)])];
        if (w > 0.0) {
            wk.push_back(k);
            wv.push_back(w);
        }
    }
    const Index nv = 2 * n + nl + static_cast<Index>(wk.size());
    auto ix = [](Index k, int d) { return 2 * k + d; };
    auto itc = [n](Index k) { return 2 * n + k; };
    const Index ir0 = 2 * n + nl;
    Block Z, L, Q;
    std::vector<int> qs;
    for (Index k = 0; k < n; ++k) {
        const ConvexSet& C = sets[static_cast<std::size_t>(order[static_cast<std::size_t>(k)])];
        for (Index r = 0; r < C.H.rows(); ++r) L.row({ix(k, 0), ix(k, 1)}, {C.H(r, 0), C.H(r, 1)}, C.h(r) / S);
    }
    const Vec2 s0 = pStart / S;
    for (Index k = 0; k < nl; ++k) {
        Q.row({itc(k)}, {-1.0}, 0.0);
        for (int d = 0; d < 2; ++d) {
            std::vector<Index> cols;
            std::vector<double> vals;
            double b = 0.0;
            if (k < n) {
                cols.push_back(ix(k, d));
                vals.push_back(-1.0);
            } else {
                b += (*pGoal)(d) / S;
            }
            if (k >= 1) {
                cols.push_back(ix(k - 1, d));
                vals.push_back(1.0);
            } else {
                b -= s0(d);
            }
            Q.row(cols, vals, b);
        }
        qs.push_back(3);
    }
    for (std::size_t j = 0; j < wk.size(); ++j) {
        const Vec2 ck = sets[static_cast<std::size_t>(order[static_cast<std::size_t>(wk[j])])].c / S;
        Q.row({ir0 + static_cast<Index>(j)}, {-1.0}, 0.0);
        for (int d = 0; d < 2; ++d) Q.row({ix(wk[j], d)}, {-1.0}, -ck(d));
        qs.push_back(3);
    }
    VecX c = VecX::Zero(nv);
    for (Index k = 0; k < nl; ++k) c(itc(k)) = 1.0;
    for (std::size_t j = 0; j < wk.size(); ++j) c(ir0 + static_cast<Index>(j)) = wv[j];
    const ConicResult cr = solveConic(assemble(c, Z, L, Q, qs, nv), opts);
    R.pts.resize(n, 2);
    for (Index k = 0; k < n; ++k) R.pts.row(k) << S * cr.x(ix(k, 0)), S * cr.x(ix(k, 1));
    R.len = routeLength(R.pts, pStart, pGoal);
    return R;
}

TourResult tour(const std::vector<ConvexSet>& sets, const Vec2& pStart, const std::optional<Vec2>& pGoal,
                const std::vector<double>& centerWeights, const GcsParams& opts) {
    const auto t0 = std::chrono::steady_clock::now();
    TourResult T;
    const Index n = static_cast<Index>(sets.size());
    auto done = [&]() { T.time = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(); };
    if (n == 0) {
        T.pts.resize(0, 2);
        T.len = pGoal ? (*pGoal - pStart).norm() : 0.0;
        T.lb = T.len;
        done();
        return T;
    }
    if (n == 1) {
        const FixedOrderResult F = fixedOrderRoute(sets, {0}, pStart, pGoal, centerWeights, opts);
        T.order = {0};
        T.pts = F.pts;
        T.len = F.len;
        T.lb = F.len;
        T.nSolves = 1;
        done();
        return T;
    }
    const double S = opts.scale;
    const bool hasGoal = pGoal.has_value();
    const Vec2 s0 = pStart / S;
    const Vec2 g0 = hasGoal ? Vec2(*pGoal / S) : Vec2::Zero();

    // ---- edges: node 0 = source, 1..n sets, n+1 = terminal ----------------------
    struct Edge {
        Index u, v;
    };
    std::vector<Edge> E;
    for (Index i = 1; i <= n; ++i) E.push_back({0, i});
    for (Index i = 1; i <= n; ++i)          // MATLAB meshgrid order: head varies fastest
        for (Index j = 1; j <= n; ++j)
            if (i != j) E.push_back({i, j});
    for (Index i = 1; i <= n; ++i) E.push_back({i, n + 1});
    const Index nE = static_cast<Index>(E.size());
    std::vector<Index> iy(static_cast<std::size_t>(nE)), izu(static_cast<std::size_t>(nE), -1),
        izv(static_cast<std::size_t>(nE), -1), it(static_cast<std::size_t>(nE), -1);
    Index nv = 0;
    for (Index e = 0; e < nE; ++e) iy[static_cast<std::size_t>(e)] = nv++;
    for (Index e = 0; e < nE; ++e)
        if (E[static_cast<std::size_t>(e)].u >= 1) { izu[static_cast<std::size_t>(e)] = nv; nv += 2; }
    for (Index e = 0; e < nE; ++e)
        if (E[static_cast<std::size_t>(e)].v <= n) { izv[static_cast<std::size_t>(e)] = nv; nv += 2; }
    for (Index e = 0; e < nE; ++e)
        if (E[static_cast<std::size_t>(e)].v <= n || hasGoal) it[static_cast<std::size_t>(e)] = nv++;
    std::vector<double> w(static_cast<std::size_t>(n), 0.0);
    if (!centerWeights.empty()) w = centerWeights;
    std::vector<Index> ir(static_cast<std::size_t>(n), -1);
    for (Index v = 0; v < n; ++v)
        if (w[static_cast<std::size_t>(v)] > 0.0) ir[static_cast<std::size_t>(v)] = nv++;

    std::vector<std::vector<Index>> ein(static_cast<std::size_t>(n + 2)), eout(static_cast<std::size_t>(n + 2));
    for (Index e = 0; e < nE; ++e) {
        eout[static_cast<std::size_t>(E[static_cast<std::size_t>(e)].u)].push_back(e);
        ein[static_cast<std::size_t>(E[static_cast<std::size_t>(e)].v)].push_back(e);
    }
    Block Z, Lb, Q;
    std::vector<int> qs;
    auto ys = [&](const std::vector<Index>& es) {
        std::vector<Index> c;
        for (const Index e : es) c.push_back(iy[static_cast<std::size_t>(e)]);
        return c;
    };
    for (Index v = 1; v <= n; ++v) {
        const auto& a = ein[static_cast<std::size_t>(v)];
        const auto& b = eout[static_cast<std::size_t>(v)];
        Z.row(ys(a), std::vector<double>(a.size(), 1.0), 1.0);
        Z.row(ys(b), std::vector<double>(b.size(), 1.0), 1.0);
    }
    Z.row(ys(eout[0]), std::vector<double>(eout[0].size(), 1.0), 1.0);
    for (Index v = 1; v <= n; ++v)
        for (int d = 0; d < 2; ++d) {
            std::vector<Index> cols;
            std::vector<double> vals;
            for (const Index e : ein[static_cast<std::size_t>(v)]) { cols.push_back(izv[static_cast<std::size_t>(e)] + d); vals.push_back(1.0); }
            for (const Index e : eout[static_cast<std::size_t>(v)]) { cols.push_back(izu[static_cast<std::size_t>(e)] + d); vals.push_back(-1.0); }
            Z.row(cols, vals, 0.0);
        }
    for (Index e = 0; e < nE; ++e) Lb.row({iy[static_cast<std::size_t>(e)]}, {-1.0}, 0.0);
    for (Index e = 0; e < nE; ++e) {
        const Edge& ed = E[static_cast<std::size_t>(e)];
        if (ed.u >= 1) {
            const ConvexSet& C = sets[static_cast<std::size_t>(ed.u - 1)];
            for (Index r = 0; r < C.H.rows(); ++r)
                Lb.row({izu[static_cast<std::size_t>(e)], izu[static_cast<std::size_t>(e)] + 1, iy[static_cast<std::size_t>(e)]},
                       {C.H(r, 0), C.H(r, 1), -C.h(r) / S}, 0.0);
        }
        if (ed.v <= n) {
            const ConvexSet& C = sets[static_cast<std::size_t>(ed.v - 1)];
            for (Index r = 0; r < C.H.rows(); ++r)
                Lb.row({izv[static_cast<std::size_t>(e)], izv[static_cast<std::size_t>(e)] + 1, iy[static_cast<std::size_t>(e)]},
                       {C.H(r, 0), C.H(r, 1), -C.h(r) / S}, 0.0);
        }
    }
    for (Index e = 0; e < nE; ++e) {
        if (it[static_cast<std::size_t>(e)] < 0) continue;
        const Edge& ed = E[static_cast<std::size_t>(e)];
        Q.row({it[static_cast<std::size_t>(e)]}, {-1.0}, 0.0);
        for (int d = 0; d < 2; ++d) {
            std::vector<Index> cols;
            std::vector<double> vals;
            if (ed.v <= n) { cols.push_back(izv[static_cast<std::size_t>(e)] + d); vals.push_back(-1.0); }
            else           { cols.push_back(iy[static_cast<std::size_t>(e)]); vals.push_back(-g0(d)); }
            if (ed.u >= 1) { cols.push_back(izu[static_cast<std::size_t>(e)] + d); vals.push_back(1.0); }
            else           { cols.push_back(iy[static_cast<std::size_t>(e)]); vals.push_back(s0(d)); }
            Q.row(cols, vals, 0.0);
        }
        qs.push_back(3);
    }
    for (Index v = 1; v <= n; ++v) {
        if (ir[static_cast<std::size_t>(v - 1)] < 0) continue;
        Q.row({ir[static_cast<std::size_t>(v - 1)]}, {-1.0}, 0.0);
        for (int d = 0; d < 2; ++d) {
            std::vector<Index> cols;
            for (const Index e : eout[static_cast<std::size_t>(v)]) cols.push_back(izu[static_cast<std::size_t>(e)] + d);
            Q.row(cols, std::vector<double>(cols.size(), -1.0), -sets[static_cast<std::size_t>(v - 1)].c(d) / S);
        }
        qs.push_back(3);
    }
    VecX c = VecX::Zero(nv);
    for (Index e = 0; e < nE; ++e)
        if (it[static_cast<std::size_t>(e)] >= 0) c(it[static_cast<std::size_t>(e)]) = 1.0;
    for (Index v = 0; v < n; ++v)
        if (ir[static_cast<std::size_t>(v)] >= 0) c(ir[static_cast<std::size_t>(v)]) = w[static_cast<std::size_t>(v)];

    // ---- relaxation + cutset separation --------------------------------------------
    Block cuts;
    VecX x;
    std::set<std::vector<int>> seenCuts;
    for (int rnd = 0; rnd <= opts.maxCutRounds; ++rnd) {
        Block L = Lb;
        for (const Trip& tr : cuts.t) L.t.emplace_back(tr.row() + Lb.m, tr.col(), tr.value());
        L.b.insert(L.b.end(), cuts.b.begin(), cuts.b.end());
        L.m += cuts.m;
        const ConicResult cr = solveConic(assemble(c, Z, L, Q, qs, nv), opts);
        ++T.nSolves;
        x = cr.x;
        if (!cr.ok && !x.allFinite()) {
            T.ok = false;
            break;
        }
        // separation
        const int N = static_cast<int>(n + 2);
        MatX Cap = MatX::Zero(N, N);
        for (Index e = 0; e < nE; ++e)
            Cap(E[static_cast<std::size_t>(e)].u, E[static_cast<std::size_t>(e)].v) += std::max(0.0, x(iy[static_cast<std::size_t>(e)]));
        int added = 0;
        for (Index v = 1; v <= n; ++v) {
            std::vector<char> reach;
            const double f = maxflow(Cap, 0, static_cast<int>(v), reach);
            if (f >= 1.0 - 1e-4) continue;
            std::vector<int> Sset;
            for (int u = 0; u < N; ++u)
                if (!reach[static_cast<std::size_t>(u)]) Sset.push_back(u);
            if (!seenCuts.insert(Sset).second) continue;
            std::vector<char> inS(static_cast<std::size_t>(N), 0);
            for (const int u : Sset) inS[static_cast<std::size_t>(u)] = 1;
            std::vector<Index> cols;
            for (Index e = 0; e < nE; ++e)
                if (!inS[static_cast<std::size_t>(E[static_cast<std::size_t>(e)].u)] && inS[static_cast<std::size_t>(E[static_cast<std::size_t>(e)].v)])
                    cols.push_back(iy[static_cast<std::size_t>(e)]);
            cuts.row(cols, std::vector<double>(cols.size(), -1.0), -1.0);
            ++added;
            ++T.nCuts;
        }
        if (added == 0 || rnd == opts.maxCutRounds) break;
    }
    T.lb = S * c.dot(x);

    Path2 Xr(n, 2);
    for (Index v = 1; v <= n; ++v) {
        Vec2 p = Vec2::Zero();
        for (const Index e : eout[static_cast<std::size_t>(v)]) p += Vec2(x(izu[static_cast<std::size_t>(e)]), x(izu[static_cast<std::size_t>(e)] + 1));
        Xr.row(v - 1) = projectToSet(sets[static_cast<std::size_t>(v - 1)], S * p).transpose();
    }

    // ---- rounding -----------------------------------------------------------------------
    MatX Y = MatX::Zero(n + 2, n + 2);
    for (Index e = 0; e < nE; ++e)
        Y(E[static_cast<std::size_t>(e)].u, E[static_cast<std::size_t>(e)].v) = std::max(0.0, x(iy[static_cast<std::size_t>(e)]));
    std::mt19937_64 rng(opts.seed);
    std::vector<std::vector<Index>> orders;
    orders.push_back(walk(Y, n, false, Xr, pStart, rng));
    for (int k = 0; k < opts.nRound; ++k) orders.push_back(walk(Y, n, true, Xr, pStart, rng));
    orders.push_back(nearestOrder(Xr, pStart));
    std::vector<std::vector<Index>> uniq;
    std::set<std::vector<Index>> seen;
    for (const auto& o : orders)
        if (seen.insert(o).second) uniq.push_back(o);
    std::vector<double> est;
    for (const auto& o : uniq) est.push_back(routeLen(rowsOf(Xr, o), pStart, pGoal));
    std::vector<std::size_t> rk(uniq.size());
    std::iota(rk.begin(), rk.end(), 0);
    std::stable_sort(rk.begin(), rk.end(), [&](std::size_t a, std::size_t b) { return est[a] < est[b]; });

    auto solveOrder = [&](const std::vector<Index>& o, Path2& pts, double& len) {
        std::vector<ConvexSet> so;
        std::vector<double> wo;
        for (const Index i : o) {
            so.push_back(sets[static_cast<std::size_t>(i)]);
            wo.push_back(w[static_cast<std::size_t>(i)]);
        }
        std::vector<Index> id(o.size());
        std::iota(id.begin(), id.end(), 0);
        const FixedOrderResult F = fixedOrderRoute(so, id, pStart, pGoal, wo, opts);
        ++T.nSolves;
        pts = F.pts;
        len = F.len;
    };
    double bestLen = kInf;
    std::vector<Index> bestO;
    Path2 bestP;
    for (std::size_t q = 0; q < std::min<std::size_t>(static_cast<std::size_t>(opts.nExact), rk.size()); ++q) {
        Path2 p;
        double len;
        solveOrder(uniq[rk[q]], p, len);
        if (len < bestLen) {
            bestLen = len;
            bestO = uniq[rk[q]];
            bestP = p;
        }
    }
    if (opts.polish) {
        for (int r = 0; r < opts.polishRounds; ++r) {
            Path2 Xp(n, 2);
            for (std::size_t k = 0; k < bestO.size(); ++k) Xp.row(bestO[k]) = bestP.row(static_cast<Index>(k));
            const std::vector<Index> o2 = localSearch(bestO, Xp, pStart, pGoal);
            if (o2 == bestO) break;
            Path2 p;
            double len;
            solveOrder(o2, p, len);
            if (len < bestLen - 1e-6) {
                bestLen = len;
                bestO = o2;
                bestP = p;
            } else {
                break;
            }
        }
    }
    T.order = bestO;
    T.pts = bestP;
    T.len = bestLen;
    T.gap = T.len / std::max(T.lb, 1e-300) - 1.0;
    done();
    return T;
}

// =====================================================================================
//  Orienteering over convex sets (GcsMode::Orienteering) - see gcs.hpp.
// =====================================================================================
AgentGcsResult planAgentGcsOrienteering(const std::vector<ConvexSet>& allSets, const VecX& allInfo,
                                        const std::vector<Index>& ids, const Vec2& pStart,
                                        const std::optional<Vec2>& pGoal, double budget, double Rmin,
                                        const GcsParams& opts,
                                        const std::vector<std::vector<Index>>* warm) {
    const auto t0 = std::chrono::steady_clock::now();
    AgentGcsResult R;
    const Index n = static_cast<Index>(ids.size());
    auto finish = [&]() {
        R.time = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        return R;
    };
    R.wp.resize(0, 2);
    if (n == 0) return finish();
    std::vector<ConvexSet> sets;
    VecX info(n);
    for (Index i = 0; i < n; ++i) {
        sets.push_back(allSets[static_cast<std::size_t>(ids[static_cast<std::size_t>(i)])]);
        info(i) = allInfo(ids[static_cast<std::size_t>(i)]);
    }
    const double iScale = std::max(info.maxCoeff(), 1e-300);
    const VecX iv = info / iScale;
    const double S = opts.scale;
    const bool hasGoal = pGoal.has_value();
    const Vec2 s0 = pStart / S;
    const Vec2 g0 = hasGoal ? Vec2(*pGoal / S) : Vec2::Zero();
    auto fullLen = [&](const Path2& P) { return routeLength(P, pStart, pGoal) + turnOverhead(P, pStart, pGoal, Rmin); };

    // ---- edges: 0 = source, 1..n sets, n+1 = terminal (goal or free end) ------------
    auto U = [](Index i) { return static_cast<std::size_t>(i); };
    struct Edge { Index u, v; };
    std::vector<Edge> E;
    for (Index i = 1; i <= n; ++i) E.push_back({0, i});
    {
        // set-to-set edges: complete, or the symmetrised k nearest neighbours
        std::vector<std::vector<char>> adj(U(n), std::vector<char>(U(n), opts.knn <= 0 || opts.knn >= n - 1 ? 1 : 0));
        if (opts.knn > 0 && opts.knn < n - 1) {
            for (Index i = 0; i < n; ++i) {
                std::vector<std::pair<double, Index>> d;
                for (Index j = 0; j < n; ++j)
                    if (j != i) d.emplace_back((sets[U(i)].c - sets[U(j)].c).squaredNorm(), j);
                std::partial_sort(d.begin(), d.begin() + opts.knn, d.end());
                for (int q = 0; q < opts.knn; ++q) adj[U(i)][U(d[U(q)].second)] = adj[U(d[U(q)].second)][U(i)] = 1;
            }
        }
        for (Index i = 1; i <= n; ++i)
            for (Index j = 1; j <= n; ++j)
                if (i != j && adj[U(i - 1)][U(j - 1)]) E.push_back({i, j});
    }
    for (Index i = 1; i <= n; ++i) E.push_back({i, n + 1});
    E.push_back({0, n + 1});                                   // the empty route
    const Index nE = static_cast<Index>(E.size());
    std::vector<Index> iy(U(nE)), izu(U(nE), -1), izv(U(nE), -1), it(U(nE), -1), ip(U(n));
    Index nv = 0;
    for (Index e = 0; e < nE; ++e) iy[U(e)] = nv++;
    for (Index e = 0; e < nE; ++e) if (E[U(e)].u >= 1) { izu[U(e)] = nv; nv += 2; }
    for (Index e = 0; e < nE; ++e) if (E[U(e)].v <= n) { izv[U(e)] = nv; nv += 2; }
    for (Index e = 0; e < nE; ++e) if (E[U(e)].v <= n || hasGoal) it[U(e)] = nv++;
    for (Index v = 0; v < n; ++v) ip[U(v)] = nv++;
    // coverage-aware reward: info_v (p_v - cw |x_v - p_v c_v| / r_v), concave,
    // so the relaxation stays an SOCP (r_v = the set's circumradius)
    const bool pull = opts.centerWeight > 0.0;
    std::vector<Index> ir(U(n), -1);
    if (pull) for (Index v = 0; v < n; ++v) ir[U(v)] = nv++;
    std::vector<std::vector<Index>> ein(U(n + 2)), eout(U(n + 2));
    for (Index e = 0; e < nE; ++e) { eout[U(E[U(e)].u)].push_back(e); ein[U(E[U(e)].v)].push_back(e); }
    auto ys = [&](const std::vector<Index>& es) { std::vector<Index> c; for (const Index e : es) c.push_back(iy[U(e)]); return c; };

    Block Z, Lb, Q;
    std::vector<int> qs;
    for (Index v = 1; v <= n; ++v) {
        for (const auto* es : {&ein[U(v)], &eout[U(v)]}) {
            std::vector<Index> cols = ys(*es);
            std::vector<double> vals(cols.size(), 1.0);
            cols.push_back(ip[U(v - 1)]);
            vals.push_back(-1.0);
            Z.row(cols, vals, 0.0);                            // degree = p_v
        }
    }
    Z.row(ys(eout[0]), std::vector<double>(eout[0].size(), 1.0), 1.0);
    for (Index v = 1; v <= n; ++v)
        for (int d = 0; d < 2; ++d) {
            std::vector<Index> cols;
            std::vector<double> vals;
            for (const Index e : ein[U(v)]) { cols.push_back(izv[U(e)] + d); vals.push_back(1.0); }
            for (const Index e : eout[U(v)]) { cols.push_back(izu[U(e)] + d); vals.push_back(-1.0); }
            Z.row(cols, vals, 0.0);
        }
    for (Index e = 0; e < nE; ++e) Lb.row({iy[U(e)]}, {-1.0}, 0.0);
    for (Index v = 0; v < n; ++v) { Lb.row({ip[U(v)]}, {1.0}, 1.0); Lb.row({ip[U(v)]}, {-1.0}, 0.0); }
    for (Index e = 0; e < nE; ++e) {
        const Edge& ed = E[U(e)];
        if (ed.u >= 1) {
            const ConvexSet& C = sets[U(ed.u - 1)];
            for (Index r = 0; r < C.H.rows(); ++r)
                Lb.row({izu[U(e)], izu[U(e)] + 1, iy[U(e)]}, {C.H(r, 0), C.H(r, 1), -C.h(r) / S}, 0.0);
        }
        if (ed.v <= n) {
            const ConvexSet& C = sets[U(ed.v - 1)];
            for (Index r = 0; r < C.H.rows(); ++r)
                Lb.row({izv[U(e)], izv[U(e)] + 1, iy[U(e)]}, {C.H(r, 0), C.H(r, 1), -C.h(r) / S}, 0.0);
        }
    }
    {   // budget on the relaxed length (the turning is checked after rounding)
        std::vector<Index> cols;
        for (Index e = 0; e < nE; ++e) if (it[U(e)] >= 0) cols.push_back(it[U(e)]);
        Lb.row(cols, std::vector<double>(cols.size(), 1.0), budget / S);
    }
    for (Index e = 0; e < nE; ++e) {
        if (it[U(e)] < 0) continue;
        const Edge& ed = E[U(e)];
        Q.row({it[U(e)]}, {-1.0}, 0.0);
        for (int d = 0; d < 2; ++d) {
            std::vector<Index> cols;
            std::vector<double> vals;
            double ycoef = 0.0;
            if (ed.v <= n) { cols.push_back(izv[U(e)] + d); vals.push_back(-1.0); } else ycoef -= g0(d);
            if (ed.u >= 1) { cols.push_back(izu[U(e)] + d); vals.push_back(1.0); } else ycoef += s0(d);
            if (ycoef != 0.0 || (ed.u == 0 && ed.v > n)) { cols.push_back(iy[U(e)]); vals.push_back(ycoef); }
            Q.row(cols, vals, 0.0);
        }
        qs.push_back(3);
    }
    if (pull)
        for (Index v = 1; v <= n; ++v) {
            Q.row({ir[U(v - 1)]}, {-1.0}, 0.0);
            for (int d = 0; d < 2; ++d) {
                std::vector<Index> cols;
                std::vector<double> vals;
                for (const Index e : eout[U(v)]) { cols.push_back(izu[U(e)] + d); vals.push_back(-1.0); }
                cols.push_back(ip[U(v - 1)]);
                vals.push_back(sets[U(v - 1)].c(d) / S);          // s = x_v - p_v c_v
                Q.row(cols, vals, 0.0);
            }
            qs.push_back(3);
        }
    VecX c = VecX::Zero(nv);
    for (Index v = 0; v < n; ++v) c(ip[U(v)]) = -iv(v);
    if (pull)
        for (Index v = 0; v < n; ++v) {
            const ConvexSet& C = sets[U(v)];
            double rv = 0.0;
            for (Index i = 0; i < C.V.rows(); ++i) rv = std::max(rv, (Vec2(C.V.row(i).transpose()) - C.c).norm());
            c(ir[U(v)]) = opts.centerWeight * iv(v) / std::max(rv / S, 1e-6);
        }
    for (Index e = 0; e < nE; ++e) if (it[U(e)] >= 0) c(it[U(e)]) = opts.opLengthWeight;

    // ---- relaxation + generalised cutsets: y(delta-(S)) >= p_v, v in S ---------------
    Block cuts;
    VecX x;
    std::set<std::pair<std::vector<int>, int>> seen;
    double lbObj = 0.0;
    for (int rnd = 0; rnd <= opts.maxCutRounds; ++rnd) {
        Block L = Lb;
        for (const Trip& tr : cuts.t) L.t.emplace_back(tr.row() + Lb.m, tr.col(), tr.value());
        L.b.insert(L.b.end(), cuts.b.begin(), cuts.b.end());
        L.m += cuts.m;
        const ConicResult cr = solveConic(assemble(c, Z, L, Q, qs, nv), opts);
        ++R.nSolves;
        x = cr.x;
        lbObj = c.dot(x);
        if (!x.allFinite()) break;
        const int N = static_cast<int>(n + 2);
        MatX Cap = MatX::Zero(N, N);
        for (Index e = 0; e < nE; ++e) Cap(E[U(e)].u, E[U(e)].v) += std::max(0.0, x(iy[U(e)]));
        int added = 0;
        for (Index v = 1; v <= n; ++v) {
            const double pv = x(ip[U(v - 1)]);
            if (pv < 1e-3) continue;
            std::vector<char> reach;
            const double f = maxflow(Cap, 0, static_cast<int>(v), reach);
            if (f >= pv - 1e-4) continue;
            std::vector<int> Sset;
            for (int u = 0; u < N; ++u) if (!reach[U(u)]) Sset.push_back(u);
            if (!seen.insert({Sset, static_cast<int>(v)}).second) continue;
            std::vector<char> inS(U(N), 0);
            for (const int u : Sset) inS[U(u)] = 1;
            std::vector<Index> cols;
            std::vector<double> vals;
            for (Index e = 0; e < nE; ++e)
                if (!inS[U(E[U(e)].u)] && inS[U(E[U(e)].v)]) { cols.push_back(iy[U(e)]); vals.push_back(-1.0); }
            cols.push_back(ip[U(v - 1)]);
            vals.push_back(1.0);
            cuts.row(cols, vals, 0.0);
            ++added;
        }
        if (added == 0 || rnd == opts.maxCutRounds) break;
    }
    R.lb = -lbObj * iScale;    // relaxation's information (an UPPER bound on what a route collects)

    // relaxation points (per unit visit)
    Path2 Xr(n, 2);
    for (Index v = 1; v <= n; ++v) {
        const double pv = x.allFinite() ? x(ip[U(v - 1)]) : 0.0;
        Vec2 p = Vec2::Zero();
        if (pv > 1e-6) for (const Index e : eout[U(v)]) p += Vec2(x(izu[U(e)]), x(izu[U(e)] + 1));
        Xr.row(v - 1) = (pv > 1e-6 ? projectToSet(sets[U(v - 1)], S * p / pv) : sets[U(v - 1)].c).transpose();
    }
    MatX Y = MatX::Zero(n + 2, n + 2);
    if (x.allFinite())
        for (Index e = 0; e < nE; ++e) Y(E[U(e)].u, E[U(e)].v) = std::max(0.0, x(iy[U(e)]));

    // ---- rounding: walk the flow until the terminal ---------------------------------
    std::mt19937_64 rng(opts.seed);
    std::uniform_real_distribution<double> Ud(0.0, 1.0);
    auto walkOP = [&](bool random) {
        std::vector<Index> o;
        std::vector<char> vis(U(n), 0);
        Index u = 0;
        while (true) {
            std::vector<Index> cand;
            std::vector<double> w;
            for (Index v = 0; v < n; ++v) if (!vis[U(v)] && Y(u, v + 1) > 1e-6) { cand.push_back(v); w.push_back(Y(u, v + 1)); }
            cand.push_back(-1);
            w.push_back(Y(u, n + 1) + 1e-9);
            std::size_t j = 0;
            if (random) {
                const double tot = std::accumulate(w.begin(), w.end(), 0.0), r = Ud(rng) * tot;
                double acc = 0.0;
                j = w.size() - 1;
                for (std::size_t i = 0; i < w.size(); ++i) { acc += w[i]; if (r <= acc) { j = i; break; } }
            } else {
                j = static_cast<std::size_t>(std::max_element(w.begin(), w.end()) - w.begin());
            }
            if (cand[j] < 0) break;
            o.push_back(cand[j]);
            vis[U(cand[j])] = 1;
            u = cand[j] + 1;
        }
        return o;
    };
    auto solveOrder = [&](const std::vector<Index>& o, Path2& pts, double& len) {
        std::vector<ConvexSet> so;
        for (const Index i : o) so.push_back(sets[U(i)]);
        std::vector<Index> id(o.size());
        std::iota(id.begin(), id.end(), 0);
        std::vector<double> wo;
        if (pull) {
            const double im = info.mean();
            for (const Index i : o) wo.push_back(opts.centerWeight * info(i) / std::max(im, 1e-300));
        }
        const FixedOrderResult F = fixedOrderRoute(so, id, pStart, pGoal, wo, opts);
        ++R.nSolves;
        pts = F.pts;
        len = fullLen(pts);
    };
    auto infoOf = [&](const std::vector<Index>& o) { double s2 = 0; for (const Index i : o) s2 += info(i); return s2; };
    // make an order feasible: drop by information per metre saved, re-solving
    auto repair = [&](std::vector<Index>& o, Path2& pts, double& len) {
        solveOrder(o, pts, len);
        while (len > budget && !o.empty()) {
            std::size_t jb = 0;
            double best = kInf;
            for (std::size_t i = 0; i < o.size(); ++i) {
                Path2 Pi(pts.rows() - 1, 2);
                for (Index r = 0, q = 0; r < pts.rows(); ++r) if (r != static_cast<Index>(i)) Pi.row(q++) = pts.row(r);
                const double sc = info(o[i]) / std::max(len - fullLen(Pi), 1e-9);
                if (sc < best) { best = sc; jb = i; }
            }
            o.erase(o.begin() + static_cast<std::ptrdiff_t>(jb));
            solveOrder(o, pts, len);
        }
    };
    // add sets by best information per metre of cheapest insertion while it fits
    auto augment = [&](std::vector<Index>& o, Path2& pts, double& len) {
        bool grew = true;
        while (grew) {
            grew = false;
            std::vector<char> in(U(n), 0);
            for (const Index i : o) in[U(i)] = 1;
            double bestSc = 0.0;
            std::vector<Index> bestO;
            for (Index v = 0; v < n; ++v) {
                if (in[U(v)]) continue;
                for (std::size_t pos = 0; pos <= o.size(); ++pos) {
                    Path2 P2(pts.rows() + 1, 2);
                    for (Index r = 0, q = 0; r <= pts.rows(); ++r) {
                        if (r == static_cast<Index>(pos)) P2.row(r) = Xr.row(v);
                        else P2.row(r) = pts.row(q++);
                    }
                    const double l2 = fullLen(P2);
                    if (l2 > budget) continue;
                    const double sc = info(v) / std::max(l2 - len, 1.0);
                    if (sc > bestSc) {
                        bestSc = sc;
                        bestO = o;
                        bestO.insert(bestO.begin() + static_cast<std::ptrdiff_t>(pos), v);
                    }
                }
            }
            if (!bestO.empty()) {
                Path2 p2;
                double l2;
                solveOrder(bestO, p2, l2);
                if (l2 <= budget) { o = bestO; pts = p2; len = l2; grew = true; }
            }
        }
    };

    std::vector<std::vector<Index>> walks;
    std::set<std::vector<Index>> seenW;
    for (int k = 0; k <= opts.nRound; ++k) {
        const std::vector<Index> o = walkOP(k > 0);
        if (seenW.insert(o).second) walks.push_back(o);
    }
    std::vector<double> est;
    for (const auto& o : walks) est.push_back(infoOf(o) - 1e-9 * routeLen(rowsOf(Xr, o), pStart, pGoal));
    std::vector<std::size_t> rk(walks.size());
    std::iota(rk.begin(), rk.end(), 0);
    std::stable_sort(rk.begin(), rk.end(), [&](std::size_t a, std::size_t b) { return est[a] > est[b]; });
    double bestInfo = -1.0, bestLen = kInf;
    std::vector<Index> bestO;
    Path2 bestP(0, 2);
    // orders evaluated exactly: the nExact best flow walks, plus the warm orders
    // (e.g. the orienteering heuristic's), so the conic route can only improve
    // on the heuristic incumbent rather than ignore it
    std::vector<std::vector<Index>> evalList;
    for (std::size_t q = 0; q < std::min<std::size_t>(static_cast<std::size_t>(opts.nExact), rk.size()); ++q)
        evalList.push_back(walks[rk[q]]);
    if (warm)
        for (const std::vector<Index>& wo : *warm) {
            std::vector<Index> loc;
            for (const Index g : wo) {
                const auto itg = std::find(ids.begin(), ids.end(), g);
                if (itg != ids.end()) loc.push_back(static_cast<Index>(itg - ids.begin()));
            }
            if (!loc.empty() && seenW.insert(loc).second) evalList.push_back(loc);
        }
    for (std::size_t q = 0; q < evalList.size(); ++q) {
        std::vector<Index> o = evalList[q];
        Path2 p;
        double len;
        repair(o, p, len);
        augment(o, p, len);
        if (opts.polish && o.size() > 2) {
            for (int r = 0; r < opts.polishRounds; ++r) {
                Path2 Xp(n, 2);
                Xp = Xr;
                for (std::size_t k = 0; k < o.size(); ++k) Xp.row(o[k]) = p.row(static_cast<Index>(k));
                const std::vector<Index> o2 = localSearch(o, Xp, pStart, pGoal);
                if (o2 == o) break;
                Path2 p2;
                double l2;
                solveOrder(o2, p2, l2);
                if (l2 < len - 1e-6) { o = o2; p = p2; len = l2; augment(o, p, len); } else break;
            }
        }
        const double inf = infoOf(o);
        if (inf > bestInfo + 1e-12 || (std::abs(inf - bestInfo) <= 1e-12 && len < bestLen)) {
            bestInfo = inf; bestLen = len; bestO = o; bestP = p;
        }
    }
    for (const Index i : bestO) R.clusters.push_back(ids[U(i)]);
    for (Index i = 0; i < n; ++i)
        if (std::find(bestO.begin(), bestO.end(), i) == bestO.end()) R.dropped.push_back(ids[U(i)]);
    R.wp = bestP;
    R.len = bestLen;
    R.lenEuclid = routeLength(bestP, pStart, pGoal);
    R.gap = R.lb > 0 ? 1.0 - bestInfo / R.lb : 0.0;    // information gap to the relaxation
    return finish();
}

// =====================================================================================
//  Coverage GCS (GcsMode::Coverage) - see gcs.hpp.
// =====================================================================================
namespace {

std::vector<Vec2> clipPoly(const std::vector<Vec2>& V, const Vec2& a, double b) {
    std::vector<Vec2> out;
    const std::size_t n = V.size();
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t j = (i + 1) % n;
        const double fi = a.dot(V[i]) - b, fj = a.dot(V[j]) - b;
        if (fi <= 0.0) out.push_back(V[i]);
        if ((fi < 0.0 && fj > 0.0) || (fi > 0.0 && fj < 0.0)) out.push_back(V[i] + fi / (fi - fj) * (V[j] - V[i]));
    }
    std::vector<Vec2> dd;
    for (std::size_t i = 0; i < out.size(); ++i)
        if ((out[(i + 1) % out.size()] - out[i]).norm() > 1e-9) dd.push_back(out[i]);
    return dd;
}

double polyArea(const std::vector<Vec2>& V) {
    double A = 0.0;
    for (std::size_t i = 0; i < V.size(); ++i) {
        const Vec2& p = V[i];
        const Vec2& q = V[(i + 1) % V.size()];
        A += p.x() * q.y() - q.x() * p.y();
    }
    return 0.5 * A;
}

}  // namespace

VecX CoverageCells::cellMass(const MatX& W) const {
    VecX m = VecX::Zero(static_cast<Index>(sets.size()));
    for (Index c = 0; c < pixelCell.cols(); ++c)
        for (Index r = 0; r < pixelCell.rows(); ++r)
            if (pixelCell(r, c) >= 0) m(pixelCell(r, c)) += W(r, c);
    return m;
}

CoverageCells coverageCells(const FastGrid& G, const SwathKernel& K, double cellFrac, const Eigen::Vector4d& bounds) {
    CoverageCells C;
    const double hw = K.halfWidth;
    C.rho = cellFrac * hw;
    C.spacing = std::sqrt(3.0) * C.rho;
    const double dy = 1.5 * C.rho;
    std::vector<Vec2> lat;   // lattice centres of the kept cells
    std::vector<std::vector<Vec2>> polys;
    for (int j = 0;; ++j) {
        const double y = bounds(2) + j * dy;
        if (y > bounds(3) + C.rho) break;
        const double off = (j % 2) ? 0.5 * C.spacing : 0.0;
        for (int i = -1;; ++i) {
            const double x = bounds(0) + off + i * C.spacing;
            if (x > bounds(1) + C.spacing) break;
            std::vector<Vec2> V;
            for (int k = 0; k < 6; ++k) {
                const double a = kPi / 6.0 + k * kPi / 3.0;
                V.emplace_back(x + C.rho * std::cos(a), y + C.rho * std::sin(a));
            }
            V = clipPoly(V, Vec2(-1, 0), -bounds(0));
            V = clipPoly(V, Vec2(1, 0), bounds(1));
            V = clipPoly(V, Vec2(0, -1), -bounds(2));
            V = clipPoly(V, Vec2(0, 1), bounds(3));
            if (V.size() < 3 || polyArea(V) < 0.05 * 2.598 * C.rho * C.rho) continue;
            lat.emplace_back(x, y);
            polys.push_back(V);
        }
    }
    const Index n = static_cast<Index>(lat.size());
    C.centers.resize(n, 2);
    C.sets.resize(static_cast<std::size_t>(n));
    for (Index k = 0; k < n; ++k) {
        const std::vector<Vec2>& V = polys[static_cast<std::size_t>(k)];
        ConvexSet& S = C.sets[static_cast<std::size_t>(k)];
        S.V.resize(static_cast<Index>(V.size()), 2);
        Vec2 cen = Vec2::Zero();
        std::vector<Vec2> Hn;
        std::vector<double> hn;
        for (std::size_t i = 0; i < V.size(); ++i) {
            S.V.row(static_cast<Index>(i)) = V[i].transpose();
            cen += V[i] / static_cast<double>(V.size());
            const Vec2 e = V[(i + 1) % V.size()] - V[i];
            Vec2 nr(e.y(), -e.x());
            if (nr.norm() <= 1e-9) continue;
            nr.normalize();
            Hn.push_back(nr);
            hn.push_back(nr.dot(V[i]));
        }
        S.H.resize(static_cast<Index>(Hn.size()), 2);
        S.h.resize(static_cast<Index>(hn.size()));
        for (std::size_t i = 0; i < Hn.size(); ++i) {
            S.H.row(static_cast<Index>(i)) = Hn[i].transpose();
            S.h(static_cast<Index>(i)) = hn[i];
        }
        S.c = cen;
        S.cheb = std::max(0.0, (S.h - S.H * cen).minCoeff());
        C.centers.row(k) = cen.transpose();
    }
    C.nbr.assign(static_cast<std::size_t>(n), {});
    for (Index a = 0; a < n; ++a)
        for (Index b = 0; b < n; ++b)
            if (a != b && (lat[static_cast<std::size_t>(a)] - lat[static_cast<std::size_t>(b)]).norm() < 1.1 * C.spacing)
                C.nbr[static_cast<std::size_t>(a)].push_back(b);
    // pixel -> nearest lattice centre
    C.pixelCell = Eigen::MatrixXi::Constant(G.ny, G.nx, -1);
    for (Index c = 0; c < G.nx; ++c)
        for (Index r = 0; r < G.ny; ++r) {
            const Vec2 p(G.x(c), G.y(r));
            double bd = kInf;
            int bk = -1;
            for (Index k = 0; k < n; ++k) {
                const double d = (lat[static_cast<std::size_t>(k)] - p).squaredNorm();
                if (d < bd) { bd = d; bk = static_cast<int>(k); }
            }
            C.pixelCell(r, c) = bk;
        }
    // straight-pass detection profile of the fast model
    auto Pdet = [&](double d) {
        const VecX& D = K.d;
        const VecX& L = K.lambdaModel.size() ? K.lambdaModel : K.lambdaEff;
        if (D.size() == 0) return d <= hw ? 1.0 : 0.0;
        if (d <= D(0)) return 1.0 - std::exp(L(0));
        for (Index i = 1; i < D.size(); ++i)
            if (d <= D(i)) {
                const double t = (d - D(i - 1)) / (D(i) - D(i - 1));
                return 1.0 - std::exp((1 - t) * L(i - 1) + t * L(i));
            }
        return 0.0;
    };
    std::vector<Eigen::Triplet<double>> tr;
    for (Index a = 0; a < n; ++a)
        for (Index b = 0; b < n; ++b) {
            const double d = (Vec2(C.centers.row(a).transpose()) - Vec2(C.centers.row(b).transpose())).norm();
            if (d > K.reach + C.rho) continue;
            const double pc = Pdet(d);
            if (pc >= 0.05) tr.emplace_back(static_cast<int>(a), static_cast<int>(b), pc);
        }
    C.cover.resize(n, n);
    C.cover.setFromTriplets(tr.begin(), tr.end());
    C.cover.makeCompressed();
    return C;
}

AgentGcsResult planAgentGcsCoverage(const CoverageCells& Cc, const VecX& infoAll, const Vec2& pStart,
                                    const std::optional<Vec2>& pGoal, double budget, double Rmin,
                                    const GcsParams& opts) {
    const auto t0 = std::chrono::steady_clock::now();
    AgentGcsResult R;
    R.wp.resize(0, 2);
    auto U = [](Index i) { return static_cast<std::size_t>(i); };
    const Index nAll = static_cast<Index>(Cc.sets.size());
    // vertices: cells the budget can reach
    std::vector<Index> ids, loc(U(nAll), -1);
    for (Index k = 0; k < nAll; ++k)
        if ((Vec2(Cc.centers.row(k).transpose()) - pStart).norm() <= budget + Cc.rho) {
            loc[U(k)] = static_cast<Index>(ids.size());
            ids.push_back(k);
        }
    const Index n = static_cast<Index>(ids.size());
    auto finish = [&]() {
        R.time = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        return R;
    };
    if (n == 0) return finish();
    // coverage targets: cells with belief that a vertex can cover
    std::vector<Index> tg, tloc(U(nAll), -1);
    for (Index c = 0; c < nAll; ++c) {
        if (!(infoAll(c) > 0.0)) continue;
        bool reach = false;
        for (const Index v : ids)
            if (Cc.cover.coeff(c, v) > 0.0) { reach = true; break; }
        if (reach) { tloc[U(c)] = static_cast<Index>(tg.size()); tg.push_back(c); }
    }
    const Index nt = static_cast<Index>(opts.coverageOwnOnly ? 0 : tg.size());
    const double iScale = std::max(infoAll.maxCoeff(), 1e-300);
    // the sets the route points live in: the cells, shrunk about their centres
    std::vector<ConvexSet> pset(U(nAll));
    for (Index k = 0; k < nAll; ++k) {
        ConvexSet Sk = Cc.sets[U(k)];
        const double f = std::min(1.0, std::max(1e-3, opts.coveragePointFrac));
        Sk.h = Sk.H * Sk.c + f * (Sk.h - Sk.H * Sk.c);
        Sk.V = (Sk.V.rowwise() - Sk.c.transpose()) * f;
        Sk.V.rowwise() += Sk.c.transpose();
        pset[U(k)] = Sk;
    }
    const double S = opts.scale;
    const bool hasGoal = pGoal.has_value();
    const Vec2 s0 = pStart / S;
    const Vec2 g0 = hasGoal ? Vec2(*pGoal / S) : Vec2::Zero();
    auto fullLen = [&](const Path2& P) { return routeLength(P, pStart, pGoal) + turnOverhead(P, pStart, pGoal, Rmin); };

    // ---- edges ---------------------------------------------------------------------------
    struct Edge { Index u, v; };
    std::vector<Edge> E;
    Index kStart = 0;
    {
        double bd = kInf;
        for (Index i = 0; i < n; ++i) {
            const double d = (Vec2(Cc.centers.row(ids[U(i)]).transpose()) - pStart).norm();
            if (d < bd) { bd = d; kStart = i; }
        }
        E.push_back({0, kStart + 1});
        for (const Index b : Cc.nbr[U(ids[U(kStart)])])
            if (loc[U(b)] >= 0) E.push_back({0, loc[U(b)] + 1});
    }
    for (Index i = 0; i < n; ++i)
        for (const Index b : Cc.nbr[U(ids[U(i)])])
            if (loc[U(b)] >= 0) E.push_back({i + 1, loc[U(b)] + 1});
    for (Index i = 0; i < n; ++i) E.push_back({i + 1, n + 1});
    E.push_back({0, n + 1});
    const Index nE = static_cast<Index>(E.size());
    std::vector<Index> iy(U(nE)), izu(U(nE), -1), izv(U(nE), -1), it(U(nE), -1), ip(U(n)), iq(U(nt));
    Index nv = 0;
    for (Index e = 0; e < nE; ++e) iy[U(e)] = nv++;
    for (Index e = 0; e < nE; ++e) if (E[U(e)].u >= 1) { izu[U(e)] = nv; nv += 2; }
    for (Index e = 0; e < nE; ++e) if (E[U(e)].v <= n) { izv[U(e)] = nv; nv += 2; }
    for (Index e = 0; e < nE; ++e) if (E[U(e)].v <= n || hasGoal) it[U(e)] = nv++;
    for (Index v = 0; v < n; ++v) ip[U(v)] = nv++;
    for (Index c = 0; c < nt; ++c) iq[U(c)] = nv++;
    std::vector<std::vector<Index>> ein(U(n + 2)), eout(U(n + 2));
    for (Index e = 0; e < nE; ++e) { eout[U(E[U(e)].u)].push_back(e); ein[U(E[U(e)].v)].push_back(e); }

    Block Z, Lb, Q;
    std::vector<int> qs;
    for (Index v = 1; v <= n; ++v)
        for (const auto* es : {&ein[U(v)], &eout[U(v)]}) {
            std::vector<Index> cols;
            std::vector<double> vals;
            for (const Index e : *es) { cols.push_back(iy[U(e)]); vals.push_back(1.0); }
            cols.push_back(ip[U(v - 1)]);
            vals.push_back(-1.0);
            Z.row(cols, vals, 0.0);
        }
    {
        std::vector<Index> cols;
        for (const Index e : eout[0]) cols.push_back(iy[U(e)]);
        Z.row(cols, std::vector<double>(cols.size(), 1.0), 1.0);
    }
    for (Index v = 1; v <= n; ++v)
        for (int d = 0; d < 2; ++d) {
            std::vector<Index> cols;
            std::vector<double> vals;
            for (const Index e : ein[U(v)]) { cols.push_back(izv[U(e)] + d); vals.push_back(1.0); }
            for (const Index e : eout[U(v)]) { cols.push_back(izu[U(e)] + d); vals.push_back(-1.0); }
            Z.row(cols, vals, 0.0);
        }
    for (Index e = 0; e < nE; ++e) Lb.row({iy[U(e)]}, {-1.0}, 0.0);
    for (Index v = 0; v < n; ++v) { Lb.row({ip[U(v)]}, {1.0}, 1.0); Lb.row({ip[U(v)]}, {-1.0}, 0.0); }
    for (Index c = 0; c < nt; ++c) {
        Lb.row({iq[U(c)]}, {1.0}, 1.0);
        Lb.row({iq[U(c)]}, {-1.0}, 0.0);
        std::vector<Index> cols{iq[U(c)]};
        std::vector<double> vals{1.0};
        for (Index v = 0; v < n; ++v) {
            const double a = Cc.cover.coeff(tg[U(c)], ids[U(v)]);
            if (a > 0.0) { cols.push_back(ip[U(v)]); vals.push_back(-a); }
        }
        Lb.row(cols, vals, 0.0);                                  // q_c <= sum_v a_cv p_v
    }
    for (Index e = 0; e < nE; ++e) {
        const Edge& ed = E[U(e)];
        if (ed.u >= 1) {
            const ConvexSet& C = pset[U(ids[U(ed.u - 1)])];
            for (Index r = 0; r < C.H.rows(); ++r)
                Lb.row({izu[U(e)], izu[U(e)] + 1, iy[U(e)]}, {C.H(r, 0), C.H(r, 1), -C.h(r) / S}, 0.0);
        }
        if (ed.v <= n) {
            const ConvexSet& C = pset[U(ids[U(ed.v - 1)])];
            for (Index r = 0; r < C.H.rows(); ++r)
                Lb.row({izv[U(e)], izv[U(e)] + 1, iy[U(e)]}, {C.H(r, 0), C.H(r, 1), -C.h(r) / S}, 0.0);
        }
    }
    {
        std::vector<Index> cols;
        for (Index e = 0; e < nE; ++e) if (it[U(e)] >= 0) cols.push_back(it[U(e)]);
        Lb.row(cols, std::vector<double>(cols.size(), 1.0), budget / S);
    }
    for (Index e = 0; e < nE; ++e) {
        if (it[U(e)] < 0) continue;
        const Edge& ed = E[U(e)];
        Q.row({it[U(e)]}, {-1.0}, 0.0);
        for (int d = 0; d < 2; ++d) {
            std::vector<Index> cols;
            std::vector<double> vals;
            double yc = 0.0;
            if (ed.v <= n) { cols.push_back(izv[U(e)] + d); vals.push_back(-1.0); } else yc -= g0(d);
            if (ed.u >= 1) { cols.push_back(izu[U(e)] + d); vals.push_back(1.0); } else yc += s0(d);
            if (yc != 0.0 || (ed.u == 0 && ed.v > n)) { cols.push_back(iy[U(e)]); vals.push_back(yc); }
            Q.row(cols, vals, 0.0);
        }
        qs.push_back(3);
    }
    VecX c = VecX::Zero(nv);
    for (Index k = 0; k < nt; ++k) c(iq[U(k)]) = -infoAll(tg[U(k)]) / iScale;
    if (opts.coverageOwnOnly)
        for (Index v = 0; v < n; ++v) c(ip[U(v)]) = -infoAll(ids[U(v)]) / iScale;
    for (Index e = 0; e < nE; ++e) if (it[U(e)] >= 0) c(it[U(e)]) = opts.opLengthWeight;

    // ---- relaxation + generalised cutsets ---------------------------------------------------
    Block cuts;
    VecX x;
    std::set<std::pair<std::vector<int>, int>> seen;
    for (int rnd = 0; rnd <= opts.maxCutRounds; ++rnd) {
        Block L = Lb;
        for (const Trip& t2 : cuts.t) L.t.emplace_back(t2.row() + Lb.m, t2.col(), t2.value());
        L.b.insert(L.b.end(), cuts.b.begin(), cuts.b.end());
        L.m += cuts.m;
        const ConicResult cr = solveConic(assemble(c, Z, L, Q, qs, nv), opts);
        ++R.nSolves;
        x = cr.x;
        R.lb = -c.dot(x) * iScale;
        if (!x.allFinite()) break;
        const int N = static_cast<int>(n + 2);
        MatX Cap = MatX::Zero(N, N);
        for (Index e = 0; e < nE; ++e) Cap(E[U(e)].u, E[U(e)].v) += std::max(0.0, x(iy[U(e)]));
        int added = 0;
        for (Index v = 1; v <= n; ++v) {
            const double pv = x(ip[U(v - 1)]);
            if (pv < 1e-3) continue;
            std::vector<char> reach;
            const double f = maxflow(Cap, 0, static_cast<int>(v), reach);
            if (f >= pv - 1e-4) continue;
            std::vector<int> Sset;
            for (int u = 0; u < N; ++u) if (!reach[U(u)]) Sset.push_back(u);
            if (!seen.insert({Sset, static_cast<int>(v)}).second) continue;
            std::vector<char> inS(U(N), 0);
            for (const int u : Sset) inS[U(u)] = 1;
            std::vector<Index> cols;
            std::vector<double> vals;
            for (Index e = 0; e < nE; ++e)
                if (!inS[U(E[U(e)].u)] && inS[U(E[U(e)].v)]) { cols.push_back(iy[U(e)]); vals.push_back(-1.0); }
            cols.push_back(ip[U(v - 1)]);
            vals.push_back(1.0);
            cuts.row(cols, vals, 0.0);
            ++added;
        }
        if (added == 0 || rnd == opts.maxCutRounds) break;
    }

    // ---- rounding ---------------------------------------------------------------------------
    MatX Y = MatX::Zero(n + 2, n + 2);
    if (x.allFinite())
        for (Index e = 0; e < nE; ++e) Y(E[U(e)].u, E[U(e)].v) = std::max(0.0, x(iy[U(e)]));
    // true coverage of a set of cells (local indices): sum_c info_c (1 - prod (1 - a))
    auto covered = [&](const std::vector<Index>& o) {
        VecX miss = VecX::Ones(nAll);
        // cover is (c, v): walk column v = ids[v]
        for (const Index v : o)
            for (Eigen::SparseMatrix<double>::InnerIterator itc(Cc.cover, ids[U(v)]); itc; ++itc)
                miss(itc.row()) *= 1.0 - itc.value();
        double s2 = 0.0;
        for (Index k = 0; k < nAll; ++k) s2 += infoAll(k) * (1.0 - miss(k));
        return s2;
    };
    std::mt19937_64 rng(opts.seed);
    std::uniform_real_distribution<double> Ud(0.0, 1.0);
    auto walk2 = [&](bool random) {
        std::vector<Index> o;
        std::vector<char> vis(U(n), 0);
        Index u = 0;
        while (true) {
            std::vector<Index> cand;
            std::vector<double> w;
            for (const Index e : eout[U(u)]) {
                const Index v = E[U(e)].v;
                if (v <= n && !vis[U(v - 1)] && Y(u, v) > 1e-6) { cand.push_back(v - 1); w.push_back(Y(u, v)); }
            }
            cand.push_back(-1);
            w.push_back(Y(u, n + 1) + 1e-9);
            std::size_t j = 0;
            if (random) {
                const double tot = std::accumulate(w.begin(), w.end(), 0.0), r = Ud(rng) * tot;
                double acc = 0.0;
                j = w.size() - 1;
                for (std::size_t i = 0; i < w.size(); ++i) { acc += w[i]; if (r <= acc) { j = i; break; } }
            } else {
                j = static_cast<std::size_t>(std::max_element(w.begin(), w.end()) - w.begin());
            }
            if (cand[j] < 0) break;
            o.push_back(cand[j]);
            vis[U(cand[j])] = 1;
            u = cand[j] + 1;
        }
        return o;
    };
    auto solveOrder = [&](const std::vector<Index>& o, Path2& pts, double& len) {
        std::vector<ConvexSet> so;
        for (const Index i : o) so.push_back(pset[U(ids[U(i)])]);
        std::vector<Index> id(o.size());
        std::iota(id.begin(), id.end(), 0);
        const FixedOrderResult F = fixedOrderRoute(so, id, pStart, pGoal, {}, opts);
        ++R.nSolves;
        pts = F.pts;
        len = fullLen(pts);
    };
    auto repair = [&](std::vector<Index>& o, Path2& pts, double& len) {
        solveOrder(o, pts, len);
        while (len > budget && !o.empty()) {
            const double cv = covered(o);
            std::size_t jb = 0;
            double best = kInf;
            for (std::size_t i = 0; i < o.size(); ++i) {
                std::vector<Index> oi = o;
                oi.erase(oi.begin() + static_cast<std::ptrdiff_t>(i));
                Path2 Pi(pts.rows() - 1, 2);
                for (Index r = 0, q = 0; r < pts.rows(); ++r) if (r != static_cast<Index>(i)) Pi.row(q++) = pts.row(r);
                const double sc = (cv - covered(oi)) / std::max(len - fullLen(Pi), 1e-9);
                if (sc < best) { best = sc; jb = i; }
            }
            o.erase(o.begin() + static_cast<std::ptrdiff_t>(jb));
            solveOrder(o, pts, len);
        }
    };
    auto augment = [&](std::vector<Index>& o, Path2& pts, double& len) {
        for (int guard = 0; guard < 200; ++guard) {
            const double cv = covered(o);
            std::vector<char> in(U(n), 0);
            for (const Index i : o) in[U(i)] = 1;
            double bestSc = 0.0;
            std::vector<Index> bestO;
            for (Index v = 0; v < n; ++v) {
                if (in[U(v)]) continue;
                std::vector<Index> ov = o;
                ov.push_back(v);
                const double gain = covered(ov) - cv;
                if (gain <= 0.0) continue;
                const Vec2 pv = Cc.centers.row(ids[U(v)]).transpose();
                for (std::size_t pos = 0; pos <= o.size(); ++pos) {
                    Path2 P2(pts.rows() + 1, 2);
                    for (Index r = 0, q = 0; r <= pts.rows(); ++r) {
                        if (r == static_cast<Index>(pos)) P2.row(r) = pv.transpose();
                        else P2.row(r) = pts.row(q++);
                    }
                    const double l2 = fullLen(P2);
                    if (l2 > budget) continue;
                    const double sc = gain / std::max(l2 - len, 1.0);
                    if (sc > bestSc) {
                        bestSc = sc;
                        bestO = o;
                        bestO.insert(bestO.begin() + static_cast<std::ptrdiff_t>(pos), v);
                    }
                }
            }
            if (bestO.empty()) break;
            Path2 p2;
            double l2;
            solveOrder(bestO, p2, l2);
            if (l2 > budget) break;
            o = bestO;
            pts = p2;
            len = l2;
        }
    };
    std::vector<std::vector<Index>> walks;
    std::set<std::vector<Index>> seenW;
    for (int k = 0; k <= opts.nRound; ++k) {
        const std::vector<Index> o = walk2(k > 0);
        if (seenW.insert(o).second) walks.push_back(o);
    }
    std::vector<double> est;
    for (const auto& o : walks) est.push_back(covered(o));
    std::vector<std::size_t> rk(walks.size());
    std::iota(rk.begin(), rk.end(), 0);
    std::stable_sort(rk.begin(), rk.end(), [&](std::size_t a, std::size_t b) { return est[a] > est[b]; });
    double bestCov = -1.0, bestLen = kInf;
    std::vector<Index> bestO;
    Path2 bestP(0, 2);
    struct Alt { double cov; std::vector<Index> o; Path2 P; };
    std::vector<Alt> alts;
    for (std::size_t q = 0; q < std::min<std::size_t>(static_cast<std::size_t>(std::max(opts.nExact, opts.nSeeds)), rk.size()); ++q) {
        std::vector<Index> o = walks[rk[q]];
        Path2 p;
        double len;
        repair(o, p, len);
        augment(o, p, len);
        if (opts.polish && o.size() > 2) {
            for (int r = 0; r < opts.polishRounds; ++r) {
                Path2 Xp(n, 2);
                for (Index i = 0; i < n; ++i) Xp.row(i) = Cc.centers.row(ids[U(i)]);
                for (std::size_t k = 0; k < o.size(); ++k) Xp.row(o[k]) = p.row(static_cast<Index>(k));
                const std::vector<Index> o2 = localSearch(o, Xp, pStart, pGoal);
                if (o2 == o) break;
                Path2 p2;
                double l2;
                solveOrder(o2, p2, l2);
                if (l2 < len - 1e-6) { o = o2; p = p2; len = l2; augment(o, p, len); } else break;
            }
        }
        const double cv = covered(o);
        bool dup = false;
        for (const Alt& al : alts) dup = dup || al.o == o;
        if (!dup) alts.push_back({cv, o, p});
        if (cv > bestCov + 1e-12 || (std::abs(cv - bestCov) <= 1e-12 && len < bestLen)) {
            bestCov = cv; bestLen = len; bestO = o; bestP = p;
        }
    }
    std::stable_sort(alts.begin(), alts.end(), [](const Alt& a1, const Alt& a2) { return a1.cov > a2.cov; });
    for (const Alt& al : alts)
        if (al.o != bestO) R.altWp.push_back(al.P);
    for (const Index i : bestO) R.clusters.push_back(ids[U(i)]);
    R.wp = bestP;
    R.len = bestLen;
    R.lenEuclid = routeLength(bestP, pStart, pGoal);
    R.gap = R.lb > 0 ? 1.0 - bestCov / R.lb : 0.0;
    return finish();
}

}  // namespace mtl::curve::gcs
