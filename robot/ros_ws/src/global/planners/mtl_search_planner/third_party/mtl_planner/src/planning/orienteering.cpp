#include "mtl/planning/orienteering.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <random>

namespace mtl::planning {
namespace {

/// Insertion criterion used by the greedy construction.
enum class Criterion { Ratio, Reward2, Reward };

/// Internal indexing: 0 = the depot, 1+k = node k, and, on a closed path,
/// N-1 = the forced end.  The depot's prize is zero.
struct Problem {
    MatX              D;        ///< (N x N) Euclidean cost matrix
    VecX              r;        ///< (N) prize per internal index
    std::vector<char> mandatory;///< (K) per NODE, not per internal index
    std::vector<int>  cand;     ///< internal indices of the reachable nodes
    bool              closed = false;
    int               endIdx = -1;
    double            budget = kInf;
};

double pathLength(const std::vector<int>& tour, const MatX& D) {
    double len = 0.0;
    for (std::size_t i = 0; i + 1 < tour.size(); ++i) len += D(tour[i], tour[i + 1]);
    return len;
}

/// Cheapest insertion of EVERY pool node, in one sweep.  deltas[k] is the extra
/// length of splicing pool[k] in AFTER tour position poss[k].  The depot is
/// never displaced, and on a closed path neither is the forced end.
void allInsertions(const std::vector<int>& tour, const std::vector<int>& pool, const MatX& D,
                   int endIdx, std::vector<double>& deltas, std::vector<std::size_t>& poss) {
    const std::size_t n  = tour.size();
    const std::size_t np = pool.size();
    deltas.assign(np, kInf);
    poss.assign(np, 0);
    if (n == 0) return;

    // Open path: appending after the final node is allowed.  Closed: the new
    // node must stay before the forced end.
    const std::size_t lastPos = (endIdx < 0) ? (n - 1) : (n >= 2 ? n - 2 : 0);
    if ((endIdx >= 0 && n < 2)) return;

    for (std::size_t k = 0; k < np; ++k) {
        const int j = pool[k];
        double best = kInf;
        std::size_t bestP = 0;
        for (std::size_t p = 0; p <= lastPos; ++p) {
            const int a = tour[p];
            double d;
            if (p + 1 < n) {
                const int b = tour[p + 1];
                d = D(a, j) + D(j, b) - D(a, b);
            } else {
                d = D(a, j);  // the open-path append slot
            }
            if (d < best) {
                best  = d;
                bestP = p;
            }
        }
        deltas[k] = best;
        poss[k]   = bestP;
    }
}

/// Insert from `pool`, best prize per metre of detour, until nothing fits.
void greedyFill(std::vector<int>& tour, double& len, const Problem& pb, std::vector<int>& pool,
                Criterion crit, int rcl, bool randomize, std::mt19937_64& rng) {
    std::vector<double>      deltas;
    std::vector<std::size_t> poss;

    while (!pool.empty()) {
        allInsertions(tour, pool, pb.D, pb.endIdx, deltas, poss);

        std::vector<char> ok(pool.size(), 0);
        bool any = false;
        for (std::size_t k = 0; k < pool.size(); ++k) {
            ok[k] = (len + deltas[k]) <= pb.budget + 1e-9;
            any   = any || ok[k];
        }
        if (!any) break;

        std::vector<double> sc(pool.size(), -kInf);
        for (std::size_t k = 0; k < pool.size(); ++k) {
            if (!ok[k]) continue;
            const double rw = pb.r(pool[k]);
            switch (crit) {
                case Criterion::Reward2: sc[k] = (rw * rw) / std::max(deltas[k], 1e-9); break;
                case Criterion::Reward:  sc[k] = rw; break;
                default:                 sc[k] = rw / std::max(deltas[k], 1e-9); break;
            }
        }

        std::size_t pick = 0;
        if (randomize && rcl > 1) {
            std::vector<std::size_t> order(pool.size());
            for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
            std::stable_sort(order.begin(), order.end(),
                             [&](std::size_t a, std::size_t b) { return sc[a] > sc[b]; });
            const auto nAvail = static_cast<std::size_t>(std::count(ok.begin(), ok.end(), 1));
            const std::size_t take =
                std::max<std::size_t>(1, std::min<std::size_t>(static_cast<std::size_t>(rcl), nAvail));
            std::uniform_real_distribution<double> u(0.0, 1.0);
            pick = order[static_cast<std::size_t>(u(rng) * static_cast<double>(take))];
        } else {
            double best = -kInf;
            for (std::size_t k = 0; k < pool.size(); ++k) {
                if (sc[k] > best) {
                    best = sc[k];
                    pick = k;
                }
            }
        }

        const int j = pool[pick];
        tour.insert(tour.begin() + static_cast<long>(poss[pick]) + 1, j);
        len += deltas[pick];
        pool.erase(pool.begin() + static_cast<long>(pick));
    }
}

/// 2-opt + Or-opt on a fixed node set.  Prize invariant, length down.  Position
/// 0 (the start) is never moved, and on a closed path neither is the final node.
/// Shortening is what converts "we chose these clusters" into "we can now afford
/// one more cluster".
void shortenPath(std::vector<int>& tour, const MatX& D, int endIdx, int maxIter, double& len) {
    if (tour.size() < 3) {
        len = pathLength(tour, D);
        return;
    }
    const auto lastFree = [&]() -> std::size_t {
        return (endIdx < 0) ? tour.size() - 1 : tour.size() - 2;
    };

    bool improved = true;
    int  it       = 0;
    while (improved && it < std::max(1, maxIter)) {
        improved = false;
        ++it;

        // --- 2-opt: reverse tour[i+1 .. j] ---
        {
            const std::size_t n  = tour.size();
            const std::size_t lf = lastFree();
            for (std::size_t i = 0; i + 2 <= lf; ++i) {
                double      bestGain = 1e-9;
                std::size_t bestJ    = 0;
                const int   a = tour[i], b = tour[i + 1];
                for (std::size_t j = i + 2; j <= lf; ++j) {
                    const int c = tour[j];
                    double cur = D(a, b);
                    double nxt = D(a, c);
                    if (j + 1 < n) {
                        const int dnx = tour[j + 1];
                        cur += D(c, dnx);
                        nxt += D(b, dnx);
                    }
                    const double gain = cur - nxt;
                    if (gain > bestGain) {
                        bestGain = gain;
                        bestJ    = j;
                    }
                }
                if (bestJ > 0) {
                    std::reverse(tour.begin() + static_cast<long>(i + 1),
                                 tour.begin() + static_cast<long>(bestJ + 1));
                    improved = true;
                }
            }
        }

        // --- Or-opt: relocate a segment of length 1..3, forward or reversed ---
        // Only the EXTERNAL edges are compared: with a symmetric metric the
        // relocated segment keeps its internal length either way, so that term
        // cancels out.
        for (std::size_t segLen = 1; segLen <= 3; ++segLen) {
            std::size_t s = 1;
            while (s + segLen - 1 <= lastFree()) {
                const std::size_t n = tour.size();
                const std::size_t e = s + segLen - 1;

                const int a0 = tour[s - 1];
                double gain = D(a0, tour[s]);
                if (e + 1 < n) gain += D(tour[e], tour[e + 1]) - D(a0, tour[e + 1]);
                if (gain <= 1e-9) {
                    ++s;
                    continue;
                }

                std::vector<int> seg(tour.begin() + static_cast<long>(s),
                                     tour.begin() + static_cast<long>(e) + 1);
                std::vector<int> rest;
                rest.reserve(n - seg.size());
                rest.insert(rest.end(), tour.begin(), tour.begin() + static_cast<long>(s));
                rest.insert(rest.end(), tour.begin() + static_cast<long>(e) + 1, tour.end());

                const std::size_t nr = rest.size();
                const std::size_t lastPosRest = (endIdx < 0) ? (nr - 1) : (nr >= 2 ? nr - 2 : 0);
                if (nr < 2) {
                    ++s;
                    continue;
                }

                double      bestAdd = kInf;
                std::size_t bestP   = 0;
                bool        bestRev = false;
                for (int rev = 0; rev < 2; ++rev) {
                    std::vector<int> sg = seg;
                    if (rev) std::reverse(sg.begin(), sg.end());
                    for (std::size_t p = 0; p <= lastPosRest; ++p) {
                        const int aR = rest[p];
                        double add;
                        if (p + 1 < nr) {
                            const int bR = rest[p + 1];
                            add = D(aR, sg.front()) + D(sg.back(), bR) - D(aR, bR);
                        } else {
                            add = D(aR, sg.front());
                        }
                        if (add < bestAdd) {
                            bestAdd = add;
                            bestP   = p;
                            bestRev = (rev != 0);
                        }
                    }
                }

                if (bestAdd < gain - 1e-9) {
                    std::vector<int> sg = seg;
                    if (bestRev) std::reverse(sg.begin(), sg.end());
                    std::vector<int> next;
                    next.reserve(n);
                    next.insert(next.end(), rest.begin(), rest.begin() + static_cast<long>(bestP) + 1);
                    next.insert(next.end(), sg.begin(), sg.end());
                    next.insert(next.end(), rest.begin() + static_cast<long>(bestP) + 1, rest.end());
                    tour     = std::move(next);
                    improved = true;
                } else {
                    ++s;
                }
            }
        }
    }
    len = pathLength(tour, D);
}

/// Resume greedy insertion into an existing tour (after shortening freed length).
void insertMore(std::vector<int>& tour, double& len, const Problem& pb, Criterion crit,
                std::mt19937_64& rng) {
    std::vector<char> inTour(static_cast<std::size_t>(pb.D.rows()), 0);
    for (const int t : tour) inTour[static_cast<std::size_t>(t)] = 1;
    std::vector<int> pool;
    for (const int c : pb.cand)
        if (!inTour[static_cast<std::size_t>(c)]) pool.push_back(c);
    greedyFill(tour, len, pb, pool, crit, 1, false, rng);
}

/// Swap a chosen node out for a richer unchosen one, if it fits.  This is the
/// move that undoes greedy's early commitment to cheap nodes.
void replaceMoves(std::vector<int>& tour, double& len, const Problem& pb) {
    std::vector<char> inTour(static_cast<std::size_t>(pb.D.rows()), 0);
    for (const int t : tour) inTour[static_cast<std::size_t>(t)] = 1;
    std::vector<int> pool;
    for (const int c : pb.cand)
        if (!inTour[static_cast<std::size_t>(c)]) pool.push_back(c);
    if (pool.empty()) return;

    bool improved = true;
    while (improved) {
        improved = false;
        const std::size_t n = tour.size();
        const std::size_t lastSel = (pb.endIdx < 0) ? (n - 1) : (n >= 2 ? n - 2 : 0);

        double      bestGain  = 1e-12;
        std::size_t bestQ     = 0;
        std::size_t bestPool  = 0;
        double      bestDelta = 0.0;
        bool        found     = false;

        for (std::size_t q = 1; q <= lastSel; ++q) {
            const int out = tour[q];
            if (pb.mandatory[static_cast<std::size_t>(out - 1)]) continue;
            const int a = tour[q - 1];
            const bool hasNext = (q + 1 < n);
            const int  b       = hasNext ? tour[q + 1] : -1;
            const double oldCost = hasNext ? (pb.D(a, out) + pb.D(out, b)) : pb.D(a, out);

            for (std::size_t k = 0; k < pool.size(); ++k) {
                const int j = pool[k];
                const double newCost = hasNext ? (pb.D(a, j) + pb.D(j, b)) : pb.D(a, j);
                const double dlt     = newCost - oldCost;
                if (len + dlt > pb.budget + 1e-9) continue;
                const double gain = pb.r(j) - pb.r(out);
                if (gain > bestGain) {
                    bestGain  = gain;
                    bestQ     = q;
                    bestPool  = k;
                    bestDelta = dlt;
                    found     = true;
                }
            }
        }

        if (found) {
            const int outNode = tour[bestQ];
            tour[bestQ]       = pool[bestPool];
            len += bestDelta;
            pool[bestPool] = outNode;
            improved       = true;
        }
    }
}

/// Drive a solution to a local optimum under move families 2 and 3.  Shortening
/// and insertion alternate (each shortening frees budget the insertion can
/// spend), then replacement runs, then one more round of both - a replacement
/// can free length too, and that length is worth re-spending.
void localOptimize(std::vector<int>& tour, double& len, const Problem& pb, Criterion crit,
                   int maxIter, std::mt19937_64& rng) {
    for (int it = 0; it < std::max(1, maxIter); ++it) {
        shortenPath(tour, pb.D, pb.endIdx, maxIter, len);
        const std::size_t before = tour.size();
        insertMore(tour, len, pb, crit, rng);
        if (tour.size() == before) break;
    }
    replaceMoves(tour, len, pb);
    shortenPath(tour, pb.D, pb.endIdx, maxIter, len);
    insertMore(tour, len, pb, crit, rng);
}

/// Greedy cheapest-insertion construction, optionally randomised.  Mandatory
/// nodes go in first (by cheapest insertion, ignoring their prize), then the
/// prize-driven loop runs over what is left.
void constructRoute(std::vector<int>& tour, double& len, const Problem& pb, Criterion crit,
                    int rcl, bool randomize, std::mt19937_64& rng) {
    tour.clear();
    tour.push_back(0);
    len = 0.0;
    if (pb.closed) {
        tour.push_back(pb.endIdx);
        len = pb.D(0, pb.endIdx);
    }

    std::vector<char> inTour(static_cast<std::size_t>(pb.D.rows()), 0);
    for (const int t : tour) inTour[static_cast<std::size_t>(t)] = 1;

    // --- mandatory first: feasibility before value ---
    std::vector<int> must;
    for (const int c : pb.cand)
        if (pb.mandatory[static_cast<std::size_t>(c - 1)] && !inTour[static_cast<std::size_t>(c)])
            must.push_back(c);

    std::vector<double>      deltas;
    std::vector<std::size_t> poss;
    while (!must.empty()) {
        allInsertions(tour, must, pb.D, pb.endIdx, deltas, poss);
        std::size_t pick = 0;
        double      best = kInf;
        for (std::size_t k = 0; k < must.size(); ++k) {
            if (deltas[k] < best) {
                best = deltas[k];
                pick = k;
            }
        }
        if (!std::isfinite(best)) break;
        tour.insert(tour.begin() + static_cast<long>(poss[pick]) + 1, must[pick]);
        len += best;
        must.erase(must.begin() + static_cast<long>(pick));
    }
    for (const int t : tour) inTour[static_cast<std::size_t>(t)] = 1;

    std::vector<int> pool;
    for (const int c : pb.cand)
        if (!inTour[static_cast<std::size_t>(c)]) pool.push_back(c);
    greedyFill(tour, len, pb, pool, crit, rcl, randomize, rng);
}

/// Tear nRem removable nodes out of the tour at random.  Mandatory nodes, the
/// depot and the forced end are never removable.
void ruin(std::vector<int>& tour, double& len, const Problem& pb, int nRem,
          std::mt19937_64& rng) {
    const std::size_t n = tour.size();
    const std::size_t lastFree = (pb.endIdx < 0) ? n : (n >= 1 ? n - 1 : 0);
    std::vector<std::size_t> removable;
    for (std::size_t p = 1; p < lastFree; ++p) {
        if (!pb.mandatory[static_cast<std::size_t>(tour[p] - 1)]) removable.push_back(p);
    }
    if (removable.empty()) {
        len = pathLength(tour, pb.D);
        return;
    }
    nRem = std::min<int>(nRem, static_cast<int>(removable.size()));
    std::shuffle(removable.begin(), removable.end(), rng);
    std::vector<std::size_t> pick(removable.begin(), removable.begin() + nRem);
    std::sort(pick.begin(), pick.end(), std::greater<>());
    for (const std::size_t p : pick) tour.erase(tour.begin() + static_cast<long>(p));
    len = pathLength(tour, pb.D);
}

}  // namespace

OrienteeringSolution solveBudgetedOrienteering(const Path2& nodes, const VecX& rewards,
                                               const Vec2& startPos, double budget,
                                               const OrienteeringParams& opts) {
    OrienteeringSolution sol;
    const Index K = nodes.rows();

    sol.info.rewardTotal = rewards.sum();
    sol.info.budget      = budget;
    sol.info.selected.assign(static_cast<std::size_t>(K), 0);
    for (Index i = 0; i < K; ++i) sol.info.dropped.push_back(i);

    const bool closed = opts.endPos.has_value();

    auto emptyRoute = [&]() {
        sol.routeXY.resize(closed ? 2 : 1, 2);
        sol.routeXY.row(0) = startPos.transpose();
        if (closed) sol.routeXY.row(1) = opts.endPos->transpose();
    };

    if (K == 0 || !(budget > 0.0)) {
        emptyRoute();
        sol.info.feasible = !(K > 0 && std::any_of(opts.mandatory.begin(), opts.mandatory.end(),
                                                   [](Index) { return true; }));
        return sol;
    }

    // --- cost matrix -------------------------------------------------------
    Problem pb;
    pb.closed = closed;
    const Index N = K + 1 + (closed ? 1 : 0);
    Path2 pts(N, 2);
    pts.row(0) = startPos.transpose();
    pts.block(1, 0, K, 2) = nodes;
    if (closed) pts.row(N - 1) = opts.endPos->transpose();

    pb.D = MatX::Zero(N, N);
    for (Index i = 0; i < N; ++i)
        for (Index j = 0; j < N; ++j) pb.D(i, j) = (pts.row(i) - pts.row(j)).norm();

    pb.r = VecX::Zero(N);
    pb.r.segment(1, K) = rewards;
    pb.endIdx = closed ? static_cast<int>(N - 1) : -1;
    pb.budget = budget;

    pb.mandatory.assign(static_cast<std::size_t>(K), 0);
    for (const Index m : opts.mandatory)
        if (m >= 0 && m < K) pb.mandatory[static_cast<std::size_t>(m)] = 1;

    // --- prune individually unreachable nodes ------------------------------
    // A node that cannot be visited even on a path that visits nothing else can
    // never be visited: prune it up front.
    bool feasible = true;
    for (Index k = 0; k < K; ++k) {
        double minCost = pb.D(0, k + 1);
        if (closed) minCost += pb.D(k + 1, pb.endIdx);
        if (minCost <= budget + 1e-9) {
            pb.cand.push_back(static_cast<int>(k + 1));
        } else {
            sol.info.unreachable.push_back(k);
            if (pb.mandatory[static_cast<std::size_t>(k)]) feasible = false;
        }
    }
    sol.info.feasible = feasible;

    if (pb.cand.empty()) {
        emptyRoute();
        sol.info.length     = closed ? pb.D(0, pb.endIdx) : 0.0;
        sol.info.budgetUsed = sol.info.length / budget;
        return sol;
    }

    double rewardCeiling = 0.0;
    for (const int c : pb.cand) rewardCeiling += pb.r(c);

    // --- multi-starts ------------------------------------------------------
    // Deterministic starts first, so a 1- or 2-start run is fully reproducible.
    static const Criterion kCrit[] = {Criterion::Ratio, Criterion::Reward2, Criterion::Reward};
    const int nStarts = std::max(1, opts.nStarts);

    // Work guard: the local search is O(n^2) per sweep in the chosen-node count,
    // and it is re-run on every calibration iteration.  Above ~60 candidates the
    // ruin-and-recreate budget is thinned - but only when the caller left
    // nPerturb unset, so an explicit choice is never silently overridden.
    int nPerturb = opts.nPerturb.value_or(40);
    if (K > 60 && !opts.nPerturb.has_value()) nPerturb = std::min(nPerturb, 10);

    std::mt19937_64 rng(opts.seed);

    std::vector<int> bestTour;
    double bestLen    = 0.0;
    double bestReward = 0.0;
    int    bestStart  = 0;
    bestTour.push_back(0);
    if (closed) {
        bestTour.push_back(pb.endIdx);
        bestLen = pb.D(0, pb.endIdx);
    }

    for (int s = 1; s <= nStarts; ++s) {
        Criterion crit   = kCrit[std::min<int>(s, 3) - 1];
        bool      random = (s > 2);
        int       rcl    = 1;
        if (random) {
            std::uniform_int_distribution<int> jitter(-1, 1);
            std::uniform_int_distribution<int> whichCrit(0, 2);
            rcl  = std::max(2, opts.rclSize + jitter(rng));
            crit = kCrit[whichCrit(rng)];
        }

        std::vector<int> tour;
        double           len = 0.0;
        constructRoute(tour, len, pb, crit, rcl, random, rng);
        localOptimize(tour, len, pb, crit, opts.maxLocalIter, rng);

        double rew = 0.0;
        for (const int t : tour) rew += pb.r(t);
        sol.info.startRewards.push_back(rew);

        if (rew > bestReward + 1e-12 ||
            (std::abs(rew - bestReward) <= 1e-12 && len < bestLen - 1e-9)) {
            bestReward = rew;
            bestTour   = tour;
            bestLen    = len;
            bestStart  = s;
        }
        if (bestReward >= rewardCeiling - 1e-9) break;  // every reachable node is in
    }

    // --- ruin and recreate on the incumbent --------------------------------
    // A rebuild may come back with MORE nodes than were torn out, which is the
    // only way this search discovers that four cheap clusters beat three rich
    // ones.  The perturbation budget is shared across the whole solve, and
    // thinned by how many nodes the incumbent holds.
    const auto nSel = std::max<std::size_t>(1, bestTour.size() - 1);
    int nP = std::max(0, nPerturb);
    if (bestReward >= rewardCeiling - 1e-9) {
        nP = 0;
    } else {
        nP = std::min<int>(nP, std::max<int>(8, static_cast<int>(std::lround(800.0 / static_cast<double>(nSel)))));
    }

    std::uniform_real_distribution<double> u01(0.0, 1.0);
    std::uniform_int_distribution<int>     whichCrit(0, 2);
    for (int pert = 0; pert < nP; ++pert) {
        const int nRem = 1 + static_cast<int>(u01(rng) * 3.0);
        std::vector<int> tTour = bestTour;
        double           tLen  = bestLen;
        ruin(tTour, tLen, pb, nRem, rng);
        if (tTour.size() == bestTour.size()) break;  // nothing removable, ever

        // A perturbation disturbs only a few nodes, so the re-optimisation is
        // capped at a few sweeps: the full schedule after every tear costs
        // several times more for almost nothing.
        localOptimize(tTour, tLen, pb, kCrit[whichCrit(rng)], std::min(opts.maxLocalIter, 4), rng);

        double tRew = 0.0;
        for (const int t : tTour) tRew += pb.r(t);
        if (tRew > bestReward + 1e-12 ||
            (std::abs(tRew - bestReward) <= 1e-12 && tLen < bestLen - 1e-9)) {
            bestTour   = tTour;
            bestLen    = tLen;
            bestReward = tRew;
            if (bestReward >= rewardCeiling - 1e-9) break;
        }
    }

    // --- pack the output ---------------------------------------------------
    for (const int t : bestTour) {
        if (t == 0) continue;
        if (closed && t == pb.endIdx) continue;
        sol.visitOrder.push_back(static_cast<Index>(t - 1));
    }

    const auto n = static_cast<Index>(sol.visitOrder.size());
    sol.routeXY.resize(n + 1 + (closed ? 1 : 0), 2);
    sol.routeXY.row(0) = startPos.transpose();
    for (Index i = 0; i < n; ++i) sol.routeXY.row(i + 1) = nodes.row(sol.visitOrder[static_cast<std::size_t>(i)]);
    if (closed) sol.routeXY.row(n + 1) = opts.endPos->transpose();

    sol.info.reward         = bestReward;
    sol.info.rewardFraction = bestReward / std::max(sol.info.rewardTotal, 1e-300);
    sol.info.length         = bestLen;
    sol.info.budgetUsed     = bestLen / budget;
    sol.info.selected.assign(static_cast<std::size_t>(K), 0);
    for (const Index v : sol.visitOrder) sol.info.selected[static_cast<std::size_t>(v)] = 1;
    sol.info.dropped.clear();
    for (Index k = 0; k < K; ++k)
        if (!sol.info.selected[static_cast<std::size_t>(k)]) sol.info.dropped.push_back(k);
    bool mandatoryIn = true;
    for (Index k = 0; k < K; ++k)
        if (pb.mandatory[static_cast<std::size_t>(k)] && !sol.info.selected[static_cast<std::size_t>(k)])
            mandatoryIn = false;
    sol.info.feasible  = sol.info.feasible && mandatoryIn && bestLen <= budget + 1e-6;
    sol.info.nStarts   = static_cast<int>(sol.info.startRewards.size());
    sol.info.bestStart = bestStart;

    if (opts.verbose) {
        std::printf(
            "  orienteering: %lld/%lld nodes, reward %.4g/%.4g (%.1f%%), %.0f m of %.0f m "
            "budget (%.1f%%), %d starts.\n",
            static_cast<long long>(sol.visitOrder.size()), static_cast<long long>(K),
            sol.info.reward, sol.info.rewardTotal, 100.0 * sol.info.rewardFraction,
            sol.info.length, budget, 100.0 * sol.info.budgetUsed, sol.info.nStarts);
    }
    return sol;
}

}  // namespace mtl::planning
