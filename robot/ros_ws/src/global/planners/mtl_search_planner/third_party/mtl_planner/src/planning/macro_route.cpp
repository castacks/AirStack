#include "mtl/planning/macro_route.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "mtl/core/dubins.hpp"
#include "mtl/core/numeric.hpp"
#include "mtl/planning/orienteering.hpp"
#include "mtl/routing/tsp.hpp"

namespace mtl::planning {

MacroRouteSolution planBudgetedMacroRoute(const Path2& centroids, const VecX& rewards,
                                          const Vec2& startPos, double budgetDist,
                                          double minTurnRadius, const MacroRouteParams& opts,
                                          const OrienteeringParams& orienteeringOpts,
                                          const DubinsParams& dubinsOpts,
                                          const LengthFcn& lengthFcn) {
    MacroRouteSolution sol;
    const Index K = centroids.rows();

    LengthFcn measure = lengthFcn;
    if (!measure) {
        measure = [minTurnRadius, dubinsOpts](const Path2& p) {
            return core::dubinsLength(p, minTurnRadius, dubinsOpts);
        };
    }

    sol.info.rewardTotal  = rewards.sum();
    sol.info.budget       = budgetDist;
    sol.info.budgetActive = std::isfinite(budgetDist);
    sol.info.selected.assign(static_cast<std::size_t>(K), 0);
    for (Index k = 0; k < K; ++k) sol.info.dropped.push_back(k);

    // --- no clusters to service -------------------------------------------
    if (K == 0) {
        sol.routeXY.resize(1, 2);
        sol.routeXY.row(0) = startPos.transpose();
        sol.info.note      = "no clusters assigned";
        return sol;
    }

    // --- UNBUDGETED: the plain TSP over everything -------------------------
    // Deliberately NOT routed through the orienteering solver: with an infinite
    // budget every node is selected and the two answers would differ only in
    // tour order, but "differ" is the problem - existing results must reproduce.
    if (!std::isfinite(budgetDist)) {
        sol.visitOrder = routing::tspVisitOrder(centroids, startPos);
        sol.routeXY.resize(static_cast<Index>(sol.visitOrder.size()) + 1, 2);
        sol.routeXY.row(0) = startPos.transpose();
        for (std::size_t i = 0; i < sol.visitOrder.size(); ++i)
            sol.routeXY.row(static_cast<Index>(i) + 1) = centroids.row(sol.visitOrder[i]);

        sol.info.selected.assign(static_cast<std::size_t>(K), 1);
        sol.info.dropped.clear();
        sol.info.reward         = rewards.sum();
        sol.info.rewardFraction = 1.0;
        sol.info.euclidLength   = core::polylineLength(sol.routeXY);
        sol.info.flownLength    = measure(sol.routeXY);
        sol.info.orienteering.note = "bypassed (budget = Inf)";
        sol.info.note = "full TSP (unbudgeted)";
        if (opts.verbose) {
            std::printf("  route: budget Inf - full TSP over %lld clusters, %.0f m flown.\n",
                        static_cast<long long>(K), sol.info.flownLength);
        }
        return sol;
    }

    // --- BUDGETED: calibrate the Euclidean budget against the flown arc ----
    Path2              bestRoute(1, 2);
    bestRoute.row(0) = startPos.transpose();
    std::vector<Index> bestOrder;
    double             bestReward = -kInf;
    double             bestEucl   = 0.0;
    double             bestFlown  = 0.0;
    double             bestScale  = 0.0;
    OrienteeringInfo   bestOInfo;

    double lo    = 0.0;   // largest scale MEASURED feasible (0 = empty route, always so)
    double hi    = kInf;  // smallest scale measured infeasible
    double scale = 1.0;
    int    it    = 0;

    // Clusters that cannot be reached even by flying straight at them on the
    // full budget.
    Index nAttainable = 0;
    for (Index k = 0; k < K; ++k) {
        const double d = (centroids.row(k).transpose() - startPos).norm();
        if (d > budgetDist + 1e-9) {
            sol.info.unreachable.push_back(k);
        } else {
            ++nAttainable;
        }
    }

    while (it < std::max(1, opts.maxCalib)) {
        ++it;
        const double eff = budgetDist * scale;

        const OrienteeringSolution os =
            solveBudgetedOrienteering(centroids, rewards, startPos, eff, orienteeringOpts);
        const double L = measure(os.routeXY);

        if (L <= budgetDist + 1e-6) {
            // --- measured feasible: keep it if it is the richest so far ---
            const bool better = os.info.reward > bestReward + 1e-12 ||
                                (std::abs(os.info.reward - bestReward) <= 1e-12 &&
                                 L < bestFlown - 1e-9);
            if (better) {
                bestRoute  = os.routeXY;
                bestOrder  = os.visitOrder;
                bestReward = os.info.reward;
                bestEucl   = os.info.length;
                bestFlown  = L;
                bestOInfo  = os.info;
                bestScale  = scale;
            }
            lo = std::max(lo, scale);

            if (static_cast<Index>(os.visitOrder.size()) >= nAttainable) break;

            if (!std::isfinite(hi)) {
                double grow = 1.25;
                if (L > 1e-6) grow = std::max(1.1, std::min(2.0, budgetDist / L));
                scale *= grow;
            } else {
                scale = 0.5 * (lo + hi);
            }
        } else {
            // --- measured infeasible: pull the scale back toward B/L ---
            hi = std::min(hi, scale);
            double newScale = scale * (budgetDist / std::max(L, 1e-300)) * 0.98;
            if (newScale <= lo + 1e-4 || newScale >= hi) newScale = 0.5 * (lo + hi);
            scale = newScale;
        }

        if (std::isfinite(hi) && (hi - lo) < opts.scaleTol) break;
    }

    if (bestReward < 0.0) {
        // Not even one cluster could be reached inside the flown budget.
        bestReward = 0.0;
        bestRoute.resize(1, 2);
        bestRoute.row(0) = startPos.transpose();
        bestOrder.clear();
        bestEucl  = 0.0;
        bestFlown = 0.0;
        bestOInfo = OrienteeringInfo{};
        bestOInfo.note = "no feasible cluster";
    }

    sol.routeXY    = bestRoute;
    sol.visitOrder = bestOrder;

    sol.info.reward         = bestReward;
    sol.info.rewardFraction = bestReward / std::max(sol.info.rewardTotal, 1e-300);
    sol.info.euclidLength   = bestEucl;
    sol.info.flownLength    = bestFlown;
    sol.info.budgetUsed     = bestFlown / budgetDist;
    sol.info.selected.assign(static_cast<std::size_t>(K), 0);
    for (const Index v : sol.visitOrder) sol.info.selected[static_cast<std::size_t>(v)] = 1;
    sol.info.dropped.clear();
    for (Index k = 0; k < K; ++k)
        if (!sol.info.selected[static_cast<std::size_t>(k)]) sol.info.dropped.push_back(k);
    sol.info.calibIters   = it;
    sol.info.scale        = bestScale;
    sol.info.feasible     = bestFlown <= budgetDist + 1e-6;
    sol.info.orienteering = bestOInfo;

    if (opts.verbose) {
        std::printf(
            "  route: %lld/%lld clusters, reward %.4g/%.4g (%.1f%%), flown %.0f m of %.0f m "
            "(%.1f%%), euclid %.0f m, %d calib iters (scale %.3f).\n",
            static_cast<long long>(sol.visitOrder.size()), static_cast<long long>(K),
            sol.info.reward, sol.info.rewardTotal, 100.0 * sol.info.rewardFraction,
            sol.info.flownLength, budgetDist, 100.0 * sol.info.budgetUsed,
            sol.info.euclidLength, it, bestScale);
    }
    return sol;
}

}  // namespace mtl::planning
