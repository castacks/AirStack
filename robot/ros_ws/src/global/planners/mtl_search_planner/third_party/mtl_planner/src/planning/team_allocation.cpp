#include "mtl/planning/team_allocation.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>

#include "mtl/planning/info_score.hpp"

namespace mtl::planning {
namespace {

std::vector<Index> uniqueSorted(std::vector<Index> v) {
    std::sort(v.begin(), v.end());
    v.erase(std::unique(v.begin(), v.end()), v.end());
    return v;
}

std::vector<Index> setDifference(std::vector<Index> a, std::vector<Index> b) {
    a = uniqueSorted(std::move(a));
    b = uniqueSorted(std::move(b));
    std::vector<Index> out;
    std::set_difference(a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(out));
    return out;
}

std::vector<Index> setIntersection(std::vector<Index> a, std::vector<Index> b) {
    a = uniqueSorted(std::move(a));
    b = uniqueSorted(std::move(b));
    std::vector<Index> out;
    std::set_intersection(a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(out));
    return out;
}

std::vector<Index> setUnion(std::vector<Index> a, std::vector<Index> b) {
    a = uniqueSorted(std::move(a));
    b = uniqueSorted(std::move(b));
    std::vector<Index> out;
    std::set_union(a.begin(), a.end(), b.begin(), b.end(), std::back_inserter(out));
    return out;
}

}  // namespace

TeamResult allocateBudgetedTeam(const SortieContext& ctx, const std::vector<Vec2>& agentStarts,
                                const std::vector<int>& agentOfCluster,
                                const std::vector<double>& budgets) {
    const PlannerParams& P  = *ctx.params;
    const ClusterSet&    cl = *ctx.clusters;
    const CellSet&       cs = *ctx.cells;
    const BudgetParams&  bo = P.budget;

    const auto numAgents = static_cast<int>(agentStarts.size());
    const Index K        = cl.size();

    const bool budgeted =
        std::any_of(budgets.begin(), budgets.end(), [](double b) { return std::isfinite(b); });

    TeamResult out;
    out.plans.resize(static_cast<std::size_t>(numAgents));

    // --- pass 1: the k-means partition -------------------------------------
    for (int a = 0; a < numAgents; ++a) {
        AgentSpec spec;
        spec.startPos   = agentStarts[static_cast<std::size_t>(a)];
        spec.id         = a + 1;
        spec.budgetDist = budgets[static_cast<std::size_t>(a)];
        for (Index k = 0; k < K; ++k)
            if (agentOfCluster[static_cast<std::size_t>(k)] == a) spec.candClusters.push_back(k);
        out.plans[static_cast<std::size_t>(a)] = planAgentSortie(spec, ctx);
    }

    std::vector<Index> reallocated;
    int                rounds = 0;

    // --- pass 2: re-offer the leftovers ------------------------------------
    if (budgeted && bo.reallocate && K > 0) {
        std::vector<char> taken(static_cast<std::size_t>(K), 0);
        for (const AgentPlan& p : out.plans)
            for (const Index c : p.selClusters) taken[static_cast<std::size_t>(c)] = 1;
        std::vector<Index> pool;
        for (Index k = 0; k < K; ++k)
            if (!taken[static_cast<std::size_t>(k)]) pool.push_back(k);

        for (int rd = 1; rd <= std::max(0, bo.reallocRounds); ++rd) {
            if (pool.empty()) break;
            rounds = rd;
            bool changed = false;

            std::vector<int> order(static_cast<std::size_t>(numAgents));
            std::iota(order.begin(), order.end(), 0);
            std::stable_sort(order.begin(), order.end(), [&](int a, int b) {
                const double sa = budgets[static_cast<std::size_t>(a)] -
                                  out.plans[static_cast<std::size_t>(a)].flownLength;
                const double sb = budgets[static_cast<std::size_t>(b)] -
                                  out.plans[static_cast<std::size_t>(b)].flownLength;
                return sa > sb;  // richest slack first
            });

            for (const int a : order) {
                if (pool.empty() || !std::isfinite(budgets[static_cast<std::size_t>(a)])) continue;

                const AgentPlan&         old     = out.plans[static_cast<std::size_t>(a)];
                const std::vector<Index> oldSel  = old.selClusters;
                const double             oldInfo = old.score.info;

                AgentSpec spec;
                spec.startPos     = agentStarts[static_cast<std::size_t>(a)];
                spec.id           = a + 1;
                spec.budgetDist   = budgets[static_cast<std::size_t>(a)];
                spec.candClusters = setUnion(oldSel, pool);

                AgentPlan trial = planAgentSortie(spec, ctx);

                if (trial.feasible && trial.score.info > oldInfo + 1e-9) {
                    const std::vector<Index> newSel   = trial.selClusters;
                    const std::vector<Index> gained   = setIntersection(newSel, pool);
                    const std::vector<Index> released = setDifference(oldSel, newSel);

                    out.plans[static_cast<std::size_t>(a)] = std::move(trial);
                    pool        = setUnion(setDifference(pool, gained), released);
                    reallocated = setUnion(reallocated, gained);
                    changed     = true;

                    if (bo.verbose && !gained.empty()) {
                        std::printf(
                            "  realloc round %d: agent %d took %lld pooled cluster(s), released "
                            "%lld, info %.4g -> %.4g.\n",
                            rd, a + 1, static_cast<long long>(gained.size()),
                            static_cast<long long>(released.size()), oldInfo,
                            out.plans[static_cast<std::size_t>(a)].score.info);
                    }
                }
            }
            if (!changed) break;
        }
    }

    // --- team totals -------------------------------------------------------
    std::vector<Index> servicedAll, reached;
    out.info.flownLength.assign(static_cast<std::size_t>(numAgents), 0.0);
    out.info.budgetUsed.assign(static_cast<std::size_t>(numAgents), 0.0);
    double totalFlown = 0.0, totalBudget = 0.0;
    for (int a = 0; a < numAgents; ++a) {
        const AgentPlan& p = out.plans[static_cast<std::size_t>(a)];
        servicedAll.insert(servicedAll.end(), p.servicedCellIdx.begin(), p.servicedCellIdx.end());
        reached.insert(reached.end(), p.selClusters.begin(), p.selClusters.end());
        out.info.flownLength[static_cast<std::size_t>(a)] = p.flownLength;
        out.info.budgetUsed[static_cast<std::size_t>(a)]  = p.budgetUsed;
        totalFlown  += p.flownLength;
        totalBudget += budgets[static_cast<std::size_t>(a)];
    }
    servicedAll = uniqueSorted(servicedAll);
    reached     = uniqueSorted(reached);

    std::vector<Index> allCells(static_cast<std::size_t>(cs.size()));
    std::iota(allCells.begin(), allCells.end(), Index{0});
    std::vector<Index> allClusters(static_cast<std::size_t>(K));
    std::iota(allClusters.begin(), allClusters.end(), Index{0});

    InfoScoreOptions so;
    so.budget       = totalBudget;
    so.totalMapMass = cs.totalMapMass;
    const InfoScore teamScore = pathInformationScore(cs.mass, servicedAll, totalFlown, so);

    out.info.info              = teamScore.info;
    out.info.infoTotal         = teamScore.infoTotal;
    out.info.infoFraction      = teamScore.infoFraction;
    out.info.servicedCellIdx   = servicedAll;
    out.info.unservicedCellIdx = setDifference(allCells, servicedAll);
    out.info.reachedClusters   = reached;
    out.info.unreachedClusters = setDifference(allClusters, reached);
    out.info.budgets           = budgets;
    out.info.reallocated       = reallocated;
    out.info.rounds            = rounds;
    out.info.score             = teamScore;
    return out;
}

}  // namespace mtl::planning
