// The budgeted orienteering solver (the curve planner uses it to ORDER each
// agent's clusters for the 'clusters' seed).  The solver sections of
// cpp_planner/tests/test_orienteering.cpp; the Dubins / anchor sections are
// not carried over because those stages are not part of this package.
#include <random>

#include "mtl_curve/core/numeric.hpp"
#include "mtl_curve/planning/orienteering.hpp"
#include "test_util.hpp"

using namespace mtl::curve;
using namespace mtl::curve::planning;

int main() {
    // --- an infinite budget takes everything -------------------------------
    {
        Path2 nodes(5, 2);
        nodes << 100, 0, 200, 0, 300, 0, 400, 0, 500, 0;
        VecX rewards = VecX::Ones(5);
        const OrienteeringSolution s =
            solveBudgetedOrienteering(nodes, rewards, Vec2(0, 0), kInf, {});
        CHECK(static_cast<Index>(s.visitOrder.size()) == 5);
        CHECK_NEAR(s.info.reward, 5.0, 1e-9);
        CHECK_NEAR(s.info.length, 500.0, 1e-6);  // the optimal order is the obvious one
    }

    // --- the budget is respected, and never exceeded -----------------------
    {
        std::mt19937_64 rng(5);
        std::uniform_real_distribution<double> u(0.0, 3000.0);
        Path2 nodes(25, 2);
        VecX  rewards(25);
        for (Index i = 0; i < 25; ++i) {
            nodes(i, 0) = u(rng);
            nodes(i, 1) = u(rng);
            rewards(i)  = 1.0 + static_cast<double>(i % 5);
        }
        for (const double budget : {500.0, 2000.0, 6000.0}) {
            const OrienteeringSolution s =
                solveBudgetedOrienteering(nodes, rewards, Vec2(1500, 1500), budget, {});
            CHECK(s.info.length <= budget + 1e-6);
            // The reported length must match the route it handed back.
            CHECK_NEAR(s.info.length, core::polylineLength(s.routeXY), 1e-6);
            // No node appears twice.
            std::vector<Index> v = s.visitOrder;
            std::sort(v.begin(), v.end());
            CHECK(std::adjacent_find(v.begin(), v.end()) == v.end());
        }
    }

    // --- a bigger budget never collects less -------------------------------
    // This is the property that says the search is not simply lucky: the solver
    // must be monotone in the budget, up to heuristic noise.
    {
        std::mt19937_64 rng(9);
        std::uniform_real_distribution<double> u(0.0, 4000.0);
        Path2 nodes(20, 2);
        VecX  rewards(20);
        for (Index i = 0; i < 20; ++i) {
            nodes(i, 0) = u(rng);
            nodes(i, 1) = u(rng);
            rewards(i)  = 1.0;
        }
        double prev = -1.0;
        for (const double budget : {1000.0, 2000.0, 4000.0, 8000.0, 16000.0}) {
            const OrienteeringSolution s =
                solveBudgetedOrienteering(nodes, rewards, Vec2(2000, 2000), budget, {});
            CHECK(s.info.reward >= prev - 1e-9);
            prev = s.info.reward;
        }
        CHECK_NEAR(prev, 20.0, 1e-9);  // 16 km buys the lot
    }

    // --- unreachable nodes are pruned, not silently attempted --------------
    {
        Path2 nodes(2, 2);
        nodes << 100, 0, 100000, 0;
        VecX rewards(2);
        rewards << 1.0, 1000.0;  // the far node is worth far more, and cannot be had
        const OrienteeringSolution s =
            solveBudgetedOrienteering(nodes, rewards, Vec2(0, 0), 500.0, {});
        CHECK(s.info.unreachable.size() == 1);
        CHECK(s.info.unreachable[0] == 1);
        CHECK(s.visitOrder.size() == 1 && s.visitOrder[0] == 0);
    }

    return test::report("test_orienteering");
}
