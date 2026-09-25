// The budgeted route: the solver, and the Dubins re-costing loop around it.
#include <random>

#include "mtl/core/dubins.hpp"
#include "mtl/core/numeric.hpp"
#include "mtl/planning/cell_anchors.hpp"
#include "mtl/planning/macro_route.hpp"
#include "mtl/planning/orienteering.hpp"
#include "test_util.hpp"

using namespace mtl;
using namespace mtl::planning;

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

    // --- the macro route enforces the budget on the FLOWN arc --------------
    // The Euclidean tour always fits; the point is that the Dubins arc does too.
    {
        std::mt19937_64 rng(13);
        std::uniform_real_distribution<double> u(0.0, 3000.0);
        Path2 cent(15, 2);
        VecX  rew(15);
        for (Index i = 0; i < 15; ++i) {
            cent(i, 0) = u(rng);
            cent(i, 1) = u(rng);
            rew(i)     = 1.0;
        }
        DubinsParams dop;
        const double budget = 5000.0;
        const MacroRouteSolution m = planBudgetedMacroRoute(cent, rew, Vec2(1500, 1500), budget,
                                                            100.0, {}, {}, dop);
        CHECK(m.info.feasible);
        CHECK(m.info.flownLength <= budget + 1e-6);
        CHECK_NEAR(m.info.flownLength, core::dubinsLength(m.routeXY, 100.0, dop), 1e-6);
        // Dubins arcs are never shorter than the straight-line tour.
        CHECK(m.info.flownLength >= m.info.euclidLength - 1e-6);
    }

    // --- cell anchors: free harvest costs nothing, anchors stay in budget ---
    {
        Path2 route(3, 2);
        route << 0, 0, 1000, 0, 2000, 0;
        DubinsParams dop;
        const double L0 = core::dubinsLength(route, 100.0, dop);

        // Two cells within reach of a waypoint already flown, one far off.
        Path2 cells(3, 2);
        cells << 1000, 300, 1050, 350, 1000, 2500;
        VecX mass(3);
        mass << 1.0, 1.0, 5.0;

        CellAnchorParams cap;
        cap.gimbalReach = 500.0;
        const CellAnchorResult r =
            refineWithCellAnchors(route, cells, mass, L0 + 100.0, 100.0, cap, dop);

        CHECK(r.info.nHarvested >= 2);           // the two near cells ride along free
        CHECK(r.info.flownLength <= L0 + 100.0 + 1e-6);
        CHECK(r.info.feasible);
        // The row bookkeeping must cover every returned row.
        CHECK(static_cast<Index>(r.rowTag.size()) == r.routeXY.rows());
        CHECK(r.rowCells.size() == r.rowTag.size());
    }

    return test::report("test_orienteering");
}
