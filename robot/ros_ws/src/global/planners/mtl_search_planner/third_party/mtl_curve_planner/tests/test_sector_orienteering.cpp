// The 'sector' seed's coverage orienteering (planning::solveSectorOrienteering):
// the route fits its budget (open and closed), its reward is the weight in the
// union of the visited boxes (overlapping boxes are not paid for twice), it
// is deterministic, and an empty
// map gives an empty route.  Then the default planner flies it.
#include <cmath>

#include "mtl_curve/mapgen/scenario.hpp"
#include "mtl_curve/planner.hpp"
#include "mtl_curve/planning/sector_orienteering.hpp"
#include "test_util.hpp"

using namespace mtl::curve;

namespace {

// Gaussian blobs on a 50 m raster over a 5 km square
MatX blobs(const std::vector<Vec2>& c, double sigma, double hg, Index n) {
    MatX W = MatX::Zero(n, n);
    for (Index r = 0; r < n; ++r)
        for (Index q = 0; q < n; ++q)
            for (const Vec2& m : c) {
                const Vec2 x((q + 0.5) * hg, (r + 0.5) * hg);
                W(r, q) += std::exp(-0.5 * (x - m).squaredNorm() / (sigma * sigma));
            }
    return W / W.sum();
}

double routeLength(const Vec2& s, const Path2& wp, const std::optional<Vec2>& goal) {
    double L = 0.0;
    Vec2 p = s;
    for (Index k = 0; k < wp.rows(); ++k) {
        const Vec2 q = wp.row(k).transpose();
        L += (q - p).norm();
        p = q;
    }
    if (goal) L += (*goal - p).norm();
    return L;
}

}  // namespace

int main() {
    const double hg = 50.0, hw = 515.0;
    const Index n = 100;
    const Eigen::Vector4d bounds(0.0, 5000.0, 0.0, 5000.0);
    const MatX W = blobs({Vec2(1000, 1000), Vec2(4000, 1200), Vec2(2500, 4000), Vec2(3800, 3800)}, 250.0, hg, n);
    const Vec2 s0(2500.0, 200.0);
    SectorParams sp;

    // --- open route: fits the budget, reward = union of its boxes ------------
    const double L = 8000.0;
    const planning::SectorRoute r = planning::solveSectorOrienteering(W, 0.5 * hg, 0.5 * hg, hg, bounds, hw, s0,
                                                                      std::nullopt, L, sp);
    std::printf("  open: %lld sectors, %.0f m of %.0f, reward %.4f, %lld candidates, %.3f s\n",
                static_cast<long long>(r.wp.rows()), r.length, r.budget, r.reward,
                static_cast<long long>(r.nCandidates), r.seconds);
    CHECK(r.wp.rows() > 0);
    CHECK(r.length <= sp.budgetFrac * L + 1e-6);
    CHECK_NEAR(routeLength(s0, r.wp, std::nullopt), r.length, 1e-6);
    {
        const double R = sp.boxFrac * hw;
        double cov = 0.0;
        for (Index rr = 0; rr < n; ++rr)
            for (Index q = 0; q < n; ++q) {
                const Vec2 x((q + 0.5) * hg, (rr + 0.5) * hg);
                bool in = false;
                for (Index k = 0; k < r.wp.rows() && !in; ++k)
                    in = std::abs(x.x() - r.wp(k, 0)) <= R && std::abs(x.y() - r.wp(k, 1)) <= R;
                if (in) cov += W(rr, q);
            }
        CHECK_NEAR(cov, r.reward, 1e-9);
    }
    CHECK(r.reward > 0.9);   // four blobs, 8 km: all of them

    // --- determinism ----------------------------------------------------------
    {
        const planning::SectorRoute r2 = planning::solveSectorOrienteering(W, 0.5 * hg, 0.5 * hg, hg, bounds, hw, s0,
                                                                           std::nullopt, L, sp);
        CHECK(r2.wp.rows() == r.wp.rows() && (r2.wp - r.wp).norm() == 0.0);
    }

    // --- return home: the leg back counts --------------------------------------
    {
        const planning::SectorRoute rc = planning::solveSectorOrienteering(W, 0.5 * hg, 0.5 * hg, hg, bounds, hw, s0,
                                                                           s0, L, sp);
        CHECK(rc.length <= sp.budgetFrac * L + 1e-6);
        CHECK_NEAR(routeLength(s0, rc.wp, s0), rc.length, 1e-6);
        CHECK(rc.reward <= r.reward + 1e-12);
    }

    // --- marginal gains: overlapping boxes are paid for once -------------------
    {
        const double R = sp.boxFrac * hw;
        double additive = 0.0;
        for (Index k = 0; k < r.wp.rows(); ++k)
            for (Index rr = 0; rr < n; ++rr)
                for (Index q = 0; q < n; ++q)
                    if (std::abs((q + 0.5) * hg - r.wp(k, 0)) <= R && std::abs((rr + 0.5) * hg - r.wp(k, 1)) <= R)
                        additive += W(rr, q);
        std::printf("  additive box value of the route %.4f vs its union reward %.4f\n", additive, r.reward);
        CHECK(additive > r.reward + 1e-3);   // 20 sectors, 4 blobs: the boxes overlap
    }

    // --- nothing to find: empty route --------------------------------------------
    {
        const planning::SectorRoute re = planning::solveSectorOrienteering(MatX::Zero(n, n), 0.5 * hg, 0.5 * hg, hg,
                                                                           bounds, hw, s0, std::nullopt, L, sp);
        CHECK(re.wp.rows() == 0);
        CHECK(re.reward == 0.0);
    }

    // --- the default planner: one agent, the sector seed, one optimisation ------
    {
        PlannerParams p;
        p.cellSize = 10.0;
        p.verbose = false;
        p.numAgents = 1;
        p.maxFlightTime = 150.0;
        p.optimizer.maxIter = 40;
        p.optimizer.exploreIter = 20;
        const BeliefField belief = mapgen::generateBeliefMap(p.mapSize, p.cellSize, BeliefMapParams{}, p.rngSeed);
        Planner planner(p);
        const PlanningResult res = planner.plan(belief, {Vec2(2000.0, 2000.0)});
        CHECK(res.plans.size() == 1);
        CHECK(res.plans[0].initStrategy == "sector");
        CHECK(res.plans[0].feasible);
        CHECK(res.team.sectorWaypoints.size() == 1 && res.team.sectorWaypoints[0].rows() > 0);
        CHECK(res.team.log.empty());                  // no reallocation by default
        CHECK(res.team.Jhist.size() == 1);            // one sweep: one optimisation
        std::printf("  planner: sector seed, %lld waypoints, fast residual %.4f\n",
                    static_cast<long long>(res.team.sectorWaypoints[0].rows()), res.team.Jfinal);
    }
    return test::report("test_sector_orienteering");
}
