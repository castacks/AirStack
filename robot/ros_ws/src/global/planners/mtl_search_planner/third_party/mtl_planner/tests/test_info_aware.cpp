// The information-aware abstraction: peak basins, mass levels, the coverage
// model, and the search that plans over them (PlannerParams::infoAware).
#include <algorithm>
#include <array>
#include <stdexcept>
#include <cmath>
#include <map>
#include <set>

#include "mtl/core/numeric.hpp"
#include "mtl/eval/detection.hpp"
#include "mtl/mapgen/scenario.hpp"
#include "mtl/mapping/cells.hpp"
#include "mtl/mapping/peak_clusters.hpp"
#include "mtl/planner.hpp"
#include "mtl/planning/coverage_score.hpp"
#include "mtl/planning/info_aware.hpp"
#include "test_util.hpp"

using namespace mtl;

namespace {

/// Cells on a 200 m lattice holding a sum of Gaussian bumps.
CellSet latticeCells(const std::vector<std::array<double, 4>>& bumps, double size = 5000.0) {
    const double cs = 200.0;
    std::vector<Vec2> c;
    std::vector<double> m;
    for (double x = cs / 2; x < size; x += cs)
        for (double y = cs / 2; y < size; y += cs) {
            double v = 0.0;
            for (const auto& b : bumps)  // x, y, sigma, amplitude
                v += b[3] * std::exp(-0.5 * (std::pow((x - b[0]) / b[2], 2) + std::pow((y - b[1]) / b[2], 2)));
            if (v > 1e-3) {
                c.emplace_back(x, y);
                m.push_back(v);
            }
        }
    CellSet out;
    const auto n = static_cast<Index>(c.size());
    out.centers.resize(n, 2);
    out.mass.resize(n);
    double tot = 0.0;
    for (const double v : m) tot += v;
    for (Index i = 0; i < n; ++i) {
        out.centers.row(i) = c[static_cast<std::size_t>(i)].transpose();
        out.mass(i)        = m[static_cast<std::size_t>(i)] / tot;
    }
    out.cellSize     = cs;
    out.area         = VecX::Constant(n, cs * cs);
    out.nPix         = VecX::Constant(n, 1.0);
    out.meanBelief   = out.mass / (cs * cs);
    out.peakBelief   = out.meanBelief;
    out.retainedMass = out.mass.sum();
    out.totalMapMass = 1.0;
    out.massNorm     = out.mass / out.retainedMass;
    out.gridRes      = Vec2(10, 10);
    return out;
}

PlannerParams smallParams() {
    PlannerParams p;
    p.cellSize         = 10.0;
    p.numAgents        = 1;
    p.verbose          = false;
    p.verifyGeometryVerbose = false;
    p.droneAltitude    = 300.0;
    p.avgDroneSpeed    = 6.0;
    p.minTurnRadius    = 12.0;
    p.sensorTiltAngle  = deg2rad(30.0);
    p.maxClusterRadius = 550.0;
    p.maxFlightTime    = 700.0;
    p.targetCellSize   = 200.0;
    p.dubins.stepSize  = 0.5;
    p.sensor.b         = 0.1;
    p.gimbal.targetTol = 5.0;
    p.extension.extendDist = 200.0;
    p.infoAware.threads = 2;
    p.infoAware.maxMoves = 6;
    return p;
}

}  // namespace

int main() {
    // --- basins: two separated peaks stay two, two overlapping ones merge ----
    {
        const CellSet two = latticeCells({{{1200, 1200, 250, 1.0}}, {{3800, 3800, 250, 0.7}}});
        const mapping::PeakBasins b2 = mapping::findPeakBasins(two, 0.6);
        CHECK(b2.size() == 2);
        // basin 0 is the higher peak, and its top is next to (1200, 1200)
        CHECK((two.centers.row(b2.peakCell[0]).transpose() - Vec2(1200, 1200)).norm() < 150.0);
        CHECK((two.centers.row(b2.peakCell[1]).transpose() - Vec2(3800, 3800)).norm() < 150.0);
        double tot = 0.0;
        for (const double m : b2.mass) tot += m;
        CHECK_NEAR(tot, two.mass.sum(), 1e-12);

        // 500 m apart with sigma 400: one peak with a shallow dip -> merged at 0.6,
        // kept apart when nothing short of a plateau may merge (persistence 1).
        const CellSet near = latticeCells({{{2000, 2500, 400, 1.0}}, {{3000, 2500, 400, 0.9}}});
        const mapping::PeakBasins merged = mapping::findPeakBasins(near, 0.6);
        const mapping::PeakBasins apart  = mapping::findPeakBasins(near, 1.0);
        CHECK(merged.size() == 1);
        CHECK(apart.size() >= 2);
        // persistence 0 merges everything that touches
        CHECK(mapping::findPeakBasins(two, 0.0).size() == 2);  // two islands, no shared edge
    }

    // --- levels: a partition, densest-first, under the radius ----------------
    {
        const CellSet cells = latticeCells({{{1500, 1500, 350, 1.0}}, {{3500, 2000, 250, 0.6}},
                                            {{2500, 4000, 450, 0.8}}});
        const mapping::PeakBasins b = mapping::findPeakBasins(cells, 0.6);
        const std::vector<double> fr = {0.35, 0.7, 1.0};
        const double R = 400.0;
        const ClusterSet cl = mapping::clusterByPeaks(cells, b, fr, R, ClusterParams{}, 5);

        CHECK(cl.size() > 0);
        CHECK(static_cast<Index>(cl.cellCluster.size()) == cells.size());
        CHECK(cl.maxRadius <= R + 1e-9);
        CHECK(static_cast<Index>(cl.basin.size()) == cl.size());
        CHECK(static_cast<Index>(cl.level.size()) == cl.size());
        CHECK(static_cast<Index>(cl.parent.size()) == cl.size());
        std::vector<int> seen(static_cast<std::size_t>(cells.size()), 0);
        for (Index k = 0; k < cl.size(); ++k)
            for (const Index g : cl.cellIdx[static_cast<std::size_t>(k)]) ++seen[static_cast<std::size_t>(g)];
        CHECK(std::all_of(seen.begin(), seen.end(), [](int s) { return s == 1; }));
        CHECK_NEAR(cl.reward.sum(), cells.mass.sum(), 1e-12);

        // Within a basin every level-l cell is at least as dense as every level-(l+1) cell,
        // and the core holds at least its fraction of the basin.
        for (Index bb = 0; bb < b.size(); ++bb) {
            std::map<int, double> minOf, maxOf, massOf;
            for (Index k = 0; k < cl.size(); ++k) {
                if (cl.basin[static_cast<std::size_t>(k)] != static_cast<int>(bb)) continue;
                const int l = cl.level[static_cast<std::size_t>(k)];
                for (const Index g : cl.cellIdx[static_cast<std::size_t>(k)]) {
                    minOf[l] = minOf.count(l) ? std::min(minOf[l], cells.mass(g)) : cells.mass(g);
                    maxOf[l] = maxOf.count(l) ? std::max(maxOf[l], cells.mass(g)) : cells.mass(g);
                    massOf[l] += cells.mass(g);
                }
                const int par = cl.parent[static_cast<std::size_t>(k)];
                if (l == 0) CHECK(par == -1);
                else CHECK(par >= 0 && cl.level[static_cast<std::size_t>(par)] == l - 1 &&
                           cl.basin[static_cast<std::size_t>(par)] == static_cast<int>(bb));
            }
            for (const auto& [l, mn] : minOf)
                if (maxOf.count(l + 1)) CHECK(mn >= maxOf[l + 1] - 1e-15);
            CHECK(massOf[0] >= fr[0] * b.mass[static_cast<std::size_t>(bb)] - 1e-12);
        }
    }

    // --- the coverage model agrees with the residual-belief metric -----------
    {
        PlannerParams p = smallParams();
        BeliefMapParams bp;
        bp.numCentroids = 6;
        bp.sigmaMin = 200;
        bp.sigmaMax = 400;
        const BeliefField prior = mapgen::generateBeliefMap(5000.0, 10.0, bp, 3);
        const CellSet cells = mapping::extractValidCells(prior, 200.0, 2e-4, false);
        Planner planner(p);
        const PlanningResult r = planner.planFromCells(cells, {Vec2(2300, 2300)});
        const planning::CoverageModel cov(cells, 4, p.fov, p.sensor);

        CHECK_NEAR(cov.totalMass(), cells.mass.sum(), 1e-12);
        CHECK_NEAR(cov.detectedMass({}, 1), 0.0, 1e-12);
        const double model = cov.detectedMass(r.trajectories, 1);
        const double strided = cov.detectedMass(r.trajectories, 4);
        const eval::ResidualBelief rb = eval::computeResidualBelief(prior, r.trajectories, p.fov, p.sensor);
        std::printf("  coverage model %.4f (stride 4: %.4f) vs residual-belief detected %.4f\n", model,
                    strided, rb.detectedMass);
        CHECK(model > 0.05);
        CHECK_NEAR(model, rb.detectedMass, 0.03);
        CHECK_NEAR(strided, model, 0.01);
    }

    // --- detection reach: single-axis swept line and multi-axis --------------
    {
        PlannerParams p = smallParams();
        p.finalize();
        const double S = p.infoAware.slantMargin * p.sensor.beta;
        const double d = planning::detectionReach(p);
        const double phi = std::atan(d / p.droneAltitude);
        CHECK_NEAR(p.droneAltitude / (std::cos(p.sensorTiltAngle) * std::cos(phi)), S, 1e-6);
        p.singleAxisGimbal = false;
        CHECK_NEAR(planning::detectionReach(p), std::sqrt(S * S - 300.0 * 300.0), 1e-9);
        p.droneAltitude = 2000.0;
        CHECK(planning::detectionReach(p) == 0.0);
    }

    // --- the search end to end ------------------------------------------------
    {
        PlannerParams p = smallParams();
        BeliefMapParams bp;
        bp.numCentroids = 8;
        bp.sigmaMin = 150;
        bp.sigmaMax = 350;
        const BeliefField prior = mapgen::generateBeliefMap(5000.0, 10.0, bp, 11);
        const CellSet cells = mapping::extractValidCells(prior, 200.0, 2e-4, false);
        const std::vector<Vec2> st = {Vec2(2300, 2300)};

        Planner plain(p);
        const PlanningResult base = plain.planFromCells(cells, st);
        CHECK(!base.infoAware.enabled);

        p.infoAware.enabled = true;
        Planner smart(p);
        const PlanningResult r = smart.planFromCells(cells, st);
        CHECK(r.infoAware.enabled);
        CHECK(!r.infoAware.candidates.empty());
        CHECK(r.infoAware.candidates.front().label == "baseline");
        CHECK(r.infoAware.chosenScore >= r.infoAware.baselineScore - 1e-12);
        CHECK(r.infoAware.basins >= 1);
        CHECK(r.infoAware.detectionReach > 0.0);

        // the baseline candidate IS the plain plan
        const planning::CoverageModel cov(cells, p.infoAware.subsample, p.fov, p.sensor);
        CHECK_NEAR(r.infoAware.baselineScore, cov.detectedMass(base.trajectories, p.infoAware.lookStride), 1e-12);
        // the returned plan is the one that was scored
        CHECK_NEAR(r.infoAware.chosenScore, cov.detectedMass(r.trajectories, p.infoAware.lookStride), 1e-12);

        // it is an ordinary, budget-verified plan
        CHECK(r.plans.size() == 1);
        const double flown = core::polylineLength(r.plans[0].droneTraj);
        CHECK(flown <= r.plans[0].budget * (1.0 + p.budget.tol) + 1e-6);
        CHECK(r.trajectories[0].drone.allFinite());
        CHECK(static_cast<Index>(r.clusters.cellCluster.size()) == cells.size());

        // ... and the metric agrees that it searched at least as well
        const double resBase = eval::computeResidualBelief(prior, base.trajectories, p.fov, p.sensor).residualMass;
        const double resNew  = eval::computeResidualBelief(prior, r.trajectories, p.fov, p.sensor).residualMass;
        std::printf("  info-aware: chose %s, residual %.4f vs plain %.4f (%zu plans, %.1f s)\n",
                    r.infoAware.chosen.c_str(), resNew, resBase, r.infoAware.candidates.size(),
                    r.infoAware.seconds);
        CHECK(resNew <= resBase + 0.01);

        // deterministic, whatever the thread count
        PlannerParams p1 = p;
        p1.infoAware.threads = 1;
        const PlanningResult r1 = Planner(p1).planFromCells(cells, st);
        CHECK(r1.infoAware.chosen == r.infoAware.chosen);
        CHECK_NEAR(r1.infoAware.chosenScore, r.infoAware.chosenScore, 1e-12);
        CHECK(r1.trajectories[0].drone.rows() == r.trajectories[0].drone.rows());

        // bad level sets are rejected at construction
        bool threw = false;
        try {
            PlannerParams bad = p;
            bad.infoAware.levelSets = {{0.5, 0.9}};
            Planner x(bad);
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        CHECK(threw);
    }

    return mtl::test::report("test_info_aware");
}
