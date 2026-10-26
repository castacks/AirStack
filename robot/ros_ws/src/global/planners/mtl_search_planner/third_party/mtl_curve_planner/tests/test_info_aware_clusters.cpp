// The information-aware cluster search of the curve planner
// (PlannerParams::infoAware): peak basins and mass levels are a partition with
// the right ordering, the search never returns an abstraction whose seed
// objective is worse than the plain clusters', the flown curve keeps every
// guarantee of the plain planner, the plan is deterministic, and with the
// flag off nothing changes.
#include <cmath>
#include <map>

#include "mtl_curve/mapgen/scenario.hpp"
#include "mtl_curve/mapping/cells.hpp"
#include "mtl_curve/mapping/peak_clusters.hpp"
#include "mtl_curve/planner.hpp"
#include "test_util.hpp"

using namespace mtl::curve;

int main() {
    PlannerParams p;
    p.cellSize = 10.0;
    p.verbose = false;
    p.numAgents = 1;
    p.maxFlightTime = 150.0;
    p.optimizer.maxIter = 60;
    p.optimizer.exploreIter = 30;
    p.optimizer.exploreIterWarm = 20;
    p.team.coordinationSweeps = 1;
    p.team.reallocate = false;
    // the search re-scores cluster abstractions: it needs a cluster-dependent
    // seed (the default sector seed does not use the clusters)
    p.team.initStrategies = {InitStrategy::Clusters, InitStrategy::Greedy};
    p.rngSeed = 7;
    const BeliefField belief = mapgen::generateBeliefMap(p.mapSize, p.cellSize, BeliefMapParams{}, p.rngSeed);
    const std::vector<Vec2> starts{Vec2(2000.0, 2000.0)};

    // ---- 1. basins and levels --------------------------------------------------------
    const CellSet cells = mapping::extractValidCells(belief, p.targetCellSize, p.minimumBeliefMass, false);
    const mapping::PeakBasins B = mapping::findPeakBasins(cells, 0.6);
    CHECK(B.size() >= 1);
    double tot = 0.0;
    for (const double m : B.mass) tot += m;
    CHECK_NEAR(tot, cells.mass.sum(), 1e-12);
    for (Index b = 0; b < B.size(); ++b) {
        // a peak is the densest cell of its basin
        for (Index i = 0; i < cells.size(); ++i)
            if (B.basinOfCell[static_cast<std::size_t>(i)] == static_cast<int>(b))
                CHECK(cells.mass(i) <= cells.mass(B.peakCell[static_cast<std::size_t>(b)]) + 1e-15);
    }
    const ClusterSet CS = mapping::clusterByPeaks(cells, B, {0.5, 1.0}, 520.0, p.cluster, p.rngSeed);
    std::vector<int> seen(static_cast<std::size_t>(cells.size()), 0);
    for (Index k = 0; k < CS.size(); ++k) {
        for (const Index i : CS.cellIdx[static_cast<std::size_t>(k)]) {
            ++seen[static_cast<std::size_t>(i)];
            CHECK((cells.centers.row(i) - CS.centroids.row(k)).norm() <= 520.0 + 1e-9);
        }
    }
    for (const int s : seen) CHECK(s == 1);   // a partition
    // within a basin every core cell is at least as dense as every shoulder cell
    std::map<int, double> minCore, maxShoulder;
    for (Index i = 0; i < cells.size(); ++i) {
        const int k = CS.cellCluster[static_cast<std::size_t>(i)];
        const int b = CS.basin[static_cast<std::size_t>(k)];
        if (CS.level[static_cast<std::size_t>(k)] == 0)
            minCore[b] = minCore.count(b) ? std::min(minCore[b], cells.mass(i)) : cells.mass(i);
        else
            maxShoulder[b] = maxShoulder.count(b) ? std::max(maxShoulder[b], cells.mass(i)) : cells.mass(i);
    }
    for (const auto& [b, mc] : minCore)
        if (maxShoulder.count(b)) CHECK(maxShoulder[b] <= mc + 1e-15);

    // ---- 2. the search end to end -------------------------------------------------------
    PlannerParams q = p;
    q.infoAware.enabled = true;
    q.infoAware.maxMoves = 4;
    q.infoAware.scoreIters = 10;
    Planner planner(q);
    const PlanningResult r = planner.plan(belief, starts);
    const InfoAwareReport& ia = r.infoAware;
    CHECK(ia.enabled);
    CHECK(!ia.candidates.empty());
    CHECK(ia.candidates.front().label == "plain");
    CHECK(ia.chosenSeedJ <= ia.plainSeedJ + 1e-12);
    for (const InfoAwareCandidate& c : ia.candidates)
        if (c.label.find('+') == std::string::npos) CHECK(ia.chosenSeedJ <= c.seedJ + 1e-12);
    CHECK(r.plans.size() == 1);
    CHECK(r.plans[0].feasible);
    CHECK_NEAR(r.trajectories[0].length, p.budgetDist(), 0.002 * p.budgetDist());

    // ---- 3. deterministic ----------------------------------------------------------------
    Planner planner2(q);
    const PlanningResult r2 = planner2.plan(belief, starts);
    CHECK(r2.infoAware.chosen == ia.chosen);
    CHECK(r2.plans[0].v == r.plans[0].v);

    // ---- 4. off = the plain planner ----------------------------------------------------------
    PlannerParams o = p;
    o.infoAware.enabled = false;
    Planner pa(o), pb(p);
    const PlanningResult ra = pa.plan(belief, starts), rb = pb.plan(belief, starts);
    CHECK(!ra.infoAware.enabled);
    CHECK(ra.plans[0].v == rb.plans[0].v);

    return test::report("test_info_aware_clusters");
}
