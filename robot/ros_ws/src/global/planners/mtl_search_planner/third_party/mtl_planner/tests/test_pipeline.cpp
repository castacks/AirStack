// The package end to end: construct, plan, fly, score - and the invariants the
// budget is supposed to guarantee.
#include <cmath>
#include <iostream>
#include <sstream>

#include "mtl/core/numeric.hpp"
#include "mtl/eval/detection.hpp"
#include "mtl/eval/geometry_audit.hpp"
#include "mtl/eval/report.hpp"
#include "mtl/mapgen/scenario.hpp"
#include "mtl/planner.hpp"
#include "test_util.hpp"

using namespace mtl;

namespace {

PlannerParams fastParams() {
    PlannerParams p;
    // A 10 m belief grid instead of 1 m: 500x500 rather than 5001x5001, which
    // keeps the test quick without changing anything the planner reasons about
    // (the cells are 200 m blocks either way).
    p.cellSize  = 10.0;
    p.numAgents = 3;
    p.verbose   = false;
    p.verifyGeometryVerbose = false;
    return p;
}

std::vector<Vec2> starts(int n) {
    std::vector<Vec2> s;
    for (int a = 0; a < n; ++a) s.emplace_back(2000.0 + a, 2000.0 + a);
    return s;
}

void checkResult(const PlanningResult& r, const PlannerParams& p, const char* label) {
    std::printf("  [%s] %lld cells, %lld clusters, %lld reached, info %.1f%%\n", label,
                static_cast<long long>(r.cells.size()),
                static_cast<long long>(r.clusters.size()),
                static_cast<long long>(r.team.reachedClusters.size()),
                100.0 * r.team.infoFraction);

    CHECK(static_cast<int>(r.plans.size()) == p.numAgents);
    CHECK(static_cast<int>(r.trajectories.size()) == p.numAgents);
    CHECK(r.numSteps() > 1);

    for (std::size_t a = 0; a < r.trajectories.size(); ++a) {
        const AgentTrajectory& t = r.trajectories[a];
        // Every agent is padded onto ONE timeline - a host simulation steps them
        // together, so a ragged result would be unusable.
        CHECK(t.drone.rows() == r.numSteps());
        CHECK(t.sensor.rows() == r.numSteps());
        CHECK(t.rpy.rows() == r.numSteps());
        CHECK(t.drone.allFinite());
        CHECK(t.sensor.allFinite());
        CHECK(t.rpy.allFinite());

        // THE BUDGET IS A HARD GUARANTEE, measured on the flown arc.
        const AgentPlan& plan = r.plans[a];
        if (std::isfinite(plan.budget)) {
            const double flown = core::polylineLength(plan.droneTraj);
            CHECK(flown <= plan.budget * (1.0 + p.budget.tol) + 1e-6);
            CHECK_NEAR(plan.flownLength, flown, 1e-6);
        }

        // Serviced cells are real cell indices, each named once.
        for (const Index c : plan.servicedCellIdx) CHECK(c >= 0 && c < r.cells.size());
        CHECK(std::is_sorted(plan.servicedCellIdx.begin(), plan.servicedCellIdx.end()));
    }

    // Team bookkeeping partitions the cells and the clusters exactly.
    CHECK(static_cast<Index>(r.team.servicedCellIdx.size() + r.team.unservicedCellIdx.size()) ==
          r.cells.size());
    CHECK(static_cast<Index>(r.team.reachedClusters.size() + r.team.unreachedClusters.size()) ==
          r.clusters.size());
    CHECK(r.team.info <= r.team.infoTotal + 1e-9);
    CHECK(r.servicedCenters.rows() == static_cast<Index>(r.team.servicedCellIdx.size()));

    // The gimbal can only realise what was planned.
    CHECK(r.realizedCellIdx.size() <= r.team.servicedCellIdx.size());
}

}  // namespace

int main() {
    // --- the reference scenario, single-axis tilted mount ------------------
    PlannerParams p = fastParams();
    const BeliefField belief =
        mapgen::generateBeliefMap(p.mapSize, p.cellSize, BeliefMapParams{}, p.rngSeed);
    CHECK(belief.rows() == 501 && belief.cols() == 501);
    CHECK(belief.values.maxCoeff() <= BeliefMapParams{}.beliefCap + 1e-12);
    CHECK(belief.values.minCoeff() >= 0.0);

    std::vector<Target> targets = mapgen::generateTargetPoses(belief, 50, p.rngSeed + 1);
    CHECK(targets.size() == 50);
    for (const Target& t : targets) {
        CHECK(t.pose.x() >= 0.0 && t.pose.x() <= p.mapSize);
        CHECK(t.pose.y() >= 0.0 && t.pose.y() <= p.mapSize);
    }

    {
        Planner planner(p);
        const PlanningResult r = planner.plan(belief, starts(p.numAgents));
        checkResult(r, p, "single-axis, tilted");

        // The geometry audit must pass on the real pipeline output, not just on
        // the synthetic tracks in test_geometry.
        for (const AgentTrajectory& t : r.trajectories) {
            if (!t.scheduled) continue;
            const eval::GeometryReport rep =
                eval::verifySensorGeometry(t.drone, t.sensor, t.rpy, t.diagnostics, false);
            CHECK(rep.pass);
        }

        // Scoring runs, and detection probability stays a probability.
        std::vector<Target> tg = targets;
        const eval::DetectionSummary ds = eval::updateTargetDetectionProbs(
            r.trajectories, r.timeVec, tg, p.fov, p.sensor, p.detectionThreshold);
        for (const Target& t : tg) CHECK(t.detectionProb >= 0.0 && t.detectionProb <= 1.0);
        CHECK(ds.nDetected + ds.nMissed == static_cast<Index>(tg.size()));
        std::printf("  [single-axis, tilted] detected %lld/%lld targets\n",
                    static_cast<long long>(ds.nDetected), static_cast<long long>(tg.size()));

        // The reports must not throw or produce nothing.
        std::ostringstream oss;
        eval::reportBudgetSummary(oss, r, p);
        eval::reportDetectionSummary(oss, tg, p);
        eval::reportGimbalCoverage(oss, r);
        CHECK(oss.str().size() > 200);
    }

    // --- multi-axis gimbal: nothing is scheduled, everything planned is swept
    {
        PlannerParams mp = fastParams();
        mp.singleAxisGimbal = false;
        Planner planner(mp);
        const PlanningResult r = planner.plan(belief, starts(mp.numAgents));
        checkResult(r, mp, "multi-axis");
        for (const AgentTrajectory& t : r.trajectories) CHECK(!t.scheduled);
        // Planned == realized in this mode, by construction.
        CHECK(r.realizedCellIdx.size() == r.team.servicedCellIdx.size());
    }

    // --- a bigger budget collects at least as much information -------------
    {
        double prevInfo = -1.0;
        for (const double seconds : {150.0, 250.0, 400.0}) {
            PlannerParams bp = fastParams();
            bp.maxFlightTime = seconds;
            Planner planner(bp);
            const PlanningResult r = planner.plan(belief, starts(bp.numAgents));
            std::printf("  [budget %.0f s] info %.1f%%, %lld cells\n", seconds,
                        100.0 * r.team.infoFraction,
                        static_cast<long long>(r.team.servicedCellIdx.size()));
            // Heuristic, so allow a small regression rather than demanding strict
            // monotonicity; a real regression would be far larger than this.
            CHECK(r.team.info >= prevInfo * 0.90);
            prevInfo = r.team.info;
        }
    }

    // --- an unbudgeted run services every cell it owns ---------------------
    {
        PlannerParams up = fastParams();
        up.maxFlightTime     = kInf;
        up.maxFlightDistance = kInf;
        up.numAgents         = 2;
        Planner planner(up);
        const PlanningResult r = planner.plan(belief, starts(up.numAgents));
        checkResult(r, up, "unbudgeted");
        CHECK(r.team.unreachedClusters.empty());
        CHECK(r.team.unservicedCellIdx.empty());
        CHECK_NEAR(r.team.infoFraction, 1.0, 1e-9);
    }

    // --- planFromCells: the host-supplies-its-own-mapping entry point ------
    {
        PlannerParams cp = fastParams();
        Planner planner(cp);

        CellSet cells;
        cells.centers.resize(6, 2);
        cells.centers << 1000, 1000, 1200, 1000, 3000, 3000, 3200, 3100, 2000, 4000, 4000, 1500;
        cells.mass = VecX::Ones(6);
        cells.totalMapMass = 6.0;
        cells.retainedMass = 6.0;

        const PlanningResult r = planner.planFromCells(cells, starts(cp.numAgents));
        CHECK(r.cells.size() == 6);
        CHECK(r.clusters.size() > 0);
        CHECK(r.numSteps() > 1);
    }

    // --- a bad parameter set is refused at construction, not mid-flight ----
    {
        bool threw = false;
        try {
            PlannerParams bad = fastParams();
            bad.sensorTiltAngle = deg2rad(89.0);  // past the look-angle stop
            Planner planner(bad);
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        CHECK(threw);

        threw = false;
        try {
            PlannerParams bad = fastParams();
            bad.numAgents = 0;
            Planner planner(bad);
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        CHECK(threw);

        threw = false;
        try {
            Planner planner(fastParams());
            planner.plan(belief, starts(1));  // fewer launch points than agents
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        CHECK(threw);
    }

    // --- determinism: the same parameters give the same plan ---------------
    {
        Planner a(fastParams());
        Planner b(fastParams());
        const PlanningResult ra = a.plan(belief, starts(3));
        const PlanningResult rb = b.plan(belief, starts(3));
        CHECK_NEAR(ra.team.info, rb.team.info, 1e-12);
        CHECK(ra.team.servicedCellIdx == rb.team.servicedCellIdx);
        CHECK(ra.numSteps() == rb.numSteps());
    }

    return test::report("test_pipeline");
}
