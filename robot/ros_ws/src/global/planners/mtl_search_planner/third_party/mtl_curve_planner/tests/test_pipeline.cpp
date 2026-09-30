// The package end to end: construct, plan, fly, score - and the invariants the
// planner guarantees: one padded timeline, exact curve length, the turn
// radius, the residual belief bookkeeping, planFromCells, determinism, and
// construction-time rejection of bad parameters.
#include <cmath>
#include <iostream>
#include <sstream>

#include "mtl_curve/eval/detection.hpp"
#include "mtl_curve/eval/report.hpp"
#include "mtl_curve/mapgen/scenario.hpp"
#include "mtl_curve/mapping/cells.hpp"
#include "mtl_curve/planner.hpp"
#include "test_util.hpp"

using namespace mtl::curve;

namespace {

PlannerParams fastParams() {
    PlannerParams p;
    // A 10 m belief grid instead of 1 m, and a lighter optimiser: the
    // invariants are the point here, not the last percent of coverage.
    p.cellSize = 10.0;
    p.verbose = false;
    p.optimizer.maxIter = 60;
    p.optimizer.exploreIter = 30;
    p.optimizer.exploreIterWarm = 20;
    p.team.coordinationSweeps = 1;
    p.team.maxReallocTrials = 1;
    return p;
}

std::vector<Vec2> starts(int n) {
    std::vector<Vec2> s;
    for (int a = 0; a < n; ++a) s.emplace_back(2000.0 + a, 2000.0 + a);
    return s;
}

void checkResult(const PlanningResult& r, const PlannerParams& p, const char* label) {
    std::printf("  [%s] %lld cells, %lld clusters, fast residual %.4f, cells swept %.1f%%\n", label,
                static_cast<long long>(r.cells.size()), static_cast<long long>(r.clusters.size()), r.team.Jfinal,
                100.0 * r.team.infoFraction);
    CHECK(static_cast<int>(r.plans.size()) == p.numAgents);
    CHECK(static_cast<int>(r.trajectories.size()) == p.numAgents);
    CHECK(r.numSteps() > 1);
    for (std::size_t a = 0; a < r.trajectories.size(); ++a) {
        const AgentTrajectory& t = r.trajectories[a];
        CHECK(t.drone.rows() == r.numSteps());
        CHECK(t.sensor.rows() == r.numSteps());
        CHECK(t.rpy.rows() == r.numSteps());
        CHECK(t.gimbalAngle.size() == r.numSteps());
        CHECK(t.drone.allFinite() && t.sensor.allFinite() && t.rpy.allFinite());
        const AgentPlan& pl = r.plans[a];
        CHECK(pl.feasible);
        CHECK(std::abs(pl.flownLength - pl.budget) <= 0.002 * pl.budget);
        CHECK(pl.maxKappa <= 1.0 / p.minTurnRadius + 1e-4);
        CHECK_NEAR(t.drone(0, 2), p.droneAltitude + static_cast<double>(a) * p.team.altitudeStagger, 1e-9);
    }
    CHECK(static_cast<Index>(r.team.servicedCellIdx.size() + r.team.unservicedCellIdx.size()) == r.cells.size());
    CHECK(r.team.Jfinal <= r.team.Jstatic + 1e-12);
    CHECK(r.team.Jfinal > 0.0 && r.team.Jfinal < 1.0);
}

}  // namespace

int main() {
    // --- plan from a belief grid, score with the (copied) eval library ------
    PlannerParams p = fastParams();
    const BeliefField belief = mapgen::generateBeliefMap(p.mapSize, p.cellSize, BeliefMapParams{}, p.rngSeed);
    std::vector<Target> targets = mapgen::generateTargetPoses(belief, 50, p.rngSeed + 1);
    CHECK_NEAR(belief.values.sum(), 1.0, 1e-9);

    Planner planner(p);
    const PlanningResult r = planner.plan(belief, starts(p.numAgents));
    checkResult(r, planner.params(), "belief");

    eval::updateTargetDetectionProbs(r.trajectories, r.timeVec, targets, p.fov, p.sensor, p.detectionThreshold);
    const eval::ResidualBelief rb = eval::computeResidualBelief(belief, r.trajectories, p.fov, p.sensor);
    std::printf("  residual belief %.4f (fast model %.4f)\n", rb.residualMass, r.team.Jfinal);
    CHECK(rb.residualMass > 0.0 && rb.residualMass < 1.0);
    CHECK(rb.residual.maxCoeff() <= belief.values.maxCoeff() / belief.values.sum() + 1e-15);
    // the fast model is calibrated to the evaluator: same ballpark
    CHECK(std::abs(rb.residualMass - r.team.Jfinal) < 0.05);
    // the residual at a target's pixel equals its miss probability
    for (std::size_t i = 0; i < 5; ++i) {
        const Target& t = targets[i];
        const Index c = static_cast<Index>(std::lround(t.pose.x() / belief.gridResX()));
        const Index rr = static_cast<Index>(std::lround(t.pose.y() / belief.gridResY()));
        const double prior = belief.values(rr, c) / belief.values.sum();
        if (prior > 0.0) CHECK_NEAR(rb.residual(rr, c) / prior, t.pMissTotal, 1e-9);
    }
    {
        std::ostringstream os;
        CHECK(eval::reportCurveSummary(os, r, planner.params()));
        eval::reportResidualBelief(os, rb);
        CHECK(os.str().find("Residual belief mass") != std::string::npos);
    }

    // --- determinism: same parameters, same plan -----------------------------
    {
        Planner again(p);
        const PlanningResult r2 = again.plan(belief, starts(p.numAgents));
        for (std::size_t a = 0; a < r.plans.size(); ++a) CHECK((r.plans[a].v - r2.plans[a].v).norm() == 0.0);
    }

    // --- planFromCells (the JSON adapter's path), return-home mode -----------
    {
        PlannerParams q = fastParams();
        q.curve.endpointModes = {EndpointMode::ReturnHome};
        q.team.reallocate = false;
        const CellSet cells = mapping::extractValidCells(belief, q.targetCellSize, q.minimumBeliefMass, false);
        Planner pc(q);
        const PlanningResult rc = pc.planFromCells(cells, starts(q.numAgents));
        checkResult(rc, pc.params(), "cells, return_home");
        for (std::size_t a = 0; a < rc.plans.size(); ++a) CHECK(rc.plans[a].endpointError < 1.0);
    }

    // --- bad parameters are refused at construction -------------------------
    auto refused = [](PlannerParams q) {
        try {
            Planner x(q);
        } catch (const std::invalid_argument&) {
            return true;
        }
        return false;
    };
    {
        PlannerParams q = fastParams();
        q.maxFlightTime = kInf;
        CHECK(refused(q));                        // the curve IS the budget
    }
    {
        PlannerParams q = fastParams();
        q.team.altitudeStagger = 400.0;           // agent 3 out of sensor range
        CHECK(refused(q));
    }
    {
        PlannerParams q = fastParams();
        q.curve.endpointModes = {EndpointMode::Open, EndpointMode::Open};  // 2 modes for 3 agents
        CHECK(refused(q));
    }
    {
        bool threw = false;
        try {
            planner.plan(belief, starts(1));
        } catch (const std::invalid_argument&) {
            threw = true;
        }
        CHECK(threw);                             // fewer launch points than agents
    }
    return test::report("test_pipeline");
}
