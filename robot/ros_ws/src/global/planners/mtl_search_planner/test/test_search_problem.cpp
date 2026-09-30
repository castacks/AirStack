// Copyright (c) 2026 Carnegie Mellon University
// SPDX-License-Identifier: BSD-3-Clause-Clear
//
// Unit tests for the ROS-free scenario/plan adapter (no rclcpp needed).
#include <gtest/gtest.h>

#include <cmath>
#include <string>

#include "mtl_search_planner/search_problem.hpp"

namespace {

// Two agents, three tight cell blobs, NED area centred at (50, -20): exercises
// the south-west anchoring of the mtl frame and per-agent homes.
std::string tinyScenario(double budgetS = 120.0, bool singleAxis = true) {
    std::string s = R"({
  "schema": "mtl.scenario/1",
  "mission": {"name": "tiny", "seed": 7, "area": {"size_m": 200.0, "center_ned": [50.0, -20.0], "belief_res_m": 2.0}},
  "aircraft": {"altitude_m": 30.0, "speed_mps": 6.0, "min_turn_radius_m": 12.0, "dt": 0.1, "dubins_step_m": 0.5},
  "mapping": {"target_cell_size_m": 20.0, "minimum_belief_mass": 0.002, "max_cluster_radius_m": 45.0,
              "kmeans_replicates": 3, "kmeans_max_iter": 100},
  "sensor": {"fov_deg": 60.0, "single_axis_gimbal": SINGLE, "tilt_deg": 30.0, "max_slant_range_m": 90.0,
             "detection": {"a": 1.1, "b": 0.1, "c": 61.0, "beta": 61.0, "p_out_of_range": 1e-6, "threshold": 0.9}},
  "gimbal": {},
  "team": {"max_flight_time_s": BUDGET, "max_flight_distance_m": null,
           "agents": [{"name": "robot_1", "start_ned": [-40.0, -100.0], "home_ned": [-40.0, -100.0]},
                      {"name": "robot_2", "start_ned": [-40.0, -88.0],  "home_ned": [-40.0, -88.0]}]},
  "cells": {"centers": [[60, -30], [60, -10], [80, -30], [80, -10], [120, 40], [120, 60], [0, 50], [20, 50]],
            "mass": [0.12, 0.10, 0.09, 0.08, 0.15, 0.14, 0.06, 0.07], "total_map_mass": 1.0},
  "solver": {}, "verbose": false, "verify_geometry": false, "verify_geometry_verbose": false
})";
    auto rep = [&s](const std::string& from, const std::string& to) {
        s.replace(s.find(from), from.size(), to);
    };
    rep("BUDGET", std::to_string(budgetS));
    rep("SINGLE", singleAxis ? "true" : "false");
    return s;
}

}  // namespace

TEST(Frame, NedMtlRoundTripAndSouthWestAnchor) {
    const auto p = mtl_search::parseScenario(tinyScenario());
    EXPECT_DOUBLE_EQ(p.frame.nMin, 50.0 - 100.0);
    EXPECT_DOUBLE_EQ(p.frame.eMin, -20.0 - 100.0);
    for (double n : {-50.0, 0.0, 33.3, 150.0}) {
        for (double e : {-120.0, -20.0, 80.0}) {
            const mtl::Vec2 xy = p.frame.toMtl(n, e);
            double n2 = 0, e2 = 0;
            p.frame.fromMtl(xy.x(), xy.y(), n2, e2);
            EXPECT_NEAR(n, n2, 1e-12);
            EXPECT_NEAR(e, e2, 1e-12);
        }
    }
    // x East, y North in mtl
    const mtl::Vec2 sw = p.frame.toMtl(p.frame.nMin, p.frame.eMin);
    EXPECT_NEAR(sw.norm(), 0.0, 1e-12);
    EXPECT_NEAR(mtl_search::Frame::yawToNed(0.0), mtl::kPi / 2.0, 1e-12);  // East
}

TEST(Scenario, ParsesTeamCellsAndParams) {
    const auto p = mtl_search::parseScenario(tinyScenario(90.0));
    ASSERT_EQ(p.agents.size(), 2u);
    EXPECT_EQ(p.agentIndex("robot_2"), 1);
    EXPECT_EQ(p.agentIndex("robot_9"), -1);
    EXPECT_EQ(p.cells.size(), 8);
    EXPECT_DOUBLE_EQ(p.params.avgDroneSpeed, 6.0);
    EXPECT_DOUBLE_EQ(p.params.maxFlightTime, 90.0);
    EXPECT_TRUE(std::isinf(p.params.maxFlightDistance));
    EXPECT_NEAR(p.params.sensorTiltAngle, mtl::deg2rad(30.0), 1e-12);
    const mtl::Vec3 home = p.agents[1].homeEnu();
    EXPECT_DOUBLE_EQ(home.x(), -88.0);  // east
    EXPECT_DOUBLE_EQ(home.y(), -40.0);  // north
}

TEST(Scenario, CellMassesAreProbabilities) {
    // The host prior is a PMF: cell masses are P(target in cell), the map sums to 1.
    const auto p = mtl_search::parseScenario(tinyScenario());
    EXPECT_NEAR(p.cells.totalMapMass, 1.0, 1e-12);
    EXPECT_NEAR(p.cells.retainedMass, 0.81, 1e-12);
    EXPECT_NEAR(p.cells.massNorm.sum(), 1.0, 1e-12);
    EXPECT_DOUBLE_EQ(p.params.minimumBeliefMass, 0.002);
    EXPECT_DOUBLE_EQ(p.cells.minBeliefMass, 0.002);
    // The retired key is ignored: an old scenario falls back to the planner default.
    std::string s = tinyScenario();
    s.replace(s.find("\"minimum_belief_mass\": 0.002"), 28, "\"mean_information_thresh\": 0.05");
    EXPECT_DOUBLE_EQ(mtl_search::parseScenario(s).params.minimumBeliefMass,
                     mtl::PlannerParams{}.minimumBeliefMass);
}

TEST(Scenario, RejectsOutOfRangeMinimumBeliefMass) {
    std::string s = tinyScenario();
    s.replace(s.find("\"minimum_belief_mass\": 0.002"), 28, "\"minimum_belief_mass\": 1.5");
    const auto p = mtl_search::parseScenario(s);
    EXPECT_THROW(p.params.validate(), std::invalid_argument);
}

TEST(Plan, MassScaleDoesNotChangeThePlan) {
    // The optimiser compares rewards as ratios: probabilities and the old raw
    // (x pixel-area) masses must plan identically.
    std::string raw = tinyScenario(120.0);
    raw.replace(raw.find("[0.12, 0.10, 0.09, 0.08, 0.15, 0.14, 0.06, 0.07]"), 48,
                "[120, 100, 90, 80, 150, 140, 60, 70]");
    raw.replace(raw.find("\"total_map_mass\": 1.0"), 21, "\"total_map_mass\": 1000");
    const auto pp = mtl_search::parseScenario(tinyScenario(120.0));
    const auto pr = mtl_search::parseScenario(raw);
    const auto rp = mtl_search::solve(pp);
    const auto rr = mtl_search::solve(pr);
    ASSERT_EQ(rp.trajectories.size(), rr.trajectories.size());
    for (std::size_t a = 0; a < rp.trajectories.size(); ++a) {
        EXPECT_EQ(rp.plans[a].servicedCellIdx, rr.plans[a].servicedCellIdx);
        ASSERT_EQ(rp.trajectories[a].drone.rows(), rr.trajectories[a].drone.rows());
        EXPECT_LT((rp.trajectories[a].drone - rr.trajectories[a].drone).cwiseAbs().maxCoeff(), 1e-9);
    }
    EXPECT_NEAR(rp.team.infoFraction, rr.team.infoFraction, 1e-12);
}

TEST(Scenario, RejectsBadSchemaAndMissingCells) {
    EXPECT_THROW(mtl_search::parseScenario(R"({"schema": "nope"})"), std::runtime_error);
    std::string s = tinyScenario();
    s.replace(s.find("\"cells\""), 7, "\"xcells\"");
    EXPECT_THROW(mtl_search::parseScenario(s), std::runtime_error);
}

TEST(Plan, AltitudeOffsetShiftsOnlyThatAgentsTrack) {
    std::string s = tinyScenario(120.0);
    s.replace(s.find("\"home_ned\": [-40.0, -88.0]}"), 27,
              "\"home_ned\": [-40.0, -88.0], \"altitude_offset_m\": 1.0}");
    const auto p = mtl_search::parseScenario(s);
    const auto r = mtl_search::solve(p);
    const auto t0 = mtl_search::buildAgentTrack(r, p, 0);
    const auto t1 = mtl_search::buildAgentTrack(r, p, 1);
    EXPECT_NEAR(t0.samples.front().z, 30.0, 1e-6);
    EXPECT_NEAR(t0.altitude, 30.0, 1e-9);
    ASSERT_GT(t1.samples.size(), 10u);
    for (const auto& smp : t1.samples) EXPECT_NEAR(smp.z, 31.0, 1e-6);
    EXPECT_NEAR(t1.altitude, 31.0, 1e-9);
    // the boresight ground points do not move with the layer
    EXPECT_NEAR(t1.samples[t1.samples.size() / 2].bz, 0.0, 1e-9);
}

TEST(Plan, RunOutDistanceIsConfigurable) {
    std::string s = tinyScenario(120.0);
    s.replace(s.find("\"solver\": {}"), 12, "\"solver\": {\"extend_dist_m\": 24.0}");
    const auto p = mtl_search::parseScenario(s);
    EXPECT_NEAR(p.params.extension.extendDist, 24.0, 1e-12);
    EXPECT_NEAR(mtl_search::parseScenario(tinyScenario()).params.extension.extendDist,
                mtl::ExtensionParams{}.extendDist, 1e-12);
}

TEST(Plan, AgentTrackIsInMapFrameStartsAtHomeAndRespectsBudget) {
    const auto p = mtl_search::parseScenario(tinyScenario(120.0));
    const auto r = mtl_search::solve(p);
    for (int a = 0; a < 2; ++a) {
        const auto tr = mtl_search::buildAgentTrack(r, p, a);
        ASSERT_GT(tr.samples.size(), 10u);
        // starts where the robot spawned: map origin
        EXPECT_NEAR(tr.samples.front().x, 0.0, 1e-6);
        EXPECT_NEAR(tr.samples.front().y, 0.0, 1e-6);
        EXPECT_NEAR(tr.samples.front().z, 30.0, 1e-6);
        // arc length is monotone and within the endurance budget
        for (std::size_t k = 1; k < tr.samples.size(); ++k) {
            EXPECT_GE(tr.samples[k].arc, tr.samples[k - 1].arc);
        }
        EXPECT_LE(tr.totalArc(), tr.budget * 1.01 + 1.0);
        // boresight is on the ground plane (map z = -homeUp = 0)
        EXPECT_NEAR(tr.samples[tr.samples.size() / 2].bz, 0.0, 1e-9);
        // the tail padding was trimmed: last sample still moves
        const auto& l = tr.samples.back();
        const auto& m = tr.samples[tr.samples.size() - 2];
        EXPECT_GT(std::hypot(l.x - m.x, l.y - m.y), 0.05);
    }
}

TEST(Plan, MapFrameIsWorldMinusHome) {
    const auto p = mtl_search::parseScenario(tinyScenario(120.0));
    const auto r = mtl_search::solve(p);
    const auto tr = mtl_search::buildAgentTrack(r, p, 1);
    const std::size_t k = tr.samples.size() / 3;
    double n = 0, e = 0;
    p.frame.fromMtl(r.trajectories[1].drone(static_cast<mtl::Index>(k), 0),
                    r.trajectories[1].drone(static_cast<mtl::Index>(k), 1), n, e);
    EXPECT_NEAR(tr.samples[k].x, e - (-88.0), 1e-9);
    EXPECT_NEAR(tr.samples[k].y, n - (-40.0), 1e-9);
}

TEST(Plan, DeterministicAcrossSolves) {
    // Every robot container solves the same team problem independently; they
    // must agree bit for bit or the team would fly inconsistent plans.
    const auto p = mtl_search::parseScenario(tinyScenario(120.0));
    const auto r1 = mtl_search::solve(p);
    const auto r2 = mtl_search::solve(mtl_search::parseScenario(tinyScenario(120.0)));
    EXPECT_EQ(mtl_search::teamPlanJson(r1, p), mtl_search::teamPlanJson(r2, p));
}

TEST(Plan, MultiAxisModeHasNoSchedule) {
    const auto p = mtl_search::parseScenario(tinyScenario(200.0, /*singleAxis=*/false));
    const auto r = mtl_search::solve(p);
    const auto tr = mtl_search::buildAgentTrack(r, p, 0);
    EXPECT_FALSE(tr.singleAxis);
    EXPECT_DOUBLE_EQ(tr.tilt, 0.0);
    for (const auto& s : tr.samples) EXPECT_DOUBLE_EQ(s.gimbalPhi, 0.0);
}

TEST(Plan, JsonOutputsParse) {
    const auto p = mtl_search::parseScenario(tinyScenario());
    const auto r = mtl_search::solve(p);
    const auto team = jsonmini::parse(mtl_search::teamPlanJson(r, p));
    EXPECT_EQ(team["schema"].text(""), "mtl.plan/1");
    EXPECT_EQ(team["agents"].array().size(), 2u);
    const auto tr = mtl_search::buildAgentTrack(r, p, 0);
    const auto one = jsonmini::parse(mtl_search::agentTrackJson(tr, p));
    EXPECT_EQ(one["agent"].text(""), "robot_1");
    EXPECT_EQ(one["samples"]["x_map"].array().size(), tr.samples.size());
}

// ---- the planner-mode toggle (info_aware) -----------------------------------
TEST(InfoAware, OffByDefaultAndParsedFromTheScenario) {
    const auto off = mtl_search::parseScenario(tinyScenario());
    EXPECT_FALSE(off.params.infoAware.enabled);
    EXPECT_TRUE(off.reportAlternative);

    std::string s = tinyScenario();
    s.replace(s.find("\"solver\": {}"), 12,
              "\"solver\": {}, \"info_aware\": {\"enabled\": true, \"report_both\": false, "
              "\"level_sets\": [[1.0], [0.4, 1.0]], \"max_moves\": 3, \"threads\": 2, \"persistence\": 0.5}");
    const auto on = mtl_search::parseScenario(s);
    EXPECT_TRUE(on.params.infoAware.enabled);
    EXPECT_FALSE(on.reportAlternative);
    ASSERT_EQ(on.params.infoAware.levelSets.size(), 2u);
    EXPECT_NEAR(on.params.infoAware.levelSets[1][0], 0.4, 1e-12);
    EXPECT_EQ(on.params.infoAware.maxMoves, 3);
    EXPECT_EQ(on.params.infoAware.threads, 2);
    EXPECT_NEAR(on.params.infoAware.persistence, 0.5, 1e-12);
}

TEST(InfoAware, SolveFliesTheToggleAndTheAlternativeIsTheOtherMode) {
    std::string s = tinyScenario(120.0);
    s.replace(s.find("\"solver\": {}"), 12,
              "\"solver\": {}, \"info_aware\": {\"enabled\": true, \"max_moves\": 2, \"threads\": 1}");
    const auto p = mtl_search::parseScenario(s);
    const auto flown = mtl_search::solve(p);
    const auto alt   = mtl_search::solveAlternative(p);
    EXPECT_EQ(mtl_search::plannerMode(flown), "info_aware");
    EXPECT_EQ(mtl_search::plannerMode(alt), "plain");
    EXPECT_GE(flown.infoAware.chosenScore, flown.infoAware.baselineScore - 1e-12);

    // the plain mode through the toggle is exactly the plain planner
    const auto plain = mtl_search::solve(mtl_search::parseScenario(tinyScenario(120.0)));
    ASSERT_EQ(plain.trajectories.size(), alt.trajectories.size());
    EXPECT_EQ(plain.trajectories[0].drone.rows(), alt.trajectories[0].drone.rows());
    EXPECT_TRUE(plain.trajectories[0].drone.isApprox(alt.trajectories[0].drone));

    for (int a = 0; a < 2; ++a) {
        const auto tr = mtl_search::buildAgentTrack(flown, p, a);
        EXPECT_EQ(tr.plannerMode, "info_aware");
        EXPECT_LE(tr.flownLength, tr.budget * 1.01);
        const auto js = jsonmini::parse(mtl_search::agentTrackJson(tr, p));
        EXPECT_EQ(js["planner_mode"].text(""), "info_aware");
    }
    const auto team = jsonmini::parse(mtl_search::teamPlanJson(flown, p));
    EXPECT_EQ(team["meta"]["planner_mode"].text(""), "info_aware");
    EXPECT_FALSE(team["meta"]["info_aware"]["chosen"].text("").empty());
    EXPECT_TRUE(team["meta"]["info_aware"]["candidates"].isArray());
    const auto teamAlt = jsonmini::parse(mtl_search::teamPlanJson(alt, p));
    EXPECT_EQ(teamAlt["meta"]["planner_mode"].text(""), "plain");
}

// =============================================================================
//  The parameterized-curve planner (planner.type = curve)
// =============================================================================
namespace {

/// tinyScenario() with a "planner" block (and optionally a "curve" block) spliced in.
std::string curveScenario(const std::string& curveBlock = "", double budgetS = 120.0,
                          const std::string& plannerBlock = R"({"type": "curve"})") {
    std::string s = tinyScenario(budgetS);
    std::string extra = "\"solver\": {}, \"planner\": " + plannerBlock;
    if (!curveBlock.empty()) extra += ", \"curve\": " + curveBlock;
    s.replace(s.find("\"solver\": {}"), 12, extra);
    return s;
}

/// One curve solve of the default tiny curve scenario, shared by the tests below
/// (a curve plan takes seconds; every test reads the same deterministic result).
struct CurveFixture {
    mtl_search::SearchProblem problem = mtl_search::parseScenario(curveScenario());
    mtl_search::SearchResult result = mtl_search::solveFlown(problem);
    static const CurveFixture& get() {
        static const CurveFixture f;
        return f;
    }
};

double wrapPi(double a) { return std::atan2(std::sin(a), std::cos(a)); }

}  // namespace

TEST(CurveScenario, PlannerTypeDefaultsToOrienteering) {
    const auto p = mtl_search::parseScenario(tinyScenario());
    EXPECT_EQ(p.plannerType, mtl_search::PlannerType::Orienteering);
    EXPECT_TRUE(p.compareOrienteering);
    EXPECT_EQ(p.gimbalLaw, "open_loop");
    const auto c = mtl_search::parseScenario(curveScenario("", 120.0, R"({"type": "curve", "compare_orienteering": false})"));
    EXPECT_EQ(c.plannerType, mtl_search::PlannerType::Curve);
    EXPECT_FALSE(c.compareOrienteering);
    EXPECT_STREQ(mtl_search::toString(c.plannerType), "curve");
}

TEST(CurveScenario, SharedKeysAndAutoScalingMirrorMtlcPlan) {
    // No curve block: every reference length is scaled to the 200 m / beta 61 m mission.
    const auto p = mtl_search::parseScenario(curveScenario());
    const mtl::curve::PlannerParams& c = p.curveParams;
    const mtl::curve::PlannerParams ref;
    EXPECT_DOUBLE_EQ(c.mapSize, 200.0);
    EXPECT_DOUBLE_EQ(c.cellSize, 2.0);
    EXPECT_DOUBLE_EQ(c.droneAltitude, 30.0);
    EXPECT_DOUBLE_EQ(c.avgDroneSpeed, 6.0);
    EXPECT_DOUBLE_EQ(c.minTurnRadius, 12.0);
    EXPECT_DOUBLE_EQ(c.maxFlightTime, 120.0);
    EXPECT_EQ(c.numAgents, 2);
    EXPECT_NEAR(c.sensorTiltAngle, mtl::deg2rad(30.0), 1e-12);
    EXPECT_DOUBLE_EQ(c.sensor.beta, 61.0);
    EXPECT_EQ(c.rngSeed, 7u);
    const double ks = 61.0 / 610.0, s = 200.0 / 5000.0;
    EXPECT_NEAR(c.kernel.tableStep, ref.kernel.tableStep * ks, 1e-12);
    EXPECT_NEAR(c.kernel.trackLen, ref.kernel.trackLen * ks, 1e-9);
    EXPECT_NEAR(c.kernel.edgeInset, ref.kernel.edgeInset * ks, 1e-12);
    EXPECT_NEAR(c.kernel.tailSigma, ref.kernel.tailSigma * ks, 1e-12);
    EXPECT_NEAR(c.kernel.tailWeight, ref.kernel.tailWeight, 1e-12);          // a weight: never scaled
    EXPECT_NEAR(c.team.altitudeStagger, ref.team.altitudeStagger * ks, 1e-12);
    EXPECT_NEAR(c.team.fastGridStep, std::max(2.0, 25.0 * s), 1e-12);        // clamped to the raster
    EXPECT_NEAR(c.team.exploreGridStep, std::max(2.0, 50.0 * s), 1e-12);
    EXPECT_NEAR(c.curve.sampleSpacing, std::max(1.0, 25.0 * s), 1e-12);
    EXPECT_NEAR(c.curve.curvatureKnotSpacing, std::max(4.0 * c.curve.sampleSpacing, 100.0 * s), 1e-12);
    EXPECT_EQ(c.curve.representation, mtl::curve::CurveRepresentation::Curvature);
    ASSERT_EQ(c.curve.endpointModes.size(), 1u);
    EXPECT_EQ(c.curve.endpointModes[0], mtl::curve::EndpointMode::Open);
    EXPECT_EQ(p.curveCells.size(), p.cells.size());
    EXPECT_NEAR(p.curveCells.centers(3, 0), p.cells.centers(3, 0), 1e-12);
    ASSERT_EQ(p.curveStarts.size(), 2u);

    // Keys the scenario sets are used as they are (not scaled).
    const auto q = mtl_search::parseScenario(curveScenario(
        R"({"representation": "bspline", "endpoint_mode": ["return_home", "open"], "altitude_stagger_m": 1.0,
            "fast_grid_step_m": 7.0, "sweep_freq_hz": 0.1, "kernel": {"table_step_m": 3.0, "tail_weight": 0.25},
            "init_strategies": ["greedy"], "max_realloc_trials": 1, "reallocate": false,
            "initial_heading_deg": 90.0, "destinations_ned": [[0.0, 0.0], [10.0, 20.0]]})"));
    const mtl::curve::PlannerParams& d = q.curveParams;
    EXPECT_EQ(d.curve.representation, mtl::curve::CurveRepresentation::BSpline);
    ASSERT_EQ(d.curve.endpointModes.size(), 2u);
    EXPECT_EQ(d.curve.endpointModes[0], mtl::curve::EndpointMode::ReturnHome);
    EXPECT_DOUBLE_EQ(d.team.altitudeStagger, 1.0);
    EXPECT_DOUBLE_EQ(d.team.fastGridStep, 7.0);
    EXPECT_DOUBLE_EQ(d.sweep.freq, 0.1);
    EXPECT_DOUBLE_EQ(d.kernel.tableStep, 3.0);
    EXPECT_DOUBLE_EQ(d.kernel.tailWeight, 0.25);
    EXPECT_NEAR(d.kernel.gridStep, ref.kernel.gridStep * ks, 1e-12);          // absent: still scaled
    ASSERT_EQ(d.team.initStrategies.size(), 1u);
    EXPECT_EQ(d.team.initStrategies[0], mtl::curve::InitStrategy::Greedy);
    EXPECT_EQ(d.team.maxReallocTrials, 1);
    EXPECT_FALSE(d.team.reallocate);
    ASSERT_TRUE(d.curve.initialHeading.has_value());
    EXPECT_NEAR(*d.curve.initialHeading, 0.0, 1e-12);                         // NED 90 deg (East) = mtl 0
    ASSERT_EQ(d.curve.destinations.size(), 2u);
    EXPECT_NEAR(d.curve.destinations[1].x(), 20.0 - (-120.0), 1e-12);        // e - eMin
    EXPECT_NEAR(d.curve.destinations[1].y(), 10.0 - (-50.0), 1e-12);         // n - nMin
}

TEST(CurveScenario, RefusesUnknownKeysAndBadValues) {
    EXPECT_THROW(mtl_search::parseScenario(curveScenario(R"({"sweep_frq_hz": 0.2})")), std::runtime_error);
    EXPECT_THROW(mtl_search::parseScenario(curveScenario(R"({"kernel": {"tablestep_m": 2}})")), std::runtime_error);
    EXPECT_THROW(mtl_search::parseScenario(curveScenario(R"({"representation": "spline"})")), std::runtime_error);
    EXPECT_THROW(mtl_search::parseScenario(curveScenario(R"({"endpoint_mode": "home"})")), std::runtime_error);
    EXPECT_THROW(mtl_search::parseScenario(curveScenario(R"({"init_strategies": ["random"]})")), std::runtime_error);
    EXPECT_THROW(mtl_search::parseScenario(curveScenario("", 120.0, R"({"type": "curvy"})")), std::runtime_error);
    EXPECT_THROW(mtl_search::parseScenario(curveScenario("", 120.0, R"({"type": "curve", "compare": true})")),
                 std::runtime_error);
    std::string s = tinyScenario();
    s.replace(s.find("\"solver\": {}"), 12, R"("solver": {}, "airstack": {"follower": {"gimbal_law": "wobble"}})");
    EXPECT_THROW(mtl_search::parseScenario(s), std::runtime_error);
    std::string t = tinyScenario();
    t.replace(t.find("\"solver\": {}"), 12, R"("solver": {}, "airstack": {"follower": {"gimbal_law": "aim_point"}})");
    EXPECT_EQ(mtl_search::parseScenario(t).gimbalLaw, "aim_point");
}

TEST(CurvePlan, RequiresAFiniteBudget) {
    std::string s = curveScenario();
    s.replace(s.find("\"max_flight_time_s\": 120.000000"), 31, "\"max_flight_time_s\": null");
    const auto p = mtl_search::parseScenario(s);
    EXPECT_TRUE(std::isinf(p.curveParams.maxFlightTime));
    EXPECT_THROW(mtl_search::solveCurve(p), std::runtime_error);
    EXPECT_THROW(mtl_search::solveFlown(p), std::runtime_error);
    // the orienteering planner still plans the same scenario without a budget cap
    mtl_search::SearchProblem q = p;
    q.plannerType = mtl_search::PlannerType::Orienteering;
    q.params.maxFlightTime = 120.0;   // (keep the orienteering run short)
    EXPECT_NO_THROW(mtl_search::solveFlown(q));
}

TEST(CurvePlan, SolveFliesTheCurvePlanner) {
    const auto& f = CurveFixture::get();
    ASSERT_NE(f.result.curve(), nullptr);
    EXPECT_EQ(f.result.orienteering(), nullptr);
    EXPECT_EQ(f.result.type(), mtl_search::PlannerType::Curve);
    EXPECT_EQ(mtl_search::plannerMode(f.result), "curve");
    EXPECT_EQ(f.result.numAgents(), 2u);
    EXPECT_EQ(f.result.numCells(), 8);
    EXPECT_GT(f.result.planningSeconds, 0.0);
    EXPECT_GE(f.result.teamInfoFraction(), 0.0);
    EXPECT_LE(f.result.teamInfoFraction(), 1.0 + 1e-12);
}

TEST(CurvePlan, MapFrameRoundTripAndStartAtHome) {
    const auto& f = CurveFixture::get();
    const auto& r = f.result.curve()->result;
    for (int a = 0; a < 2; ++a) {
        const auto tr = mtl_search::buildAgentTrack(f.result, f.problem, a);
        ASSERT_GT(tr.samples.size(), 100u);
        EXPECT_NEAR(tr.samples.front().x, 0.0, 1e-6);   // starts where the robot spawned
        EXPECT_NEAR(tr.samples.front().y, 0.0, 1e-6);
        for (std::size_t k = 0; k < tr.samples.size(); k += 97) {
            double n = 0, e = 0;
            f.problem.frame.fromMtl(r.trajectories[a].drone(static_cast<mtl::Index>(k), 0),
                                    r.trajectories[a].drone(static_cast<mtl::Index>(k), 1), n, e);
            EXPECT_NEAR(tr.samples[k].x, e - f.problem.agents[a].homeNed.y(), 1e-9);
            EXPECT_NEAR(tr.samples[k].y, n - f.problem.agents[a].homeNed.x(), 1e-9);
            const mtl::Vec2 back = f.problem.frame.toMtl(n, e);
            EXPECT_NEAR(back.x(), r.trajectories[a].drone(static_cast<mtl::Index>(k), 0), 1e-9);
            EXPECT_NEAR(back.y(), r.trajectories[a].drone(static_cast<mtl::Index>(k), 1), 1e-9);
        }
        // the planned altitude (30 m + a * stagger), no altitude_offset_m on top
        const double h = 30.0 + a * f.problem.curveParams.team.altitudeStagger;
        EXPECT_NEAR(tr.altitude, h, 1e-9);
        EXPECT_NEAR(tr.samples[tr.samples.size() / 2].z, h, 1e-9);
        EXPECT_NEAR(tr.samples[tr.samples.size() / 2].bz, 0.0, 1e-12);
        EXPECT_EQ(tr.plannerMode, "curve");
        EXPECT_EQ(tr.plannerType, mtl_search::PlannerType::Curve);
        EXPECT_FALSE(tr.scheduled);
        EXPECT_TRUE(tr.singleAxis);
        EXPECT_DOUBLE_EQ(tr.pitchNudgeMax, 0.0);
        for (const auto& smp : tr.samples) EXPECT_DOUBLE_EQ(smp.speed, 6.0);
    }
}

TEST(CurvePlan, GimbalSignConventionRebuildsEveryBoresight) {
    // Host convention (mtl::planner): phi + RIGHT, crossAngle = roll + phi,
    //   look = p + h tan(theta) u + h tan(roll + phi) / cos(theta) v_right,  theta = tilt - pitch.
    // Rebuild each boresight ground point from the TRACK alone (pose, roll, phi,
    // tilt) and compare it with the planner's `sensor` (= bx, by).
    const auto& f = CurveFixture::get();
    const auto& r = f.result.curve()->result;
    for (int a = 0; a < 2; ++a) {
        const auto tr = mtl_search::buildAgentTrack(f.result, f.problem, a);
        double worst = 0.0, maxAlpha = 0.0, maxRoll = 0.0, worstYaw = 0.0;
        for (std::size_t k = 0; k < tr.samples.size(); ++k) {
            const auto& s = tr.samples[k];
            const double h = s.z;                      // ground plane at map z = 0 (homeUp = 0)
            const double theta = tr.tilt - s.pitch;
            const double ux = std::cos(s.yaw), uy = std::sin(s.yaw);   // along track (ENU)
            const double vx = uy, vy = -ux;                             // right of track
            const double along = h * std::tan(theta);
            const double cross = h * std::tan(s.roll + s.gimbalPhi) / std::cos(theta);
            const double gx = s.x + along * ux + cross * vx, gy = s.y + along * uy + cross * vy;
            worst = std::max(worst, std::hypot(gx - s.bx, gy - s.by));
            const double alpha = r.trajectories[a].gimbalAngle(static_cast<mtl::Index>(k));
            maxAlpha = std::max(maxAlpha, std::abs(alpha));
            maxRoll = std::max(maxRoll, std::abs(s.roll));
            // phi = -(alpha + roll) = -gimbalCmd, and the level-frame look is -alpha (+ left -> + right)
            EXPECT_NEAR(s.gimbalPhi, -r.trajectories[a].gimbalCmd(static_cast<mtl::Index>(k)), 1e-12);
            EXPECT_NEAR(s.roll + s.gimbalPhi, -alpha, 1e-12);
            // the tangent heading equals the flown polyline's central difference (interior)
            if (k > 0 && k + 1 < tr.samples.size()) {
                const auto& p0 = tr.samples[k - 1];
                const auto& p1 = tr.samples[k + 1];
                worstYaw = std::max(worstYaw, std::abs(wrapPi(s.yaw - std::atan2(p1.y - p0.y, p1.x - p0.x))));
            }
        }
        EXPECT_LT(worst, 1e-6) << "agent " << a;
        EXPECT_LT(worstYaw, 1e-6) << "agent " << a;
        EXPECT_GT(maxAlpha, mtl::deg2rad(20.0)) << "the sweep must actually swing for the test to mean anything";
        // The WRONG sign (phi = +gimbalCmd) would miss by metres on the swing.
        double wrong = 0.0;
        for (const auto& s : tr.samples) {
            const double theta = tr.tilt - s.pitch;
            const double cross = s.z * std::tan(s.roll - s.gimbalPhi) / std::cos(theta);
            const double gx = s.x + s.z * std::tan(theta) * std::cos(s.yaw) + cross * std::sin(s.yaw);
            const double gy = s.y + s.z * std::tan(theta) * std::sin(s.yaw) - cross * std::cos(s.yaw);
            wrong = std::max(wrong, std::hypot(gx - s.bx, gy - s.by));
        }
        EXPECT_GT(wrong, 10.0);
        (void)maxRoll;
    }
}

TEST(CurvePlan, LengthIsTheBudgetAndCurvatureRespectsTheTurnRadius) {
    const auto& f = CurveFixture::get();
    for (int a = 0; a < 2; ++a) {
        const auto tr = mtl_search::buildAgentTrack(f.result, f.problem, a);
        const double budget = 120.0 * 6.0;
        EXPECT_NEAR(tr.budget, budget, 1e-9);
        EXPECT_NEAR(tr.totalArc(), budget, 0.002 * budget);
        EXPECT_NEAR(tr.flownLength, budget, 0.002 * budget);
        EXPECT_TRUE(tr.feasible);
        double kmax = 0.0;
        for (std::size_t k = 1; k + 1 < tr.samples.size(); ++k) {
            const auto& p0 = tr.samples[k - 1];
            const auto& p1 = tr.samples[k];
            const auto& p2 = tr.samples[k + 1];
            const double h1 = std::atan2(p1.y - p0.y, p1.x - p0.x), h2 = std::atan2(p2.y - p1.y, p2.x - p1.x);
            const double seg = 0.5 * (std::hypot(p1.x - p0.x, p1.y - p0.y) + std::hypot(p2.x - p1.x, p2.y - p1.y));
            if (seg > 0.0) kmax = std::max(kmax, std::abs(wrapPi(h2 - h1)) / seg);
        }
        EXPECT_LE(kmax, 1.0 / 12.0 + 1e-4) << "agent " << a;
        EXPECT_LE(tr.curve.maxCurvature, 1.0 / 12.0 + 1e-4);
    }
}

TEST(CurvePlan, ReturnHomeAndFixedDestinationEndpoints) {
    // agent 1 returns home, agent 2 ends at a fixed destination (NED [100, 50]).
    const auto p = mtl_search::parseScenario(curveScenario(
        R"({"endpoint_mode": ["return_home", "fixed_dest"], "destinations_ned": [[0.0, 0.0], [100.0, 50.0]]})"));
    const auto r = mtl_search::solveFlown(p);
    const auto t0 = mtl_search::buildAgentTrack(r, p, 0);
    const auto t1 = mtl_search::buildAgentTrack(r, p, 1);
    EXPECT_LT(std::hypot(t0.samples.back().x - t0.samples.front().x, t0.samples.back().y - t0.samples.front().y), 1.0);
    // destination in agent 2's map frame: ENU (e, n) - home ENU
    const double dx = 50.0 - p.agents[1].homeNed.y(), dy = 100.0 - p.agents[1].homeNed.x();
    EXPECT_LT(std::hypot(t1.samples.back().x - dx, t1.samples.back().y - dy), 1.0);
    EXPECT_EQ(t0.curve.endpointMode, "return_home");
    EXPECT_EQ(t1.curve.endpointMode, "fixed_dest");
    EXPECT_LT(t0.curve.endpointError, 1.0);
    EXPECT_LT(t1.curve.endpointError, 1.0);
    EXPECT_NEAR(t0.totalArc(), 720.0, 0.002 * 720.0);
}

TEST(CurvePlan, DeterministicAndJsonOutputsParse) {
    const auto& f = CurveFixture::get();
    const auto again = mtl_search::solveFlown(mtl_search::parseScenario(curveScenario()));
    const std::string j1 = mtl_search::teamPlanJson(f.result, f.problem);
    EXPECT_EQ(j1, mtl_search::teamPlanJson(again, f.problem));
    const auto team = jsonmini::parse(j1);
    EXPECT_EQ(team["schema"].text(""), "mtl.plan/1");
    EXPECT_EQ(team["meta"]["planner_type"].text(""), "curve");
    EXPECT_EQ(team["meta"]["planner_mode"].text(""), "curve");
    EXPECT_EQ(team["generator"]["planner_type"].text(""), "curve");
    EXPECT_EQ(team["meta"]["curve"]["representation"].text(""), "curvature");
    ASSERT_EQ(team["agents"].array().size(), 2u);
    const auto& a0 = team["agents"].array()[0];
    EXPECT_EQ(a0["samples"]["gimbal"].array().size(), a0["samples"]["t"].array().size());
    EXPECT_FALSE(a0["diagnostics"]["scheduled"].flag(true));
    EXPECT_GT(a0["diagnostics"]["sweep_peak_rate_deg_s"].num(0.0), 0.0);
    const auto tr = mtl_search::buildAgentTrack(f.result, f.problem, 1);
    const auto one = jsonmini::parse(mtl_search::agentTrackJson(tr, f.problem));
    EXPECT_EQ(one["planner_mode"].text(""), "curve");
    EXPECT_EQ(one["planner_type"].text(""), "curve");
    EXPECT_EQ(one["samples"]["roll"].array().size(), tr.samples.size());
    EXPECT_NEAR(one["curve"]["min_turn_radius_m"].num(0.0), 12.0, 1e-12);
    EXPECT_GT(one["curve"]["swath_half_width_m"].num(0.0), 0.0);
}

TEST(CurvePlan, ComparisonsAreBothOrienteeringModesNeverTheCurve) {
    const auto p = mtl_search::parseScenario(curveScenario());
    const auto cmp = mtl_search::solveComparisons(p);
    ASSERT_EQ(cmp.size(), 2u);
    EXPECT_EQ(mtl_search::plannerMode(cmp[0]), "plain");
    EXPECT_EQ(mtl_search::plannerMode(cmp[1]), "info_aware");
    EXPECT_EQ(mtl_search::comparisonSuffix(p, cmp[0]), "_alt_plain");
    EXPECT_EQ(mtl_search::comparisonSuffix(p, cmp[1]), "_alt_info_aware");
    // the plain comparison IS the plain orienteering plan of the same scenario
    const auto plain = mtl_search::solveOrienteering(p, false);
    EXPECT_TRUE(cmp[0].orienteering()->trajectories[0].drone.isApprox(plain.trajectories[0].drone));
    // switched off
    const auto off = mtl_search::parseScenario(curveScenario("", 120.0, R"({"type": "curve", "compare_orienteering": false})"));
    EXPECT_TRUE(mtl_search::solveComparisons(off).empty());
    // orienteering flown: the other info_aware mode only, under the old file name
    const auto o = mtl_search::parseScenario(tinyScenario(120.0));
    const auto oc = mtl_search::solveComparisons(o);
    ASSERT_EQ(oc.size(), 1u);
    EXPECT_EQ(mtl_search::plannerMode(oc[0]), "info_aware");
    EXPECT_EQ(mtl_search::comparisonSuffix(o, oc[0]), "_alt");
}

TEST(CurvePlan, OrienteeringThroughTheDispatchIsUnchanged) {
    // planner.type orienteering through solveFlown() == the orienteering planner itself
    const auto p = mtl_search::parseScenario(tinyScenario(120.0));
    const auto viaDispatch = mtl_search::solveFlown(p);
    ASSERT_NE(viaDispatch.orienteering(), nullptr);
    const auto direct = mtl_search::solve(p);
    EXPECT_EQ(mtl_search::teamPlanJson(viaDispatch, p), mtl_search::teamPlanJson(direct, p));
    const auto a = mtl_search::buildAgentTrack(viaDispatch, p, 1);
    const auto b = mtl_search::buildAgentTrack(direct, p, 1);
    EXPECT_EQ(mtl_search::agentTrackJson(a, p), mtl_search::agentTrackJson(b, p));
    // an explicit "planner": {"type": "orienteering"} (+ a curve block) changes nothing either
    const auto q = mtl_search::parseScenario(curveScenario(R"({"sweep_freq_hz": 0.1})", 120.0, R"({"type": "orienteering"})"));
    EXPECT_EQ(mtl_search::teamPlanJson(mtl_search::solveFlown(q), q), mtl_search::teamPlanJson(direct, p));
}
