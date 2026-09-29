// =============================================================================
//  test_tigris_core.cpp — ROS-free tests of the TIGRIS core.
// =============================================================================
#include <gtest/gtest.h>

#include <cmath>
#include <numeric>
#include <string>

#include "tigris_search_planner/belief.hpp"
#include "tigris_search_planner/dubins.hpp"
#include "tigris_search_planner/receding.hpp"
#include "tigris_search_planner/scenario.hpp"
#include "tigris_search_planner/tigris.hpp"

using namespace tigris_search;

namespace {

constexpr double kPi = 3.14159265358979323846;

const char* kScenario = R"({
  "schema": "mtl.scenario/1",
  "mission": {"name": "unit", "seed": 7, "area": {"size_m": 200.0, "center_ned": [0.0, 0.0], "belief_res_m": 2.0}},
  "aircraft": {"altitude_m": 30.0, "speed_mps": 6.0, "min_turn_radius_m": 12.0, "dt": 0.1, "dubins_step_m": 0.5},
  "mapping": {"target_cell_size_m": 20.0},
  "sensor": {"fov_deg": 60.0, "tilt_deg": 30.0,
             "detection": {"a": 1.1, "b": 0.1, "c": 61.0, "beta": 61.0, "p_out_of_range": 1e-6,
                           "threshold": 0.9, "dt_ref_s": 0.1}},
  "team": {"max_flight_time_s": 30.0, "max_flight_distance_m": null,
           "agents": [{"name": "robot_1", "start_ned": [-90.0, -90.0], "home_ned": [-90.0, -90.0]}]},
  "cells": {"centers": [[0.0, 40.0], [-40.0, -40.0]], "mass": [0.1, 0.05], "total_map_mass": 1.0},
  "airstack": {"belief": {"bumps": [{"n": 0.0, "e": 40.0, "sigma_n": 20.0, "sigma_e": 20.0, "amplitude": 0.4},
                                    {"n": -40.0, "e": -40.0, "sigma_n": 15.0, "sigma_e": 25.0, "amplitude": 0.4}],
                          "belief_cap": 0.85, "base_uncertainty": 0.0}}
})";

Scenario scenario() { return parseScenario(kScenario); }

}  // namespace

TEST(Scenario, ParsesFramesBudgetAndPrior) {
    const Scenario sc = scenario();
    EXPECT_DOUBLE_EQ(sc.xMin, -100.0);
    EXPECT_DOUBLE_EQ(sc.yMax, 100.0);
    EXPECT_NEAR(sc.budgetDistance(), 180.0, 1e-9);  // 30 s * 6 m/s, distance unbounded
    ASSERT_EQ(sc.agents.size(), 1u);
    EXPECT_DOUBLE_EQ(sc.agents[0].startE, -90.0);
    EXPECT_EQ(sc.prior.nx(), 101);
    EXPECT_NEAR(std::accumulate(sc.prior.norm.begin(), sc.prior.norm.end(), 0.0), 1.0, 1e-9);
    EXPECT_NEAR(sc.cellX[0], 40.0, 1e-12);  // [n, e] -> world x = e
}

TEST(Scenario, RejectsWrongSchema) {
    EXPECT_THROW(parseScenario(R"({"schema": "nope"})"), std::runtime_error);
}

TEST(Dubins, ReachesTheGoalPose) {
    const Pose2 a{0.0, 0.0, 0.3};
    const double rho = 12.0;
    const Pose2 goals[] = {{50.0, 10.0, 1.0}, {-20.0, 5.0, -2.5}, {3.0, 4.0, kPi}, {0.0, 30.0, 0.3}};
    for (const Pose2& b : goals) {
        DubinsPath p;
        ASSERT_TRUE(dubinsShortest(a, b, rho, &p));
        const Pose2 e = p.sample(p.length());
        EXPECT_NEAR(e.x, b.x, 1e-6);
        EXPECT_NEAR(e.y, b.y, 1e-6);
        EXPECT_NEAR(wrapPi(e.yaw - b.yaw), 0.0, 1e-6);
        EXPECT_GE(p.length(), std::hypot(b.x - a.x, b.y - a.y) - 1e-9);
    }
}

TEST(Dubins, StraightLineIsExact) {
    DubinsPath p;
    ASSERT_TRUE(dubinsShortest({0, 0, 0}, {40, 0, 0}, 12.0, &p));
    EXPECT_NEAR(p.length(), 40.0, 1e-9);
    const Pose2 m = p.sample(10.0);
    EXPECT_NEAR(m.x, 10.0, 1e-9);
    EXPECT_NEAR(m.y, 0.0, 1e-9);
}

TEST(Sensor, BodyFixedLookGeometry) {
    DetectionModel det;
    Camera cam;  // 60 deg cone, 30 deg forward tilt
    const Look l = lookFromPose(0.0, 0.0, 30.0, kPi / 2.0, cam, det, 1.0);
    ASSERT_TRUE(l.valid);
    EXPECT_NEAR(l.gx, 0.0, 1e-9);
    EXPECT_NEAR(l.gy, 30.0 * std::tan(kPi / 6.0), 1e-9);  // 17.3 m ahead (north)
    EXPECT_NEAR(l.radius, 30.0 / std::cos(kPi / 6.0) * std::tan(kPi / 6.0), 1e-9);
    // a gimbal at pitch 60 deg down, yaw 90 deg sees the same disc
    const Look g = lookFromGimbal(0.0, 0.0, 30.0, kPi / 3.0, kPi / 2.0, cam.fov, det, 1.0);
    EXPECT_NEAR(g.gy, l.gy, 1e-9);
    EXPECT_NEAR(g.radius, l.radius, 1e-9);
    // beyond beta nothing is seen
    EXPECT_FALSE(lookFromPose(0.0, 0.0, 80.0, 0.0, cam, det, 1.0).valid);
}

TEST(Belief, GridConservesMassAndPresenceIsFloored) {
    const Scenario sc = scenario();
    const PlanningGrid g(sc, 4.0, 0.01);
    EXPECT_EQ(g.nx, 50);
    EXPECT_NEAR(std::accumulate(g.mass.begin(), g.mass.end(), 0.0), 1.0, 1e-9);
    for (const double p : g.presence0) {
        EXPECT_GE(p, 0.01);
        EXPECT_LE(p, 0.85 + 1e-12);
    }
}

TEST(Belief, MatchedRewardIsResidualDrop) {
    const Scenario sc = scenario();
    const PlanningGrid g(sc, 4.0, 0.01);
    RewardParams rp;
    RewardEvaluator ev(g, sc.det, rp);
    BeliefState b = BeliefState::fromGrid(g);
    const double before = b.residualMass();
    Camera cam;
    std::vector<Look> looks;
    for (int k = 0; k < 20; ++k) looks.push_back(lookFromPose(40.0, -20.0 + k, 30.0, kPi / 2, cam, sc.det, 1.0));
    ev.begin(b);
    ev.pass(looks, true, true);
    const PathReward r = ev.reward();
    ev.commit(b);
    EXPECT_GT(r.matched, 0.0);
    EXPECT_NEAR(before - b.residualMass(), r.matched, 1e-12);
    EXPECT_GT(r.original, 0.0);
    for (std::size_t k = 0; k < g.size(); ++k) EXPECT_LE(b.residual[k], g.mass[k] + 1e-15);
}

TEST(Belief, OriginalUpdateFollowsTigrisBranches) {
    DetectionModel det;
    RewardParams rp;
    double q;
    // p <= 0.5: assumed miss -> belief drops, reward = entropy drop * Rf
    const double r1 = originalCellUpdate(0.3, 20.0, det, rp, &q);
    EXPECT_LT(q, 0.3);
    EXPECT_GT(r1, 0.0);
    // p > 0.5: assumed hit -> belief rises, weighted by Rs
    originalCellUpdate(0.7, 20.0, det, rp, &q);
    EXPECT_GT(q, 0.7);
    // beyond beta tpr = fpr = 0.5: no information
    EXPECT_NEAR(originalCellUpdate(0.3, 100.0, det, rp, &q), 0.0, 1e-12);
    EXPECT_NEAR(q, 0.3, 1e-12);
}

TEST(Tigris, RespectsTheBudgetAndIsDeterministic) {
    const Scenario sc = scenario();
    const PlanningGrid g(sc, 4.0, 0.01);
    PlannerSetup su;
    su.altitude = sc.altitude;
    su.xMin = sc.xMin; su.xMax = sc.xMax; su.yMin = sc.yMin; su.yMax = sc.yMax;
    TigrisParams tp;
    tp.maxIterations = 400;
    const BeliefState b = BeliefState::fromGrid(g);
    for (const RewardMode mode : {RewardMode::ORIGINAL, RewardMode::MATCHED}) {
        tp.reward.mode = mode;
        TigrisPlanner p1(g, su, tp), p2(g, su, tp);
        const PlanResult a = p1.plan({-90, -90, kPi / 4}, 150.0, 60.0, b);
        const PlanResult c = p2.plan({-90, -90, kPi / 4}, 150.0, 60.0, b);
        ASSERT_TRUE(a.improved());
        EXPECT_LE(a.cost, 150.0 + 1e-9);
        EXPECT_EQ(a.path.size(), c.path.size());
        EXPECT_NEAR(a.reward.of(mode), c.reward.of(mode), 1e-12);
        EXPECT_GT(a.reward.of(mode), 0.0);
        // the path is connected: each node is its edge's end point
        for (std::size_t i = 1; i < a.path.size(); ++i) {
            const Pose2 e = a.path[i].edge.sample(a.path[i].edgeLen);
            EXPECT_NEAR(e.x, a.path[i].pose.x, 1e-9);
            EXPECT_NEAR(a.path[i].edge.q0.x, a.path[i - 1].pose.x, 1e-9);
            EXPECT_NEAR(a.path[i].edge.q0.y, a.path[i - 1].pose.y, 1e-9);
        }
    }
}

TEST(Receding, KeepsTheCommittedPrefixAndTheBudget) {
    const Scenario sc = scenario();
    TigrisParams tp;
    tp.maxIterations = 300;
    HorizonParams hp;
    hp.initialPlanningTime = 60.0;
    hp.replanPlanningTime = 60.0;
    Camera cam;
    RecedingHorizon rh(sc, 0, cam, tp, hp);
    ASSERT_TRUE(rh.start(defaultStartPose(sc, 0, hp)));
    const auto before = rh.track();
    const double progress = 30.0;
    rh.replan(progress, rh.plannedLooks(0.0, progress), "period");
    const auto& after = rh.track();
    const double commit = rh.records().back().commitArc;
    for (std::size_t k = 0; k < after.size() && before[k].arc <= commit; ++k) {
        EXPECT_DOUBLE_EQ(after[k].x, before[k].x);
        EXPECT_DOUBLE_EQ(after[k].arc, before[k].arc);
    }
    EXPECT_LE(rh.totalArc(), rh.budget() + 1e-6);
    for (std::size_t k = 1; k < after.size(); ++k) {
        EXPECT_GT(after[k].arc, after[k - 1].arc);
        EXPECT_NEAR(std::hypot(after[k].x - after[k - 1].x, after[k].y - after[k - 1].y),
                    after[k].arc - after[k - 1].arc, 0.05);  // no jumps at the splice
    }
    EXPECT_LT(rh.belief().residualMass(), 1.0);  // the flown looks were folded in
}

TEST(Receding, TrackJsonHasTheMtlLayout) {
    const Scenario sc = scenario();
    TigrisParams tp;
    tp.maxIterations = 200;
    HorizonParams hp;
    hp.receding = false;
    hp.initialPlanningTime = 60.0;
    RecedingHorizon rh(sc, 0, Camera(), tp, hp);
    ASSERT_TRUE(rh.start(defaultStartPose(sc, 0, hp)));
    TrackMeta meta;
    meta.rewardMode = "original";
    const auto v = tigris_json::parse(agentTrackJson(sc, 0, rh, meta));
    EXPECT_EQ(v["schema"].str(), "mtl.agent_track/1");
    EXPECT_EQ(v["samples"]["x_map"].array().size(), rh.track().size());
    EXPECT_NEAR(v["samples"]["x_map"].array()[0].number(), 0.0, 1e-3);  // starts at home
    EXPECT_EQ(v["tigris"]["planner"].str(), "tigris");
}

// ---- gimbal actuation (mission.yaml gimbal_actuation) ------------------------
namespace {

Scenario sweepScenario(bool enabled) {
    std::string text = kScenario;
    const std::string key = "\"airstack\": {";
    const auto at = text.find(key);
    text.insert(at + key.size(), std::string("\"gimbal_actuation\": {\"enabled\": ") + (enabled ? "true" : "false") +
                                     ", \"sweep_rate_deg_s\": 30.0, \"sweep_amplitude_deg\": 45.0}, ");
    return parseScenario(text);
}

}  // namespace

TEST(GimbalSweep, ParsesTheMissionBlock) {
    const Scenario off = scenario();
    EXPECT_FALSE(off.gimbal.enabled);
    EXPECT_FALSE(cameraFromScenario(off).sweep);
    const Scenario on = sweepScenario(true);
    EXPECT_TRUE(on.gimbal.enabled);
    EXPECT_NEAR(on.gimbal.rate, 30.0 * kPi / 180.0, 1e-12);
    EXPECT_NEAR(on.gimbal.amplitude, 45.0 * kPi / 180.0, 1e-12);
    const Camera cam = cameraFromScenario(on);
    EXPECT_TRUE(cam.sweep);
    EXPECT_FALSE(cameraFromScenario(sweepScenario(false)).sweep);
}

TEST(GimbalSweep, TriangleWaveAtConstantRate) {
    Camera cam;
    cam.sweep = true;
    cam.sweepRate = 30.0 * kPi / 180.0;
    cam.sweepAmplitude = 45.0 * kPi / 180.0;
    const double A = cam.sweepAmplitude, period = 4.0 * A / cam.sweepRate;  // 6 s
    EXPECT_NEAR(period, 6.0, 1e-12);
    EXPECT_NEAR(cam.phiAt(0.0), 0.0, 1e-12);
    EXPECT_NEAR(cam.phiAt(1.5), A, 1e-12);        // right end
    EXPECT_NEAR(cam.phiAt(3.0), 0.0, 1e-12);
    EXPECT_NEAR(cam.phiAt(4.5), -A, 1e-12);       // left end
    EXPECT_NEAR(cam.phiAt(6.0 + 0.7), cam.phiAt(0.7), 1e-12);
    for (double t = 0.0; t < 20.0; t += 0.01) {
        const double v = std::fabs(cam.phiAt(t + 0.01) - cam.phiAt(t)) / 0.01;
        EXPECT_LE(v, cam.sweepRate + 1e-9);
        EXPECT_LE(std::fabs(cam.phiAt(t)), A + 1e-12);
    }
    Camera fixed;
    EXPECT_EQ(fixed.phiAt(3.3), 0.0);
}

TEST(GimbalSweep, SweptLookGeometry) {
    DetectionModel det;
    Camera cam;
    const double h = 30.0, phi = 30.0 * kPi / 180.0;
    // heading north: right = east
    const Look l = lookFromPose(0.0, 0.0, h, kPi / 2.0, cam, det, 1.0, phi);
    ASSERT_TRUE(l.valid);
    EXPECT_NEAR(l.gx, h * std::tan(phi), 1e-9);                                   // right of track
    EXPECT_NEAR(l.gy, h * std::tan(kPi / 6.0) / std::cos(phi), 1e-9);             // ahead
    const double slant = h / (std::cos(kPi / 6.0) * std::cos(phi));
    EXPECT_NEAR(std::sqrt(l.gx * l.gx + l.gy * l.gy + h * h), slant, 1e-9);        // on the constraint plane
    EXPECT_NEAR(l.radius, slant * std::tan(kPi / 6.0), 1e-9);
    // phi = 0 is the body-fixed look, bit for bit
    const Look a = lookFromPose(3.0, -4.0, h, 0.4, cam, det, 1.0);
    const Look b = lookFromPose(3.0, -4.0, h, 0.4, cam, det, 1.0, 0.0);
    EXPECT_EQ(a.gx, b.gx);
    EXPECT_EQ(a.gy, b.gy);
    EXPECT_EQ(a.radius, b.radius);
    // swung out past beta the look sees nothing (61 m at 30 m altitude: |phi| > 55.4 deg)
    EXPECT_FALSE(lookFromPose(0.0, 0.0, h, 0.0, cam, det, 1.0, 60.0 * kPi / 180.0).valid);
}

TEST(GimbalSweep, KinematicsMatchTheFollowerSolution) {
    Camera cam;
    cam.sweep = true;
    cam.sweepRate = 30.0 * kPi / 180.0;
    cam.sweepAmplitude = 45.0 * kPi / 180.0;
    const SweepKinematics k = sweepKinematics(cam);
    // gimbal_math.single_axis_command at phi = 45 deg, tilt 30 deg: roll -63.43, pitch 37.76 deg
    EXPECT_NEAR(k.maxAbsRoll * 180.0 / kPi, 63.43, 0.01);
    EXPECT_NEAR(k.minPitch * 180.0 / kPi, 37.76, 0.01);
    // near phi = 0 the roll moves 1 / sin(tilt) = 2x the cross-track rate
    EXPECT_NEAR(k.peakAxisRate, 2.0 * cam.sweepRate, 0.01 * cam.sweepRate);
    EXPECT_NEAR(k.maxSlantPerHeight, 1.0 / (std::cos(kPi / 6.0) * std::cos(cam.sweepAmplitude)), 1e-12);
}

TEST(GimbalSweep, TrackCarriesTheSweepAndItsResidualBelief) {
    const Scenario on = sweepScenario(true);
    TigrisParams tp;
    tp.maxIterations = 300;
    HorizonParams hp;
    hp.initialPlanningTime = 60.0;
    hp.replanPlanningTime = 60.0;
    const Camera cam = cameraFromScenario(on);
    RecedingHorizon rh(on, 0, cam, tp, hp);
    ASSERT_TRUE(rh.start(defaultStartPose(on, 0, hp)));
    rh.replan(30.0, rh.plannedLooks(0.0, 30.0), "period");
    const auto& tr = rh.track();
    double phiMax = 0.0;
    for (std::size_t k = 0; k < tr.size(); ++k) {
        const TrackSample& s = tr[k];
        EXPECT_NEAR(s.phi, cam.phiAt(s.t), 1e-12);  // one phase across the splice
        phiMax = std::max(phiMax, std::fabs(s.phi));
        double gx, gy;
        boresightGround(s.x, s.y, s.z, s.yaw, s.phi, cam.tilt, &gx, &gy);
        EXPECT_NEAR(s.bx, gx, 1e-9);
        EXPECT_NEAR(s.by, gy, 1e-9);
        if (k > 0) {
            EXPECT_LE(std::fabs(s.phi - tr[k - 1].phi), cam.sweepRate * (s.t - tr[k - 1].t) + 1e-9);
        }
    }
    EXPECT_NEAR(phiMax, cam.sweepAmplitude, 1e-6);
    // the residual belief is recalculated with the swept looks
    const std::vector<Look> looks = rh.plannedLooks(0.0, rh.totalArc());
    const PlanningGrid& g = rh.grid();
    RewardEvaluator ev(g, on.det, tp.reward);
    BeliefState swept = BeliefState::fromGrid(g), fixed = BeliefState::fromGrid(g);
    ev.begin(swept);
    ev.pass(looks, false, true);
    ev.commit(swept);
    std::vector<Look> fixedLooks;
    for (std::size_t k = 0; k < tr.size(); ++k) {
        const double dt = k > 0 ? tr[k].t - tr[k - 1].t : on.dt;
        fixedLooks.push_back(lookFromPose(tr[k].x, tr[k].y, tr[k].z, tr[k].yaw, cam, on.det, dt / on.det.dtRef));
    }
    ev.begin(fixed);
    ev.pass(fixedLooks, false, true);
    ev.commit(fixed);
    EXPECT_NE(swept.residualMass(), fixed.residualMass());
    TrackMeta meta;
    meta.setGimbal(gimbalLimits(cam, true));
    EXPECT_FALSE(meta.gimbalLocked);
    EXPECT_GE(meta.gimbalMaxRad, cam.sweepAmplitude);
    EXPECT_LT(meta.pitchNudgeMaxRad, 1e-5);
    const auto v = tigris_json::parse(agentTrackJson(on, 0, rh, meta));
    EXPECT_TRUE(v["scheduled"].boolean());
    EXPECT_TRUE(v["tigris"]["gimbal_actuation"]["enabled"].boolean());
    EXPECT_EQ(v["samples"]["gimbal_phi"].array().size(), tr.size());
}

TEST(GimbalSweep, SweepAwareTreeRewardsAndOffIsUnchanged) {
    const Scenario sc = scenario();
    const PlanningGrid g(sc, 4.0, 0.01);
    PlannerSetup su;
    su.altitude = sc.altitude;
    su.xMin = sc.xMin; su.xMax = sc.xMax; su.yMin = sc.yMin; su.yMax = sc.yMax;
    TigrisParams tp;
    tp.maxIterations = 300;
    const BeliefState b = BeliefState::fromGrid(g);
    for (const RewardMode mode : {RewardMode::ORIGINAL, RewardMode::MATCHED}) {
        tp.reward.mode = mode;
        TigrisPlanner fixed(g, su, tp);
        const PlanResult a = fixed.plan({-90, -90, kPi / 4}, 150.0, 60.0, b);
        // MATCHED: an actuated camera with a 0 deg sweep is the body-fixed camera (ORIGINAL
        // switches to the swept-swath edge model whenever the sweep is on, so it is not compared)
        PlannerSetup zero = su;
        zero.camera.sweep = true;
        zero.camera.sweepRate = 0.5;
        zero.camera.sweepAmplitude = 0.0;
        TigrisPlanner z(g, zero, tp);
        const PlanResult c = z.plan({-90, -90, kPi / 4}, 150.0, 60.0, b);
        if (mode == RewardMode::MATCHED) {
            EXPECT_EQ(a.path.size(), c.path.size());
            EXPECT_NEAR(a.reward.matched, c.reward.matched, 1e-12);
        }
        // a real sweep plans a feasible path with a positive reward, and its reward depends on the phase
        PlannerSetup sw = su;
        sw.camera.sweep = true;
        sw.camera.sweepRate = 30.0 * kPi / 180.0;
        sw.camera.sweepAmplitude = 45.0 * kPi / 180.0;
        TigrisPlanner s(g, sw, tp);
        const PlanResult r = s.plan({-90, -90, kPi / 4}, 150.0, 60.0, b);
        ASSERT_TRUE(r.improved());
        EXPECT_LE(r.cost, 150.0 + 1e-9);
        EXPECT_GT(r.reward.of(mode), 0.0);
        const PathReward p0 = s.scorePath(r.path, b, 0.0), p1 = s.scorePath(r.path, b, 1.5);
        EXPECT_NEAR(p0.of(mode), r.reward.of(mode), 1e-12);
        EXPECT_NE(p0.of(mode), p1.of(mode));
        for (std::size_t i = 1; i < r.path.size(); ++i) {
            EXPECT_NEAR(r.path[i].arc, r.path[i - 1].arc + r.path[i].edgeLen, 1e-9);
        }
    }
}
