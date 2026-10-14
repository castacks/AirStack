// =============================================================================
//  test_ipp_baselines_core.cpp — ROS-free tests of the shared core (scenario, Dubins,
//  footprint, belief, gimbal sweep, Planner.h extend, receding horizon, track.json).
//  The scenario is the 200 m unit scenario (gridRes 4); planner lengths are scaled
//  explicitly in unitParams() (the defaults are the 5 km mission ones).
// =============================================================================
#include <gtest/gtest.h>

#include <cmath>
#include <numeric>
#include <string>

#include "ipp_baselines_search_planner/belief.hpp"
#include "ipp_baselines_search_planner/dubins.hpp"
#include "ipp_baselines_search_planner/planner_core.hpp"
#include "ipp_baselines_search_planner/receding.hpp"
#include "ipp_baselines_search_planner/scenario.hpp"
#include "test_common.hpp"

using namespace ipp_baselines_search;
using namespace ipp_test;

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

// ---- Planner::extend_from_start_node_toward_goal_node --------------------------
namespace {
struct ExtendFixture {
    Scenario sc = scenario();
    PlanningGrid g{sc, 4.0, 0.01};
    PlannerSetup su = unitSetup(sc);
    BaselineParams bp = unitParams("greedy");
    PlannerCore core{g, su, bp};
    TreeNode root(double budget) const {
        TreeNode r;
        r.pose = {0.0, 0.0, 0.0};
        r.budgetRemaining = budget;
        return r;
    }
};
}  // namespace

TEST(Extend, StopsBeforeTheBudgetAndClosesWithinOneStep) {
    ExtendFixture f;
    TreeNode n;
    ASSERT_TRUE(f.core.extend(f.root(25.3), 0, {60.0, 0.0, 0.0}, 1e9, &n));
    EXPECT_LT(n.cost, 25.3);                 // stops BEFORE the state that reaches it
    EXPECT_GE(n.cost, 25.3 - 0.5 - 1e-9);    // the last state that fits
    EXPECT_TRUE(n.closed);                   // within one step of the budget
    EXPECT_NEAR(n.budgetRemaining, 25.3 - n.cost, 1e-12);
    EXPECT_EQ(n.depth, 1);
    EXPECT_EQ(n.parent, 0);
    // extend_dist cut: not closed
    TreeNode m;
    ASSERT_TRUE(f.core.extend(f.root(1000.0), 0, {60.0, 0.0, 0.0}, 20.0, &m));
    EXPECT_LT(m.cost, 20.0);
    EXPECT_FALSE(m.closed);
}

TEST(Extend, FirstStepOverTheBudgetGivesAZeroLengthNode) {
    // Planner.h returns a node at the parent's pose (not nullptr) when no state fits
    ExtendFixture f;
    TreeNode n;
    ASSERT_TRUE(f.core.extend(f.root(0.3), 0, {60.0, 10.0, 0.5}, 1e9, &n));
    EXPECT_DOUBLE_EQ(n.edgeLen, 0.0);
    EXPECT_DOUBLE_EQ(n.pose.x, 0.0);
    EXPECT_DOUBLE_EQ(n.pose.y, 0.0);
    EXPECT_TRUE(n.closed);
    EXPECT_FALSE(n.hasEdge);
    // the same pose: a copied state, budget unchanged
    TreeNode c;
    ASSERT_TRUE(f.core.extend(f.root(50.0), 0, {0.0, 0.0, 0.0}, 1e9, &c));
    EXPECT_DOUBLE_EQ(c.edgeLen, 0.0);
    EXPECT_DOUBLE_EQ(c.budgetRemaining, 50.0);
    EXPECT_FALSE(c.closed);
}

TEST(Extend, StraightPartCountsOnlyWhenATurnFollowsIt) {
    ExtendFixture f;
    // pure straight: the heading never changes again, so straight_edge_end_pose is never set
    TreeNode s;
    ASSERT_TRUE(f.core.extend(f.root(1000.0), 0, {80.0, 0.0, 0.0}, 1e9, &s));
    EXPECT_FALSE(s.hasEdge);
    // turn - straight - turn: the edge runs from the start to the end of the straight part
    TreeNode t;
    ASSERT_TRUE(f.core.extend(f.root(1000.0), 0, {80.0, 40.0, kPi / 2}, 1e9, &t));
    ASSERT_TRUE(t.hasEdge);
    EXPECT_LT(t.edgeStartS, t.edgeEndS);
    EXPECT_NEAR(wrapPi(t.edgeStart.yaw - t.edgeEnd.yaw), 0.0, 1e-9);
    const Pose2 a = t.edge.sample(t.edgeStartS), b = t.edge.sample(t.edgeEndS);
    EXPECT_NEAR(a.x, t.edgeStart.x, 1e-9);
    EXPECT_NEAR(b.y, t.edgeEnd.y, 1e-9);
    // the same path cut on its straight part by extend_dist: no turn after it, no edge
    TreeNode c;
    ASSERT_TRUE(f.core.extend(f.root(1000.0), 0, {80.0, 40.0, kPi / 2}, (t.edgeStartS + t.edgeEndS) / 2, &c));
    EXPECT_FALSE(c.hasEdge);
}

TEST(Factory, NamesThePlannersAndRefusesTheRest) {
    const Scenario sc = scenario();
    const PlanningGrid g(sc, 4.0, 0.01);
    const PlannerSetup su = unitSetup(sc);
    for (const char* n : {"greedy", "rect_coverage", "mcts_search"}) {
        EXPECT_STREQ(makePlanner(g, su, unitParams(n))->name(), n);
    }
    EXPECT_THROW(makePlanner(g, su, unitParams("primtree")), std::invalid_argument);
    BaselineParams learned = unitParams("mcts_search");
    learned.mctsRolloutType = "learned";
    EXPECT_THROW(makePlanner(g, su, learned), std::invalid_argument);
}

// ---- receding horizon, every planner ------------------------------------------------
class RecedingAll : public ::testing::TestWithParam<const char*> {};

TEST_P(RecedingAll, KeepsTheCommittedPrefixSnapsToANodeAndCutsAtTheBudget) {
    const Scenario sc = scenario();
    BaselineParams bp = unitParams(GetParam());
    bp.maxIterations = 60;
    const HorizonParams hp = unitHorizon();
    RecedingHorizon rh(sc, 0, Camera(), bp, hp);
    ASSERT_TRUE(rh.start(defaultStartPose(sc, 0, hp)));
    const auto before = rh.track();
    const auto plan = rh.currentPlan();
    const double progress = 30.0;
    rh.replan(progress, rh.plannedLooks(0.0, progress), "period");
    const auto& after = rh.track();
    const ReplanRecord& r = rh.records().back();
    // the commit point is a node of the previous plan (ipp_planners_node.h snaps to a segment end)
    bool atNode = false;
    for (const TreeNode& n : plan) atNode |= std::fabs(n.arc - r.commitArc) < 1e-6 || std::fabs(r.commitArc - rh.budget()) < 1e-6;
    EXPECT_TRUE(atNode || r.commitArc >= before.back().arc - 1e-6);
    EXPECT_GE(r.commitArc, r.commitTargetArc - 1e-6);
    for (std::size_t k = 0; k < after.size() && k < before.size() && before[k].arc <= r.commitArc; ++k) {
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

TEST_P(RecedingAll, TrackJsonCarriesThePlannerAndTheGimbalBlock) {
    const Scenario sc = scenario();
    BaselineParams bp = unitParams(GetParam());
    bp.maxIterations = 40;
    HorizonParams hp = unitHorizon();
    hp.receding = false;
    RecedingHorizon rh(sc, 0, Camera(), bp, hp);
    ASSERT_TRUE(rh.start(defaultStartPose(sc, 0, hp)));
    TrackMeta meta;
    meta.rewardMode = "matched";
    const auto v = ipp_baselines_json::parse(agentTrackJson(sc, 0, rh, meta));
    EXPECT_EQ(v["schema"].str(), "mtl.agent_track/1");
    EXPECT_EQ(v["samples"]["x_map"].array().size(), rh.track().size());
    EXPECT_NEAR(v["samples"]["x_map"].array()[0].number(), 0.0, 1e-3);  // starts at home
    EXPECT_EQ(v["tigris"]["planner"].str(), GetParam());
    EXPECT_TRUE(v["tigris"]["gimbal_locked"].boolean());
    const auto rp = ipp_baselines_json::parse(replansJson(sc, 0, rh, meta));
    EXPECT_EQ(rp["schema"].str(), "ipp_baselines.replans/1");
    EXPECT_EQ(rp["planner"].str(), GetParam());
    EXPECT_EQ(rp["replans"].array().size(), rh.records().size());
}

TEST_P(RecedingAll, SweptTrackKeepsOnePhaseAcrossSplices) {
    const Scenario on = sweepScenarioText(true);
    BaselineParams bp = unitParams(GetParam());
    bp.maxIterations = 60;
    const HorizonParams hp = unitHorizon();
    const Camera cam = cameraFromScenario(on);
    RecedingHorizon rh(on, 0, cam, bp, hp);
    ASSERT_TRUE(rh.start(defaultStartPose(on, 0, hp)));
    rh.replan(30.0, rh.plannedLooks(0.0, 30.0), "period");
    rh.replan(60.0, rh.plannedLooks(30.0, 60.0), "period");
    const auto& tr = rh.track();
    for (std::size_t k = 0; k < tr.size(); ++k) {
        EXPECT_NEAR(tr[k].phi, cam.phiAt(tr[k].t), 1e-12);
        double gx, gy;
        boresightGround(tr[k].x, tr[k].y, tr[k].z, tr[k].yaw, tr[k].phi, cam.tilt, &gx, &gy);
        EXPECT_NEAR(tr[k].bx, gx, 1e-9);
        EXPECT_NEAR(tr[k].by, gy, 1e-9);
    }
    TrackMeta meta;
    meta.setGimbal(gimbalLimits(cam, true));
    EXPECT_FALSE(meta.gimbalLocked);
    const auto v = ipp_baselines_json::parse(agentTrackJson(on, 0, rh, meta));
    EXPECT_TRUE(v["tigris"]["gimbal_actuation"]["enabled"].boolean());
}

INSTANTIATE_TEST_SUITE_P(Planners, RecedingAll, ::testing::Values("greedy", "rect_coverage", "mcts_search"));
