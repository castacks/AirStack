// =============================================================================
//  test_ipp_baselines_planners.cpp — per-planner tests: greedy, rect_coverage,
//  mcts_search on the 200 m unit scenario (lengths scaled in unitParams()).
// =============================================================================
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <string>

#include "ipp_baselines_search_planner/greedy.hpp"
#include "ipp_baselines_search_planner/mcts_search.hpp"
#include "ipp_baselines_search_planner/rect_coverage.hpp"
#include "test_common.hpp"

using namespace ipp_baselines_search;
using namespace ipp_test;

namespace {

struct Fixture {
    Scenario sc = scenario();
    PlanningGrid g{sc, 4.0, 0.01};
    PlannerSetup su = unitSetup(sc);
    BeliefState b = BeliefState::fromGrid(g);
};

void expectConnected(const std::vector<TreeNode>& path) {
    for (std::size_t i = 1; i < path.size(); ++i) {
        const Pose2 e = path[i].edge.sample(path[i].edgeLen);
        EXPECT_NEAR(e.x, path[i].pose.x, 1e-6);
        EXPECT_NEAR(e.y, path[i].pose.y, 1e-6);
        EXPECT_NEAR(path[i].edge.q0.x, path[i - 1].pose.x, 1e-6);
        EXPECT_NEAR(path[i].edge.q0.y, path[i - 1].pose.y, 1e-6);
        EXPECT_NEAR(path[i].arc, path[i - 1].arc + path[i].edgeLen, 1e-9);
    }
}

}  // namespace

// ================================ greedy ============================================
TEST(Greedy, BudgetDeterministicConnectedAndPositive) {
    Fixture f;
    for (const RewardMode mode : {RewardMode::ORIGINAL, RewardMode::MATCHED}) {
        BaselineParams bp = unitParams("greedy");
        bp.reward.mode = mode;
        GreedyPlanner p1(f.g, f.su, bp), p2(f.g, f.su, bp);
        const PlanResult a = p1.plan({-90, -90, kPi / 4}, 150.0, 60.0, f.b, 0, 0.0);
        const PlanResult c = p2.plan({-90, -90, kPi / 4}, 150.0, 60.0, f.b, 0, 0.0);
        ASSERT_TRUE(a.improved());
        ASSERT_EQ(a.path.size(), c.path.size());
        for (std::size_t i = 0; i < a.path.size(); ++i) EXPECT_EQ(a.path[i].pose.x, c.path[i].pose.x);
        EXPECT_GT(a.reward.of(mode), 0.0);
        EXPECT_LE(a.extra.at("counted_length_m"), 150.0);  // what greedy counts stays under the budget
        expectConnected(a.path);
    }
}

TEST(Greedy, PicksTheMaxWeightCellPerUnitPathLength) {
    Fixture f;
    for (const RewardMode mode : {RewardMode::ORIGINAL, RewardMode::MATCHED}) {
        BaselineParams bp = unitParams("greedy");
        bp.reward.mode = mode;
        bp.maxIterations = 1;
        GreedyPlanner p(f.g, f.su, bp);
        const Pose2 start{-90, -90, kPi / 4};
        // the working map the first pick sees: belief after the root's footprint (computed again here)
        GreedyPlanner q(f.g, f.su, bp);
        q.plan(start, 150.0, 0.0, f.b, 0, 0.0);  // a zero time limit: replan_setup only
        const BeliefState work = q.workingMap();
        p.plan(start, 150.0, 60.0, f.b, 0, 0.0);
        const double rNom = p.nominalRange();
        const double pNom = f.sc.det.prob(rNom);
        double best = 0.0;
        int bestK = -1;
        for (int j = 0; j < f.g.nx; ++j) {
            for (int i = 0; i < f.g.ny; ++i) {
                const int k = i * f.g.nx + j;
                double q2, w;
                if (mode == RewardMode::ORIGINAL) w = originalCellUpdate(work.presence[k], rNom, f.sc.det, bp.reward, &q2);
                else w = work.residual[k] * pNom;
                const Pose2 vp = p.findViewingPoint(start, f.g.cx(j), f.g.cy(i));
                if (vp.x == start.x || vp.y == start.y) continue;
                DubinsPath dp;
                dubinsShortest(start, vp, f.su.turnRadius, &dp);
                if (w / dp.length() > best) {
                    best = w / dp.length();
                    bestK = k;
                }
            }
        }
        ASSERT_GE(bestK, 0);
        EXPECT_EQ(p.lastCell(), bestK);
        EXPECT_NEAR(p.lastScore(), best, 1e-15);
    }
}

TEST(Greedy, ChainUpdateIsFootprintThenEdge) {
    // search_information_gain (the full walk greedy uses) scores each node's footprint BEFORE
    // its edge. The two Bayes updates of a cell commute while they take the same branch of
    // calc_reward_and_belief, so the order only shows when a belief crosses 0.5; the working
    // map must still be exactly the footprint-then-edge walk
    Fixture f;
    BaselineParams bp = unitParams("greedy");
    bp.reward.mode = RewardMode::ORIGINAL;
    bp.maxIterations = 3;
    GreedyPlanner p(f.g, f.su, bp);
    const PlanResult r = p.plan({-90, -90, kPi / 4}, 400.0, 60.0, f.b, 0, 0.0);
    ASSERT_GE(r.path.size(), 3u);
    RewardEvaluator& ev = p.evaluator();
    BeliefState nodeFirst = f.b;
    ev.begin(f.b);
    for (const TreeNode& n : r.path) p.originalNodeGain(ev, n, 0.0, false, true);
    ev.commit(nodeFirst);
    int changed = 0;
    for (std::size_t k = 0; k < f.g.size(); ++k) {
        EXPECT_EQ(p.workingMap().presence[k], nodeFirst.presence[k]);  // bit for bit
        changed += p.workingMap().presence[k] != f.b.presence[k];
    }
    EXPECT_GT(changed, 0);
    EXPECT_EQ(p.workingMap().residual, f.b.residual);  // ORIGINAL mode leaves the matched belief alone
}

TEST(Greedy, LastSegmentOvershootsAndTheTrackIsCutAtTheBudget) {
    const Scenario sc = scenario();
    BaselineParams bp = unitParams("greedy");
    HorizonParams hp = unitHorizon();
    hp.receding = false;
    RecedingHorizon rh(sc, 0, Camera(), bp, hp);
    ASSERT_TRUE(rh.start(defaultStartPose(sc, 0, hp)));
    EXPECT_GT(rh.currentPlan().back().arc, rh.budget());        // the whole last segment is kept
    EXPECT_GT(rh.records().front().cutAtBudget, 0.0);
    EXPECT_NEAR(rh.totalArc(), rh.budget(), 1e-6);                 // the published track ends at the budget
}

// ============================== rect_coverage =======================================
TEST(RectCoverage, LawnmowerCoversTheAreaLaneByLane) {
    Fixture f;
    BaselineParams bp = unitParams("rect_coverage");
    RectCoveragePlanner p(f.g, f.su, bp);
    const PlanResult r = p.plan({-90, -90, kPi / 4}, 1e5, 60.0, f.b, 0, 0.0);
    const double w = p.sensorWidth(), xd = p.xDelta();
    // the cone's diameter at the aim point (tilt - vg fov / 2 from nadir)
    const double aim = f.su.camera.tilt - bp.viewPointGoal * f.su.camera.fov / 2.0;
    EXPECT_NEAR(xd, f.su.altitude * std::tan(aim), 1e-12);
    EXPECT_NEAR(w, 2.0 * std::tan(f.su.camera.fov / 2.0) * f.su.altitude / std::cos(aim), 1e-9);
    const auto& wp = p.wayPoints();
    const int lanes = static_cast<int>(std::floor(200.0 / w));  // a square: floor(first_edge / width)
    ASSERT_EQ(static_cast<int>(wp.size()), 1 + 2 * lanes);
    for (int k = 0; k < lanes; ++k) {
        const auto& a = wp[1 + 2 * k];
        const auto& b = wp[2 + 2 * k];
        const double laneX = f.sc.xMin + w / 2 + k * w;  // lanes stepped along +x by the width
        EXPECT_NEAR(a[0], laneX, 1e-9);
        EXPECT_NEAR(b[0], laneX, 1e-9);
        if (k % 2 == 0) {  // forward: north, the aircraft x_delta behind the aim line
            EXPECT_NEAR(a[1], f.sc.yMin - xd, 1e-9);
            EXPECT_NEAR(b[1], f.sc.yMax - xd, 1e-9);
            EXPECT_NEAR(a[3], kPi / 2, 1e-9);
        } else {           // backward: south
            EXPECT_NEAR(a[1], f.sc.yMax + xd, 1e-9);
            EXPECT_NEAR(b[1], f.sc.yMin + xd, 1e-9);
            EXPECT_NEAR(a[3], 3 * kPi / 2, 1e-9);
        }
    }
    // every x in [xMin, xMin + lanes w] is within half a width of a lane: the lanes tile the area
    for (double x = f.sc.xMin; x <= f.sc.xMin + lanes * w; x += 0.37) {
        double dmin = 1e9;
        for (int k = 0; k < lanes; ++k) dmin = std::min(dmin, std::fabs(x - (f.sc.xMin + w / 2 + k * w)));
        EXPECT_LE(dmin, w / 2 + 1e-9);
    }
    EXPECT_EQ(static_cast<int>(r.path.size()), 1 + 2 * lanes);
    expectConnected(r.path);
    EXPECT_GT(r.reward.matched, 0.0);
    EXPECT_GT(r.reward.original, 0.0);
}

TEST(RectCoverage, TurnRadiusCheckRefusesWhenEnforced) {
    Fixture f;
    BaselineParams bp = unitParams("rect_coverage");
    bp.coverageEnforceTurnCheck = true;  // 2 R = 24 m < 35 m width: RectangleCoverage throws
    RectCoveragePlanner p(f.g, f.su, bp);
    EXPECT_THROW(p.plan({-90, -90, kPi / 4}, 1e5, 60.0, f.b, 0, 0.0), std::invalid_argument);
}

TEST(RectCoverage, EverySolveRestartsAtTheFirstLaneAndTheTrackIsCut) {
    const Scenario sc = scenario();
    BaselineParams bp = unitParams("rect_coverage");
    const HorizonParams hp = unitHorizon();
    RecedingHorizon rh(sc, 0, Camera(), bp, hp);
    ASSERT_TRUE(rh.start(defaultStartPose(sc, 0, hp)));
    const TreeNode entry = rh.currentPlan()[1];
    EXPECT_GT(rh.currentPlan().back().arc, rh.budget());  // the lawnmower ignores the budget
    EXPECT_NEAR(rh.totalArc(), rh.budget(), 1e-6);
    rh.replan(30.0, rh.plannedLooks(0.0, 30.0), "period");
    // the new plan from the committed node heads for the lane-1 entry again
    bool hasEntry = false;
    for (const TreeNode& n : rh.currentPlan()) hasEntry |= std::hypot(n.pose.x - entry.pose.x, n.pose.y - entry.pose.y) < 1e-6;
    EXPECT_TRUE(hasEntry);
    EXPECT_LE(rh.totalArc(), rh.budget() + 1e-6);
}

// =============================== mcts_search ========================================
namespace {
/// search_info of a chain re-walked from the root (the root itself is never scored by MCTS).
double rewalk(MctsSearchPlanner& p, const std::vector<MctsNode>& nodes, int leaf, const BeliefState& b,
              RewardMode mode, double t0) {
    std::vector<int> chain;
    for (int q = leaf; q > 0; q = nodes[static_cast<std::size_t>(q)].t.parent) chain.push_back(q);
    RewardEvaluator& ev = p.evaluator();
    ev.begin(b);
    for (auto it = chain.rbegin(); it != chain.rend(); ++it) {
        const TreeNode& n = nodes[static_cast<std::size_t>(*it)].t;
        if (mode == RewardMode::ORIGINAL) p.originalNodeGain(ev, n, t0, true, true);
        else p.matchedNodeGain(ev, n, t0);
    }
    return ev.reward().of(mode);
}
}  // namespace

TEST(Mcts, BudgetDeterministicConnectedAndPositive) {
    Fixture f;
    for (const char* rollout : {"primitive", "informed"}) {
        for (const RewardMode mode : {RewardMode::ORIGINAL, RewardMode::MATCHED}) {
            BaselineParams bp = unitParams("mcts_search");
            bp.reward.mode = mode;
            bp.mctsRolloutType = rollout;
            bp.maxIterations = 80;
            MctsSearchPlanner p1(f.g, f.su, bp), p2(f.g, f.su, bp);
            const PlanResult a = p1.plan({-90, -90, kPi / 4}, 150.0, 60.0, f.b, 3, 0.0);
            const PlanResult c = p2.plan({-90, -90, kPi / 4}, 150.0, 60.0, f.b, 3, 0.0);
            ASSERT_TRUE(a.improved()) << rollout;
            ASSERT_EQ(a.path.size(), c.path.size());
            EXPECT_EQ(a.reward.of(mode), c.reward.of(mode));
            EXPECT_EQ(a.treeSize, c.treeSize);
            EXPECT_LE(a.cost, 150.0 + 1e-9);
            EXPECT_GT(a.reward.of(mode), 0.0);
            expectConnected(a.path);
        }
    }
}

TEST(Mcts, IncrementalRewardEqualsTheFullRewalkAndInformationAccumulatesIt) {
    Fixture f;
    for (const bool sweep : {false, true}) {
        PlannerSetup su = f.su;
        if (sweep) {
            su.camera.sweep = true;
            su.camera.sweepRate = 30.0 * kPi / 180.0;
            su.camera.sweepAmplitude = 45.0 * kPi / 180.0;
        }
        for (const RewardMode mode : {RewardMode::ORIGINAL, RewardMode::MATCHED}) {
            BaselineParams bp = unitParams("mcts_search");
            bp.reward.mode = mode;
            bp.maxIterations = 40;
            MctsSearchPlanner p(f.g, su, bp);
            p.plan({-90, -90, kPi / 4}, 150.0, 60.0, f.b, 0, 0.7);
            const auto& nodes = p.nodes();
            ASSERT_GT(nodes.size(), 20u);
            for (std::size_t i = 1; i < nodes.size(); ++i) {
                const MctsNode& n = nodes[i];
                const MctsNode& par = nodes[static_cast<std::size_t>(n.t.parent)];
                const double ref = rewalk(p, nodes, static_cast<int>(i), f.b, mode, 0.7);
                EXPECT_NEAR(n.searchInfo, ref, 1e-9 * std::max(1.0, ref));
                EXPECT_DOUBLE_EQ(n.edgeValue, n.searchInfo);                  // edge_value = cumulative
                EXPECT_DOUBLE_EQ(n.information, par.information + n.searchInfo);  // (sic) sums cumulatives
            }
        }
    }
}

TEST(Mcts, PrimitiveFanIsTheHardCodedFiveAnglesAtTwoRadii) {
    Fixture f;
    BaselineParams bp = unitParams("mcts_search");
    MctsSearchPlanner p(f.g, f.su, bp);
    const Pose2 from{5.0, -3.0, 0.3};
    const double deg[5] = {-120, -60, 0, 60, 120};
    for (int a = 0; a < 10; ++a) {
        const Pose2 g = p.primitiveGoal(from, a);
        const double ang = deg[a % 5] * kPi / 180.0 + 0.3;
        const double rad = a >= 5 ? 3.0 * bp.mctsExtendRadius : bp.mctsExtendRadius;
        EXPECT_NEAR(g.x, from.x + rad * std::cos(ang), 1e-9);
        EXPECT_NEAR(g.y, from.y + rad * std::sin(ang), 1e-9);
        EXPECT_NEAR(wrapPi(g.yaw - ang), 0.0, 1e-12);
    }
}

TEST(Mcts, Ucb1PicksUnvisitedFirstThenUsesTheRootVisitCount) {
    Fixture f;
    BaselineParams bp = unitParams("mcts_search");
    bp.maxIterations = 1;
    MctsSearchPlanner p(f.g, f.su, bp);
    const double budget = 150.0;
    p.plan({-90, -90, kPi / 4}, budget, 60.0, f.b, 0, 0.0);
    auto& n = p.mutableNodes();
    ASSERT_GE(n[0].children.size(), 3u);
    const int a = n[0].children[0], b = n[0].children[1], c = n[0].children[2];
    for (const int k : n[0].children) {
        n[static_cast<std::size_t>(k)].nVisits = 4;
        n[static_cast<std::size_t>(k)].edgeValue = 0.0;
        n[static_cast<std::size_t>(k)].futureValue = 0.0;
        n[static_cast<std::size_t>(k)].children.clear();
    }
    n[static_cast<std::size_t>(c)].nVisits = 0;  // unvisited: DBL_MAX
    EXPECT_EQ(p.selectChildHighestUcb1(0), c);
    n[static_cast<std::size_t>(c)].nVisits = 4;
    n[0].nVisits = 100;
    n[static_cast<std::size_t>(a)].edgeValue = 1.0;   // value 1.0
    n[static_cast<std::size_t>(b)].edgeValue = 0.9;   // value 0.9, but visited once
    n[static_cast<std::size_t>(b)].nVisits = 1;
    const double T = budget / f.su.speed, w = bp.mctsExplorationWeight;
    const double ua = 1.0 / T + w * std::sqrt(std::log(100.0) / 4.0);
    const double ub = 0.9 / T + w * std::sqrt(std::log(100.0) / 1.0);
    EXPECT_NEAR(p.ucb1(n[static_cast<std::size_t>(a)]), ua, 1e-12);
    EXPECT_NEAR(p.ucb1(n[static_cast<std::size_t>(b)]), ub, 1e-12);
    EXPECT_EQ(p.selectChildHighestUcb1(0), ub > ua ? b : a);
    // ties: the first child wins (strict >)
    n[static_cast<std::size_t>(b)].nVisits = 4;
    n[static_cast<std::size_t>(b)].edgeValue = 1.0;
    EXPECT_EQ(p.selectChildHighestUcb1(0), a);
}

TEST(Mcts, RootStatisticsCarryOverAndThePlanIsTheValueDescent) {
    Fixture f;
    BaselineParams bp = unitParams("mcts_search");
    bp.maxIterations = 30;
    MctsSearchPlanner p(f.g, f.su, bp);
    p.plan({-90, -90, kPi / 4}, 150.0, 60.0, f.b, 0, 0.0);
    const double v1 = p.nodes()[0].nVisits;
    EXPECT_DOUBLE_EQ(v1, 30.0);  // one back-propagation per replan_loop
    const PlanResult r = p.plan({-80, -90, kPi / 4}, 150.0, 60.0, f.b, 1, 0.0);
    EXPECT_DOUBLE_EQ(p.nodes()[0].nVisits, 60.0);  // tree_root's n_visits is never reset
    // get_leaf_node_of_best_path: at every level the child with the highest get_value
    const auto& n = p.nodes();
    int cur = 0;
    std::size_t depth = 0;
    while (!n[static_cast<std::size_t>(cur)].children.empty()) {
        int bestC = -1;
        double bestV = -std::numeric_limits<double>::infinity();
        for (const int c : n[static_cast<std::size_t>(cur)].children) {
            if (n[static_cast<std::size_t>(c)].value() > bestV) {
                bestV = n[static_cast<std::size_t>(c)].value();
                bestC = c;
            }
        }
        if (bestC < 0) break;
        cur = bestC;
        ++depth;
        ASSERT_LT(depth, r.path.size());
        EXPECT_NEAR(r.path[depth].pose.x, n[static_cast<std::size_t>(cur)].t.pose.x, 1e-12);
    }
    EXPECT_EQ(depth + 1, r.path.size());
}

TEST(Mcts, InformedActionsAimAtCellCentres) {
    Fixture f;
    BaselineParams bp = unitParams("mcts_search");
    bp.mctsRolloutType = "informed";
    bp.maxIterations = 5;
    MctsSearchPlanner p(f.g, f.su, bp);
    p.plan({-90, -90, kPi / 4}, 1e5, 60.0, f.b, 0, 0.0);
    int atCentre = 0, total = 0;
    for (const int c : p.nodes()[0].children) {
        const TreeNode& t = p.nodes()[static_cast<std::size_t>(c)].t;
        ++total;
        const double fx = (t.pose.x - f.g.xMin) / f.g.res - 0.5, fy = (t.pose.y - f.g.yMin) / f.g.res - 0.5;
        if (std::fabs(fx - std::round(fx)) < 1e-6 && std::fabs(fy - std::round(fy)) < 1e-6) ++atCentre;
    }
    EXPECT_EQ(atCentre, total);  // no viewing-point back-off: the node is the cell centre
    EXPECT_EQ(total, bp.mctsNumActions);
}
