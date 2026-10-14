// =============================================================================
//  planner_core.cpp — Planner.h / SearchMap.cpp pieces shared by the baselines.
// =============================================================================
#include "ipp_baselines_search_planner/planner_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ipp_baselines_search {

namespace {
constexpr double kTurnEps = 0.0001;  // Planner.h maybe_set_edge_*_pose eps
constexpr double kSamePose = 0.00001;  // Planner.h TOLERANCE
using Clock = std::chrono::steady_clock;
}  // namespace

std::uint64_t solveSeed(std::uint64_t seed, int replanIndex) {
    return seed * 1000003ULL + static_cast<std::uint64_t>(replanIndex) * 7919ULL + 17ULL;
}

void EdgeTracker::update(const Pose2& cur, double curS, const Pose2& next) {
    // maybe_set_edge_start_pose: heading unchanged -> we are on the straight part
    if (!startSaved && std::fabs(wrapPi(next.yaw - cur.yaw)) < kTurnEps) {
        start = cur;
        startS = curS;
        startSaved = true;
    }
    // maybe_set_edge_end_pose: after the start, the heading changes -> the straight part ended
    if (startSaved && !endSaved && std::fabs(wrapPi(cur.yaw - next.yaw)) > kTurnEps) {
        end = cur;
        endS = curS;
        endSaved = true;
    }
}

void EdgeTracker::writeTo(TreeNode* n) const {
    // search_information_gain*: an edge counts only when both poses are set (size() == 4)
    n->hasEdge = startSaved && endSaved;
    n->edgeStart = start;
    n->edgeEnd = end;
    n->edgeStartS = startS;
    n->edgeEndS = endS;
}

PlannerCore::PlannerCore(const PlanningGrid& grid, const PlannerSetup& setup, const BaselineParams& params)
    : g_(grid), s_(setup), p_(params), eval_(grid, setup.det, params.reward) {}

// Planner::extend_from_start_node_toward_goal_node (Planner.h:399-513)
bool PlannerCore::extend(const TreeNode& from, int fromIdx, const Pose2& goal, double extendDist,
                         TreeNode* out) const {
    TreeNode n;
    n.parent = fromIdx;
    n.depth = from.depth + 1;
    // XYZPsi distance (|dxyz| + SO2 distance, weights 1) below TOLERANCE: a copied state
    const double d0 = std::hypot(goal.x - from.pose.x, goal.y - from.pose.y) +
                      std::fabs(wrapPi(goal.yaw - from.pose.yaw));
    if (d0 < kSamePose) {
        n.pose = from.pose;
        n.cost = from.cost;
        n.budgetRemaining = from.budgetRemaining;
        n.edge = DubinsPath();
        n.edge.q0 = from.pose;
        n.edge.rho = s_.turnRadius;
        n.edgeLen = 0.0;
        n.arc = from.arc;
        *out = n;
        return true;
    }
    DubinsPath dp;
    if (!dubinsShortest(from.pose, goal, s_.turnRadius, &dp)) return false;  // !valid
    const double L = dp.length();
    const double step = std::max(s_.dubinsStep, 1e-3);
    const int nSteps = L > 0.0 ? std::max(1, static_cast<int>(std::ceil(L / step - 1e-9))) : 0;
    if (nSteps == 0) return false;  // trochoid_path_temp.size() == 1
    const double budget = from.budgetRemaining;
    EdgeTracker et;
    double dist = 0.0, curS = 0.0;
    Pose2 cur = from.pose;  // trochoid state 0 is the start state
    bool closed = false;
    for (int i = 1; i <= nSteps; ++i) {
        const double sv = std::min(L, i * step);
        const Pose2 next = dp.sample(sv);
        const double d = std::hypot(next.x - cur.x, next.y - cur.y);  // same z: euclidean
        // is_collision: never (strict_stay_in_bounds false, no keep-out zones)
        if (dist + d >= budget || dist + d >= extendDist) {
            if (dist >= budget - step) closed = true;  // is_distance_nearly_exceeded
            break;
        }
        et.update(cur, curS, next);
        dist += d;
        cur = next;
        curS = sv;
        if (i == nSteps && !closed && dist >= budget - step) closed = true;
    }
    if (!closed && dist >= budget - step) closed = true;
    n.pose = cur;
    n.closed = closed;
    n.cost = from.cost + dist;
    n.budgetRemaining = budget - dist;
    n.edge = dp;
    n.edgeLen = curS;
    n.arc = from.arc + curS;
    et.writeTo(&n);
    *out = n;
    return true;
}

Pose2 PlannerCore::findViewingPoint(const Pose2& from, double tx, double ty) const {
    // Planner::find_viewing_point: declination = pitch + vg vfov / 2 (from the horizon);
    // with the cone the look angle from nadir is tilt - vg fov / 2
    Pose2 q;
    q.yaw = std::atan2(ty - from.y, tx - from.x);
    const double back = s_.altitude * std::tan(s_.camera.tilt - p_.viewPointGoal * s_.camera.fov / 2.0);
    q.x = tx - back * std::cos(q.yaw);
    q.y = ty - back * std::sin(q.yaw);
    return q;
}

double PlannerCore::nominalRange() const {
    return s_.altitude / std::cos(s_.camera.tilt - p_.viewPointGoal * s_.camera.fov / 2.0);
}

std::vector<Look> PlannerCore::edgeLooks(const DubinsPath& edge, double len, double tStart) const {
    std::vector<Look> out;
    if (!(len > 0.0)) return out;
    const double step = std::max(p_.rewardStep, 1e-3);
    const int n = std::max(1, static_cast<int>(std::ceil(len / step - 1e-9)));
    out.reserve(static_cast<std::size_t>(n));
    double prev = 0.0;
    for (int k = 1; k <= n; ++k) {
        const double s = std::min(len, k * step);
        const Pose2 q = edge.sample(s);
        const double w = (s - prev) / s_.speed / s_.det.dtRef;
        prev = s;
        out.push_back(lookFromPose(q.x, q.y, s_.altitude, q.yaw, s_.camera, s_.det, w,
                                   s_.camera.phiAt(tStart + s / s_.speed)));
    }
    return out;
}

void PlannerCore::originalNodeGain(RewardEvaluator& ev, const TreeNode& n, double startTime, bool edgeFirst,
                                   bool includeEdge) const {
    const Camera& cam = s_.camera;
    auto edgePart = [&]() {
        if (!includeEdge || !n.hasEdge) return;
        if (!cam.sweep) {
            ev.tigrisEdge(n.edgeStart.x, n.edgeStart.y, n.edgeEnd.x, n.edgeEnd.y, n.edgeStart.yaw, s_.altitude, cam);
            return;
        }
        // swept camera: the swath is the union of the footprints along the straight part
        const double tEdge = startTime + (n.arc - n.edgeLen) / s_.speed;
        const double a = n.edgeStartS, b = std::max(n.edgeEndS, a);
        const double step = std::max(p_.rewardStep, 1e-3);
        const int m = std::max(1, static_cast<int>(std::ceil((b - a) / step - 1e-9)));
        std::vector<RewardEvaluator::ViewPose> poses;
        for (int k = 0; k <= m; ++k) {
            const double sv = std::min(b, a + k * step);
            const Pose2 q = n.edge.sample(sv);
            poses.push_back({q.x, q.y, q.yaw, cam.phiAt(tEdge + sv / s_.speed)});
        }
        ev.tigrisEdgeSwept(poses, s_.altitude, cam);
    };
    auto nodePart = [&]() {
        ev.tigrisNode(n.pose.x, n.pose.y, s_.altitude, n.pose.yaw, cam, cam.phiAt(startTime + n.arc / s_.speed));
    };
    if (edgeFirst) {
        edgePart();
        nodePart();
    } else {
        nodePart();
        edgePart();
    }
}

void PlannerCore::matchedNodeGain(RewardEvaluator& ev, const TreeNode& n, double startTime) const {
    const double tEdge = startTime + (n.arc - n.edgeLen) / s_.speed;
    ev.pass(edgeLooks(n.edge, n.edgeLen, tEdge), false, true);
}

PathReward PlannerCore::scorePath(const std::vector<TreeNode>& path, const BeliefState& belief, double startTime) {
    PathReward r;
    eval_.begin(belief);
    for (const TreeNode& n : path) originalNodeGain(eval_, n, startTime, true, true);
    r.original = eval_.reward().original;
    eval_.begin(belief);
    for (std::size_t i = 1; i < path.size(); ++i) matchedNodeGain(eval_, path[i], startTime);
    r.matched = eval_.reward().matched;
    return r;
}

double SearchPlanner::elapsed() const { return std::chrono::duration<double>(Clock::now() - t0_).count(); }

// Planner::replan (Planner.h:124-220)
PlanResult SearchPlanner::plan(const Pose2& start, double budget, double timeLimit, const BeliefState& belief,
                               int replanIndex, double startTime) {
    PlanResult res;
    belief_ = &belief;
    startTime_ = startTime;
    budget_ = budget;
    replanIndex_ = replanIndex;
    t0_ = Clock::now();
    if (setupSolve(start, budget)) {
        bool cont = true;
        while (elapsed() < timeLimit && cont && (p_.maxIterations <= 0 || res.iterations < p_.maxIterations)) {
            ++res.iterations;
            cont = loopOnce();
        }
    }
    teardown();
    res.path = bestPath();
    res.cost = res.path.empty() ? 0.0 : res.path.back().cost;
    res.treeSize = treeSize();
    res.reward = scorePath(res.path, belief, startTime);
    extras(res.extra);
    res.seconds = elapsed();
    return res;
}

void validateParams(const BaselineParams& p) {
    if (p.planner != "greedy" && p.planner != "rect_coverage" && p.planner != "mcts_search") {
        throw std::invalid_argument("planner must be greedy, rect_coverage or mcts_search (got '" + p.planner + "')");
    }
    if (p.planner == "mcts_search") {
        if (p.mctsRolloutType == "learned") {
            throw std::invalid_argument("mcts_rollout_type 'learned' needs the learned rollout models, which are "
                                        "not in ia-tigris (commented out in MCTSSearch.h): unsupported");
        }
        if (p.mctsRolloutType != "primitive" && p.mctsRolloutType != "informed") {
            throw std::invalid_argument("mcts_rollout_type must be primitive or informed (got '" +
                                        p.mctsRolloutType + "')");
        }
        if (p.mctsNumActions < 1) throw std::invalid_argument("mcts_num_actions must be >= 1");
    }
}

}  // namespace ipp_baselines_search
