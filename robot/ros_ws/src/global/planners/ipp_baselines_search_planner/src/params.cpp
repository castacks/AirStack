// =============================================================================
//  params.cpp — key=value parameters (CLI --set and node parameters).
// =============================================================================
#include "ipp_baselines_search_planner/params.hpp"

#include <cmath>
#include <stdexcept>

namespace ipp_baselines_search {

namespace {
constexpr double kDeg = 3.14159265358979323846 / 180.0;
bool toBool(const std::string& v) { return v == "1" || v == "true" || v == "True" || v == "yes"; }
}  // namespace

const std::vector<std::string>& paramKeys() {
    static const std::vector<std::string> k = {
        "planner", "reward_mode", "planning_time_s", "initial_planning_time_s", "replan_period_s",
        "commit_margin_s", "lookahead_m", "snap_to_node", "reward_step_m", "grid_res_m", "view_point_goal",
        "use_edge_reward", "use_entropy", "rs", "rf", "initial_confidence", "budget_m", "camera_fov_deg",
        "camera_tilt_deg", "max_iterations", "seed", "rect_coverage_enforce_turn_check",
        "mcts_max_rollout_depth", "mcts_num_actions", "mcts_exploration_weight", "mcts_extend_angle_rad",
        "mcts_extend_radius_m", "mcts_extend_dist_m", "mcts_branches_each_side", "mcts_rollout_type",
        "mcts_retain_map_copies"};
    return k;
}

bool applyParam(const std::string& k, const std::string& v, BaselineParams& bp, HorizonParams& hp, Camera& cam,
                bool& camSet) {
    auto d = [&]() { return std::stod(v); };
    if (k == "planner") bp.planner = v;
    else if (k == "reward_mode") bp.reward.mode = rewardModeFromString(v);
    else if (k == "planning_time_s") hp.replanPlanningTime = d();
    else if (k == "initial_planning_time_s") hp.initialPlanningTime = d();
    else if (k == "replan_period_s") hp.replanPeriod = d();
    else if (k == "commit_margin_s") hp.commitMarginTime = d();
    else if (k == "lookahead_m") hp.lookahead = d();
    else if (k == "snap_to_node") hp.snapToNode = toBool(v);
    else if (k == "reward_step_m") bp.rewardStep = d();
    else if (k == "grid_res_m") hp.gridRes = d();
    else if (k == "view_point_goal") bp.viewPointGoal = d();
    else if (k == "use_edge_reward") bp.useEdgeReward = toBool(v);
    else if (k == "use_entropy") bp.reward.useEntropy = toBool(v);
    else if (k == "rs") bp.reward.rs = d();
    else if (k == "rf") bp.reward.rf = d();
    else if (k == "initial_confidence") bp.reward.initialConfidence = d();
    else if (k == "budget_m") hp.budgetOverride = d();
    else if (k == "camera_fov_deg") { if (d() > 0) { cam.fov = d() * kDeg; camSet = true; } }
    else if (k == "camera_tilt_deg") { if (d() >= 0) { cam.tilt = d() * kDeg; camSet = true; } }
    else if (k == "max_iterations") bp.maxIterations = static_cast<int>(d());
    else if (k == "seed") { if (d() >= 0) bp.seed = static_cast<std::uint64_t>(d()); }
    else if (k == "rect_coverage_enforce_turn_check") bp.coverageEnforceTurnCheck = toBool(v);
    else if (k == "mcts_max_rollout_depth") bp.mctsMaxRolloutDepth = static_cast<int>(d());
    else if (k == "mcts_num_actions") bp.mctsNumActions = static_cast<int>(d());
    else if (k == "mcts_exploration_weight") bp.mctsExplorationWeight = d();
    else if (k == "mcts_extend_angle_rad") bp.mctsExtendAngle = d();
    else if (k == "mcts_extend_radius_m") bp.mctsExtendRadius = d();
    else if (k == "mcts_extend_dist_m") bp.mctsExtendDist = d();
    else if (k == "mcts_branches_each_side") bp.mctsBranchesEachSide = static_cast<int>(d());
    else if (k == "mcts_rollout_type") bp.mctsRolloutType = v;
    else if (k == "mcts_retain_map_copies") bp.mctsRetainMapCopies = toBool(v);
    else return false;
    return true;
}

}  // namespace ipp_baselines_search
