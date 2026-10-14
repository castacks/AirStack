// =============================================================================
//  ipp_baselines_search_planner/planner_core.hpp — what the ia-tigris baselines
//  share (ia-tigris/src/ipp_planners: Planner.h, TreeNode.h, InfoMapSearch.h,
//  SearchMap.cpp), on top of the IA-TIGRIS port's core (scenario, Dubins, belief and
//  rewards, receding horizon), so every planner is scored on one sensor, gimbal and
//  belief model.
//
//   * TreeNode              TreeNode.h: pose, parent, cost, budget_remaining, depth, the
//                           edge from the parent (the Dubins path cut at edgeLen) and its
//                           straight part (straight_edge_start/end_pose).
//   * extend()              Planner::extend_from_start_node_toward_goal_node, exactly:
//                           same-pose clone, walk the path, stop BEFORE the state that
//                           reaches the budget or extend_dist, zero-length node when the
//                           first step already does, straight-edge start / end saved by
//                           maybe_set_edge_start_pose / maybe_set_edge_end_pose (an edge
//                           whose straight part is never followed by a turn has NO edge).
//   * originalNodeGain()    the ia-tigris SearchMap reward of one node (its straight edge,
//                           then its footprint, or the reverse for the non-hashmap walk),
//                           with the cone footprint and the swept gimbal phase.
//   * SearchPlanner         Planner.h's replan(): replan_setup, replan_loop while time
//                           (and max_iterations) allow and the loop says continue,
//                           replan_teardown, then get_best_path_segments.
//
//  The only changes from ia-tigris are the ones every AirStack port makes: the cone
//  footprint and single-axis gimbal (belief.hpp), reward_mode (matched / original),
//  Dubins instead of a zero-wind trochoid (0.5 m walk instead of 40 m discretisation,
//  which is also the "one step" final_path_discretization_distance stands for), seeded
//  RNGs and the receding-horizon glue (receding.hpp).
// =============================================================================
#ifndef IPP_BASELINES_SEARCH_PLANNER_PLANNER_CORE_HPP
#define IPP_BASELINES_SEARCH_PLANNER_PLANNER_CORE_HPP

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "ipp_baselines_search_planner/belief.hpp"
#include "ipp_baselines_search_planner/dubins.hpp"

namespace ipp_baselines_search {

/// Planner parameters. Released ia-tigris fixed-wing values in the comments.
struct BaselineParams {
    std::string planner = "greedy";  ///< greedy | rect_coverage | mcts_search (ipp_planners_node.h init_planner)
    double rewardStep = 2.0;         ///< [m] look spacing along an edge (MATCHED reward and swept-camera swath)
    double viewPointGoal = 0.6;      ///< planner.yaml viewpoint_goal
    bool   useEdgeReward = true;     ///< planner.yaml use_edge_reward (include_edge)
    int    maxIterations = 0;        ///< > 0: stop each solve after this many replan_loop calls (reproducible)
    std::uint64_t seed = 21;
    RewardParams reward;
    // rect_coverage
    bool coverageEnforceTurnCheck = false;  ///< RectangleCoverage: throw if 2 R < sensor width (off: deviation D2)
    // mcts_search (mcts_search.yaml)
    int    mctsMaxRolloutDepth = 5;          ///< max_rollout_depth
    int    mctsNumActions = 10;              ///< num_actions
    double mctsExplorationWeight = 0.141421356;  ///< exploration_weight
    double mctsExtendAngle = 3.1415926;      ///< mcts_extend_angle (read, unused by MCTSSearch.h)
    double mctsExtendRadius = 300.0;         ///< mcts_extend_radius [m]
    double mctsExtendDist = 3000.0;          ///< mcts_extend_dist [m] (read, unused: steer is unbounded)
    int    mctsBranchesEachSide = 3;         ///< mcts_branches_each_side (read, unused)
    std::string mctsRolloutType = "primitive";  ///< primitive | informed (learned: refused)
    bool   mctsRetainMapCopies = false;      ///< keep every node's InfoMapSearch copy alive (ROS 1 memory)
};

/// Everything fixed for one agent (world ENU, z = height above ground).
struct PlannerSetup {
    Camera camera;
    DetectionModel det;
    double speed = 6.0;          ///< [m/s] desired_speed
    double turnRadius = 12.0;    ///< [m] 1 / max_kappa
    double dubinsStep = 0.5;     ///< [m] path discretisation (final_path_discretization_distance)
    double altitude = 30.0;      ///< [m] flight_height
    double xMin = -200, xMax = 200, yMin = -200, yMax = 200;  ///< search bounds
};

struct TreeNode {
    Pose2 pose;
    int parent = -1;
    double cost = 0.0;            ///< path length from the root [m] (chord sums, as dist_covered)
    double budgetRemaining = 0.0; ///< budget_remaining
    int depth = 0;
    double info = 0.0;            ///< the planner's reward of root -> node (active mode)
    bool closed = false;          ///< is_in_closed_set
    DubinsPath edge;              ///< parent -> node (cut at edgeLen)
    double edgeLen = 0.0;
    double arc = 0.0;             ///< Dubins arc length from the root [m] (sets the sweep phase)
    bool hasEdge = false;         ///< straight_edge_start_pose and straight_edge_end_pose both set
    Pose2 edgeStart, edgeEnd;
    double edgeStartS = 0.0, edgeEndS = 0.0;  ///< arc of edgeStart / edgeEnd along `edge`
};

/// Planner.h maybe_set_edge_start_pose / maybe_set_edge_end_pose over consecutive path
/// states (cur -> next): the straight part starts at the first state whose successor keeps
/// the heading (|dpsi| < 1e-4) and ends at the first later state whose successor turns.
struct EdgeTracker {
    bool startSaved = false, endSaved = false;
    Pose2 start, end;
    double startS = 0.0, endS = 0.0;
    void update(const Pose2& cur, double curS, const Pose2& next);
    void writeTo(TreeNode* n) const;
};

struct PlanResult {
    std::vector<TreeNode> path;   ///< root .. last node (path[0] is the root)
    PathReward reward;            ///< both rewards of the path (yardstick, scorePath)
    double cost = 0.0;            ///< its length [m]
    int iterations = 0;           ///< replan_loop calls
    int treeSize = 0;
    double seconds = 0.0;
    std::map<std::string, double> extra;  ///< planner-specific numbers for the replans file
    bool improved() const { return path.size() > 1; }
};

/// Shared helpers (Planner.h / SearchMap.cpp).
class PlannerCore {
public:
    PlannerCore(const PlanningGrid& grid, const PlannerSetup& setup, const BaselineParams& params);

    /// Planner::extend_from_start_node_toward_goal_node. `out` gets parent = `fromIdx`.
    /// Returns false where the original returns nullptr (a one-state path).
    bool extend(const TreeNode& from, int fromIdx, const Pose2& goal, double extendDist, TreeNode* out) const;

    /// Planner::find_viewing_point with the cone: back off h tan(tilt - vg fov / 2) along the
    /// heading from `from` to the target.
    Pose2 findViewingPoint(const Pose2& from, double tx, double ty) const;
    /// h / sin(pitch + vg vfov / 2) with the cone = h / cos(tilt - vg fov / 2).
    double nominalRange() const;

    /// Looks along one edge (every rewardStep); tStart = track time at the edge's start.
    std::vector<Look> edgeLooks(const DubinsPath& edge, double len, double tStart) const;
    /// ORIGINAL reward of one node on the evaluator's current overlay. edgeFirst = true:
    /// search_information_gain_with_hashmaps (edge then footprint); false: the full walk
    /// search_information_gain (footprint then edge).
    void originalNodeGain(RewardEvaluator& ev, const TreeNode& n, double startTime, bool edgeFirst,
                          bool includeEdge) const;
    /// MATCHED reward of one node on the evaluator's current overlay: the looks of its edge.
    void matchedNodeGain(RewardEvaluator& ev, const TreeNode& n, double startTime) const;
    /// Both rewards of a node chain re-walked from the root (hashmap order), the yardstick
    /// written to the replans file.
    PathReward scorePath(const std::vector<TreeNode>& path, const BeliefState& belief, double startTime);

    const PlanningGrid& grid() const { return g_; }
    const PlannerSetup& setup() const { return s_; }
    const BaselineParams& params() const { return p_; }
    RewardEvaluator& evaluator() { return eval_; }

protected:
    const PlanningGrid& g_;
    PlannerSetup s_;
    BaselineParams p_;
    RewardEvaluator eval_;
};

/// Planner.h: replan() = replan_setup, replan_loop while time allows, replan_teardown,
/// get_best_path_segments. One instance lives for the whole sortie.
class SearchPlanner : public PlannerCore {
public:
    using PlannerCore::PlannerCore;
    virtual ~SearchPlanner() = default;

    /// One solve from `start` with path-length `budget` over `belief`. `replanIndex` offsets
    /// the RNG seed; `startTime` is the planned track time at `start` (the sweep phase).
    PlanResult plan(const Pose2& start, double budget, double timeLimit, const BeliefState& belief,
                    int replanIndex, double startTime);

    virtual const char* name() const = 0;

protected:
    virtual bool setupSolve(const Pose2& start, double budget) = 0;  ///< replan_setup
    virtual bool loopOnce() = 0;                                      ///< replan_loop: false = stop
    virtual void teardown() {}                                        ///< replan_teardown
    virtual std::vector<TreeNode> bestPath() = 0;                     ///< get_best_path_segments
    virtual int treeSize() const { return 0; }
    virtual void extras(std::map<std::string, double>&) const {}

    const BeliefState* belief_ = nullptr;
    double startTime_ = 0.0;
    double budget_ = 0.0;
    int replanIndex_ = 0;
    std::chrono::steady_clock::time_point t0_;
    double elapsed() const;
};

/// ipp_planners_node.h init_planner: greedy | rect_coverage | mcts_search.
std::unique_ptr<SearchPlanner> makePlanner(const PlanningGrid& grid, const PlannerSetup& setup,
                                           const BaselineParams& params);
/// Throws std::invalid_argument for an unknown or unsupported planner / option.
void validateParams(const BaselineParams& params);

/// Seed of a planner RNG for one solve (the IA port's rule: mission seed + replan index).
std::uint64_t solveSeed(std::uint64_t seed, int replanIndex);

}  // namespace ipp_baselines_search

#endif  // IPP_BASELINES_SEARCH_PLANNER_PLANNER_CORE_HPP
