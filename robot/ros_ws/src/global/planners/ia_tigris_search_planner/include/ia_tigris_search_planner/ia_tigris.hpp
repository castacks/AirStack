// =============================================================================
//  ia_tigris_search_planner/ia_tigris.hpp — IA-TIGRIS: an Incremental and Adaptive
//  sampling-based planner for online informative path planning (Moon, Suvarna,
//  Jong, Chatterjee, Yuan, Cao, Scherer, IEEE T-RO 2026), single agent.
//
//  Built on the TIGRIS tree search of tigris_search_planner (a line-by-line port of
//  tigris/src/ipp.cpp) with the three changes ia-tigris/src/ipp_planners/src/Tigris.cpp
//  makes on top of it:
//
//   1. THE TREE IS KEPT BETWEEN SOLVES (Tigris::update_tree / recycle_tree_from_node).
//      A replan starts from a node of the previous best path (the receding horizon
//      snaps its commit point forward to that node, as PlannerNode::
//      find_expected_waypoint_reached_after_time_to_plan does). The tree is re-rooted
//      there: every node that is not in that node's subtree is dropped
//      (erase_nodes_before_start), costs are re-based, nodes over the new budget are
//      deleted with their subtrees, and every reward is recomputed against the
//      UPDATED belief, root to leaf (add_children_to_tree). Sampling then continues on
//      the recycled tree. If the start is not a tree node (first solve, or recycling
//      off) the tree is reset to the start pose (reset_tree_to_pose).
//
//   2. INCREMENTAL REWARD (SearchMap::search_information_gain_with_hashmaps). Every
//      node stores the sparse belief updates of its own footprint and edge
//      (local_search_map_updates). A new node's reward is its parent's reward plus the
//      reward of its own looks evaluated on the belief at the end of its ancestor
//      chain (the ancestors' updates replayed over the base belief), instead of
//      re-walking the whole chain from the root. Per node: FIRST the straight part of
//      its edge, THEN its footprint, and both use SearchMap::calc_reward_and_belief
//      (entropy reduction x Rs if the belief rose, x Rf otherwise; Rs = Rf = 1 in
//      IA-TIGRIS). This is the IA-TIGRIS ORIGINAL reward; it differs from TIGRIS's
//      (node before edge, a different edge formula, Rs = 2).
//
//   3. NEAR-NODE LOOP (Tigris::extend_neighbors_towards_node / add_node_to_tree_or_prune):
//      every node that is actually ADDED is compared with the best path, so the best
//      path can end at a rewired node (TIGRIS compared the sampled node instead).
//
//  Everything else is the TIGRIS port: informed sampler (cell drawn with probability
//  ~ its one-look reward on the CURRENT belief, backed off along a random yaw; IA-TIGRIS
//  samples the cell centre, so the int truncation of ipp.cpp is gone), Dubins steer cut
//  at extend_dist or the budget, `closed` flag, prune(m) = a tree node within
//  prune_radius with cost <= and reward >=, the two reward models (ORIGINAL entropy /
//  MATCHED residual belief mass) and the single-axis gimbal (body-fixed or swept).
//
//  What differs from the ROS 1 code, and only because AirStack forces it (as for
//  tigris_search_planner): exact spatial hash instead of OMPL GNAT, Dubins instead of a
//  zero-wind trochoid, the configured cone footprint instead of the rectangular
//  frustum, a seeded RNG.
// =============================================================================
#ifndef IA_TIGRIS_SEARCH_PLANNER_IA_TIGRIS_HPP
#define IA_TIGRIS_SEARCH_PLANNER_IA_TIGRIS_HPP

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "ia_tigris_search_planner/belief.hpp"
#include "ia_tigris_search_planner/dubins.hpp"

namespace ia_tigris_search {

struct TigrisParams {
    double extendDist = 750.0;    ///< [m] steer cut (= tigris_search; released ia-tigris config: 1500)
    double extendRadius = 251.0;  ///< [m] near-node radius (= tigris_search; released: 1500)
    double pruneRadius = 750.0;   ///< [m] (= tigris_search; released: 400)
    double rewardStep = 2.0;      ///< [m] look spacing when scoring an edge (MATCHED)
    double viewPointGoal = 0.6;   ///< informed sampler: where in the FOV the sampled cell should sit
    double psiWeight = 1.0;       ///< nearest-neighbour metric weight on |dpsi| [m/rad]
    double boundsMargin = -1.0;   ///< [m] geofence around the area (< 0 = off)
    bool   informedSampler = true;
    bool   recycleTree = true;    ///< IA-TIGRIS tree recycling (false = a fresh tree every solve)
    int    maxIterations = 0;     ///< > 0: stop after this many samples (deterministic runs)
    std::uint64_t seed = 21;
    RewardParams reward;
};

/// Everything fixed for one agent (world ENU, z = height above ground).
struct PlannerSetup {
    Camera camera;
    DetectionModel det;
    double speed = 6.0;          ///< [m/s] used for the look weights dt / dt_ref
    double turnRadius = 12.0;    ///< [m]
    double dubinsStep = 0.5;     ///< [m] steer walk resolution
    double altitude = 30.0;      ///< [m] above ground
    double xMin = -200, xMax = 200, yMin = -200, yMax = 200;  ///< search area
};

struct TreeNode {
    Pose2 pose;
    int parent = -1;
    double cost = 0.0;       ///< path length from the root [m]
    double info = 0.0;       ///< reward of root -> node in the ACTIVE mode
    bool closed = false;     ///< hit the budget: never expanded
    DubinsPath edge;         ///< parent -> node (cut at edgeLen)
    double edgeLen = 0.0;
    double arc = 0.0;        ///< Dubins arc length from the root [m] (the track's arc; sets the sweep phase)
    bool inTree = false;     ///< added to the nearest-neighbour structure (not pruned)
    bool hasEdge = false;    ///< a straight segment was recorded (Motion::start_edge / end_edge)
    Pose2 edgeStart, edgeEnd;
    double edgeStartS = 0.0, edgeEndS = 0.0;  ///< arc of edgeStart / edgeEnd along `edge`
};

struct PlanResult {
    std::vector<TreeNode> path;   ///< root .. best (path[0] is the root)
    PathReward reward;            ///< both rewards of the best path
    double cost = 0.0;            ///< its length [m]
    int iterations = 0;
    int treeSize = 0;
    int nodesPruned = 0;
    double seconds = 0.0;         ///< whole solve, recycling included
    // IA-TIGRIS tree recycling
    bool recycled = false;        ///< the tree was recycled (false: reset to the start pose)
    int nodesKept = 0;            ///< tree nodes carried over from the previous solve
    int nodesDropped = 0;         ///< dropped: before the new root, off its subtree, or over budget
    double recycleSeconds = 0.0;  ///< re-rooting + recomputing every reward on the new belief
    double tailInfo = 0.0;        ///< active-mode reward of the previous plan's tail in the recycled tree
    bool keptTail = false;        ///< the best path IS the previous plan's tail (track not revised)
    bool improved() const { return path.size() > 1; }
};

class IaTigrisPlanner {
public:
    IaTigrisPlanner(const PlanningGrid& grid, const PlannerSetup& setup, const TigrisParams& params);
    ~IaTigrisPlanner();

    /// A solve from `start` on a FRESH tree (reset_tree_to_pose) with path-length `budget`
    /// over `belief`. `replanIndex` offsets the RNG seed; `startTime` is the planned track
    /// time at `start` [s] (the sweep phase).
    PlanResult plan(const Pose2& start, double budget, double timeLimit, const BeliefState& belief,
                    int replanIndex = 0, double startTime = 0.0);

    /// A solve that RECYCLES the tree from node `pathIndex` of the last returned best path
    /// (0 = its root: the tree is kept whole). `budget` is the budget left at that node and
    /// `startTime` its track time. Falls back to a fresh tree at that node's pose when there
    /// is no tree. The recycling time counts against `timeLimit`.
    PlanResult replanFrom(std::size_t pathIndex, double budget, double timeLimit, const BeliefState& belief,
                          int replanIndex, double startTime);

    bool hasTree() const { return !nodes_.empty(); }
    int treeSize() const { return static_cast<int>(nodes_.size()); }
    /// Nodes of the last best path (tree indices, root first).
    const std::vector<int>& lastPath() const { return lastPath_; }

    /// Looks along one edge (every rewardStep, weight = step / speed / dt_ref); `tStart` is the
    /// track time at the edge's start (the swept camera's phase).
    std::vector<Look> edgeLooks(const DubinsPath& edge, double len, double tStart = 0.0) const;
    /// Both rewards of a node chain (as a sequence of passes) over `belief`, re-walked from
    /// the root (the reference the incremental rewards must match).
    PathReward scorePath(const std::vector<TreeNode>& path, const BeliefState& belief, double startTime = 0.0);
    /// ORIGINAL reward of a node chain the TIGRIS way (MapRepresentation::informationGain).
    void tigrisGain(const std::vector<const TreeNode*>& chain, double startTime = 0.0);

    const TigrisParams& params() const { return p_; }
    const PlannerSetup& setup() const { return s_; }

private:
    struct NodeData {
        std::vector<Look> looks;                                ///< MATCHED: the edge's looks
        std::vector<RewardEvaluator::CellUpdate> dPres, dRes;   ///< local_search_map_updates
    };
    class NodeHash;

    void resetTree(const Pose2& start);
    void recycle(int newRoot, double budget, PlanResult* res);
    void grow(double budget, double timeLimit, int replanIndex, PlanResult* res);
    /// Reward of `n` (whose parent is a tree node) on the belief at the end of its chain;
    /// fills its local updates. Returns the node's active-mode reward from the root.
    double evaluate(TreeNode& n, NodeData& d);
    bool steer(int from, const Pose2& target, double budget, TreeNode* out) const;
    bool pruned(const TreeNode& n);
    int addNode(TreeNode&& n, NodeData&& d);
    void rebuildHash();
    void buildSampler(const BeliefState& belief);
    double dist(const Pose2& a, const Pose2& b) const;
    void finish(PlanResult* res);

    const PlanningGrid& g_;
    PlannerSetup s_;
    TigrisParams p_;
    RewardEvaluator eval_;

    // persistent tree (only nodes that were added: pruned candidates are not stored)
    std::vector<TreeNode> nodes_;
    std::vector<NodeData> data_;
    std::unique_ptr<NodeHash> hash_;
    std::vector<int> lastPath_;
    int best_ = 0;
    // per solve
    const BeliefState* belief_ = nullptr;
    double startTime_ = 0.0;
    std::vector<double> weights_;
    double weightSum_ = 0.0;
    std::chrono::steady_clock::time_point t0_;
    int tailEnd_ = -1;        ///< the previous plan's last node in the recycled tree (-1: none)
    double tailInfo_ = 0.0;
    std::vector<int> chainBuf_, nearBuf_;
};

}  // namespace ia_tigris_search

#endif  // IA_TIGRIS_SEARCH_PLANNER_IA_TIGRIS_HPP
