// =============================================================================
//  ipp_baselines_search_planner/mcts_search.hpp — MCTSSearch + MCTSNode
//  (ia-tigris/src/ipp_planners/include/ipp_planners/MCTSSearch.h, MCTSNode.h), as written.
//
//  Node: edge_value, future_value, information, n_visits, children and its OWN copy of the
//  InfoMapSearch (grid, priority, sensor ids, weighted sampler and its RNG).
//  get_value = edge_value + future_value / n_visits (NaN / inf when unvisited, as in C++).
//
//  replan_setup (137-208): root = the member tree_root at the start pose, budget, a copy of the
//    info map, information 0; its n_visits / future_value / edge_value are NOT reset (they carry
//    over from solve to solve); children cleared; expand(root); best = root.
//  replan_loop (210-246): descend by the highest UCB1 (strict >, unvisited = DBL_MAX) to a leaf;
//    if it was visited, expand it and take children[0]; roll it out and back-propagate.
//    UCB1 (91-98) = get_value / (budget / speed) + w sqrt(ln(ROOT visits) / n_visits).
//  expand (249-272): every action 0..num_actions-1; NodeStuckException skips the action.
//  act_and_transition (274-308): make_feasible_action_node up to 6 calls (the 6th call throws
//    even if it succeeded); copy the parent's map into the child; transition = the hashmap
//    reward of the child INCLUDING its parent's search_info (the cumulative root -> child
//    reward); edge_value = transition; information = parent.information + transition.
//  primitive action (387-420): num_angles = num_actions / 2, angle_idx = a % num_angles,
//    radius x3 for a >= num_angles, angle = -120 deg + 240 deg idx / (num_angles - 1) + psi;
//    the goal heading is that angle. informed action (355-377): a cell centre drawn from the
//    FROM node's map copy (its RNG), heading uniform in [-pi, pi - 1e-4). Steer WITHOUT
//    extend_dist (mcts_extend_dist, _angle and _branches_each_side are unused).
//  rollout (458-502): up to max_rollout_depth random actions from the leaf, summing each new
//    node's `information`; the rollout nodes are discarded.
//  back-propagation (441-450): every ancestor future_value += child.get_value(), n_visits++.
//  get_best_path_segments (504-539): from the root, the child with the highest get_value
//    (strict >) until a leaf - not the logged best node.
//
//  Only where ROS 1 has undefined behaviour or crashes does this port stop the solve instead:
//  a NaN UCB descent with no child picked, expanding a node that got no children (.at(0)),
//  and a best-path descent where every child is NaN (an infinite loop in ROS 1).
// =============================================================================
#ifndef IPP_BASELINES_SEARCH_PLANNER_MCTS_SEARCH_HPP
#define IPP_BASELINES_SEARCH_PLANNER_MCTS_SEARCH_HPP

#include <memory>
#include <random>
#include <string>
#include <vector>

#include "ipp_baselines_search_planner/planner_core.hpp"

namespace ipp_baselines_search {

/// MCTSSearch::NodeStuckException
struct NodeStuckException : std::exception {
    const char* what() const noexcept override { return "Chosen action/node was infeasible many times, giving up."; }
};

/// One InfoMapSearch copy: the SearchMap arrays, the weighted sampler and the RNG.
struct InfoMapCopy {
    std::vector<double> map, copyMap, priority;
    std::vector<int> sensorModelId;
    std::discrete_distribution<std::size_t> sampler;
    std::mt19937 gen;
};

struct MctsNode {
    TreeNode t;                  ///< pose, parent, cost, budget_remaining, depth, edge
    std::string name;
    double edgeValue = 0.0, futureValue = 0.0, information = 0.0, nVisits = 0.0;
    double searchInfo = 0.0;     ///< SearchMap search_info (cumulative)
    std::vector<RewardEvaluator::CellUpdate> dPres, dRes;  ///< local_search_map_updates
    std::vector<int> children;
    std::unique_ptr<InfoMapCopy> map;
    double value() const { return edgeValue + futureValue / nVisits; }  ///< get_value
};

class MctsSearchPlanner : public SearchPlanner {
public:
    using SearchPlanner::SearchPlanner;
    const char* name() const override { return "mcts_search"; }

    // for tests
    double ucb1(const MctsNode& n) const;
    int selectChildHighestUcb1(int idx) const;
    const std::vector<MctsNode>& nodes() const { return nodes_; }
    std::vector<MctsNode>& mutableNodes() { return nodes_; }
    double rootVisitsCarried() const { return rootVisits_; }
    /// The primitive action's goal pose from `from` (make_action_node).
    Pose2 primitiveGoal(const Pose2& from, int actionIdx) const;
    int bestLeaf() const;  ///< get_leaf_node_of_best_path

protected:
    bool setupSolve(const Pose2& start, double budget) override;
    bool loopOnce() override;
    void teardown() override;
    std::vector<TreeNode> bestPath() override;
    int treeSize() const override { return static_cast<int>(nodes_.size()); }
    void extras(std::map<std::string, double>& x) const override;

private:
    std::unique_ptr<InfoMapCopy> copyMap(const InfoMapCopy& from) const;
    void expand(int idx);
    int actAndTransition(int fromIdx, int actionIdx);
    bool makeFeasible(int fromIdx, int actionIdx, MctsNode* out);
    double transitionValue(MctsNode& n);
    double rollout(int idx);
    void rolloutAndBackpropagate(int idx);

    InfoMapCopy master_;            ///< the planner's info map (InfoMapSearch) for this solve
    std::vector<MctsNode> nodes_;   ///< [0] = tree_root; rollout nodes are appended and dropped
    std::mt19937 gen_;              ///< MCTSSearch::gen
    int best_ = 0;                  ///< leaf_node_of_best_path (logged only)
    double rootVisits_ = 0.0, rootFuture_ = 0.0, rootEdgeValue_ = 0.0;  ///< tree_root's carried stats
    std::size_t expansions_ = 0, depthMax_ = 0, copies_ = 0;
    bool degenerate_ = false;
};

}  // namespace ipp_baselines_search

#endif  // IPP_BASELINES_SEARCH_PLANNER_MCTS_SEARCH_HPP
