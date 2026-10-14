// =============================================================================
//  ipp_baselines_search_planner/greedy.hpp — GreedySearchPlanner
//  (ia-tigris/src/ipp_planners/include/ipp_planners/GreedySearchPlanner.h), as written.
//
//  replan_setup (49-87): the working map = the planning belief after the root's own
//    footprint (search_information_gain on the root).
//  replan_loop (151-215), one view point per call:
//    1. every cell, x-major (row = x outer, column = y inner): score = one-look reward at the
//       NOMINAL range h / sin(pitch + vg vfov / 2) on the working map (calc_reward_and_belief x
//       priority) divided by the length of the path from the current pose to the cell's
//       viewing point (find_viewing_point); only cells whose viewing point differs from the
//       pose in x AND in y; the first strict maximum above 0 wins (89-149);
//    2. a node at that viewing point, child of the previous one; walk the path setting the
//       straight-edge poses, then stop counting when total + d > budget - one step; the WHOLE
//       segment is kept (keepBefore is commented out) and the solve ends;
//    3. the working map = the planning belief after the full re-walk of the chain from the
//       root (search_information_gain: per node its footprint, THEN its edge).
//  get_best_path_segments: the segments in order.
//  MATCHED reward: the cell score is residual x P(nominal range) and the chain update is the
//  looks along each segment, so the planner chases the logger's metric with the same rule.
// =============================================================================
#ifndef IPP_BASELINES_SEARCH_PLANNER_GREEDY_HPP
#define IPP_BASELINES_SEARCH_PLANNER_GREEDY_HPP

#include "ipp_baselines_search_planner/planner_core.hpp"

namespace ipp_baselines_search {

class GreedyPlanner : public SearchPlanner {
public:
    using SearchPlanner::SearchPlanner;
    const char* name() const override { return "greedy"; }

    /// The cell the last replan_loop picked (grid index; -1 if none) and its score.
    int lastCell() const { return lastCell_; }
    double lastScore() const { return lastScore_; }
    const BeliefState& workingMap() const { return work_; }

protected:
    bool setupSolve(const Pose2& start, double budget) override;
    bool loopOnce() override;
    std::vector<TreeNode> bestPath() override { return chain_; }
    int treeSize() const override { return static_cast<int>(chain_.size()); }
    void extras(std::map<std::string, double>& x) const override;

private:
    void rewalk();  // search_information_gain on the last chain node: copy_map = map + chain looks

    std::vector<TreeNode> chain_;
    BeliefState work_;   // local_map.copy_map (and its matched counterpart)
    Pose2 pose_;         // pose_to_plan_from
    double total_ = 0.0; // total_path_length
    double rNom_ = 0.0;
    int lastCell_ = -1;
    double lastScore_ = 0.0;
    bool noCell_ = false;
};

}  // namespace ipp_baselines_search

#endif  // IPP_BASELINES_SEARCH_PLANNER_GREEDY_HPP
