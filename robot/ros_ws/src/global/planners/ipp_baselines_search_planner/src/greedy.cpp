// =============================================================================
//  greedy.cpp — GreedySearchPlanner.h (see greedy.hpp).
// =============================================================================
#include "ipp_baselines_search_planner/greedy.hpp"

#include <algorithm>
#include <cmath>

namespace ipp_baselines_search {

// SearchMap::search_information_gain(node, include_edge = true): copy_map = map, then the
// chain root -> node, each node's footprint THEN its edge, written into copy_map
void GreedyPlanner::rewalk() {
    const bool original = p_.reward.mode == RewardMode::ORIGINAL;
    work_ = *belief_;
    eval_.begin(*belief_);
    for (const TreeNode& n : chain_) {
        if (original) originalNodeGain(eval_, n, startTime_, /*edgeFirst=*/false, /*includeEdge=*/true);
        else matchedNodeGain(eval_, n, startTime_);
    }
    eval_.commit(work_);
}

bool GreedyPlanner::setupSolve(const Pose2& start, double budget) {
    pose_ = start;
    total_ = 0.0;
    chain_.clear();
    lastCell_ = -1;
    lastScore_ = 0.0;
    noCell_ = false;
    rNom_ = nominalRange();
    TreeNode root;
    root.pose = start;
    root.budgetRemaining = budget;
    root.edge.q0 = start;
    root.edge.rho = s_.turnRadius;
    chain_.push_back(root);
    rewalk();  // double edge_reward = local_map.search_information_gain(node, true, desired_speed)
    return true;
}

bool GreedyPlanner::loopOnce() {
    const bool original = p_.reward.mode == RewardMode::ORIGINAL;
    const double pNom = s_.det.prob(rNom_);
    // get_greedy_view_point
    double best = 0.0;
    int bestK = -1;
    Pose2 bestVp;
    DubinsPath bestDp;
    double bestLen = 0.0;
    for (int j = 0; j < g_.nx; ++j) {        // row (x)
        for (int i = 0; i < g_.ny; ++i) {    // column (y)
            // check_cell_inside_convex_poly(row, col, search_bounds, true): all four corners in
            // the bounds (int-cast corners, inclusive test); the grid tiles the area
            const double x0 = g_.xMin + j * g_.res, y0 = g_.yMin + i * g_.res;
            const double xs[2] = {x0, x0 + g_.res}, ys[2] = {y0, y0 + g_.res};
            bool inside = true;
            for (const double px : xs) {
                for (const double py : ys) {
                    const double ix = static_cast<int>(px), iy = static_cast<int>(py);
                    if (ix < s_.xMin || ix > s_.xMax || iy < s_.yMin || iy > s_.yMax) inside = false;
                }
            }
            if (!inside) continue;
            const std::size_t k = static_cast<std::size_t>(i * g_.nx + j);
            double score;
            if (original) {
                double q;
                score = originalCellUpdate(work_.presence[k], rNom_, s_.det, p_.reward, &q);  // x priority 1
            } else {
                score = work_.residual[k] * pNom;
            }
            const Pose2 vp = findViewingPoint(pose_, g_.cx(j), g_.cy(i));
            if (vp.x != pose_.x && vp.y != pose_.y) {  // (sic) && : evaluated only if both differ
                DubinsPath dp;
                if (!dubinsShortest(pose_, vp, s_.turnRadius, &dp)) continue;
                const double len = dp.length();  // trochoids::get_length
                const double ratio = score / len;
                if (ratio > best) {
                    best = ratio;
                    bestK = static_cast<int>(k);
                    bestVp = vp;
                    bestDp = dp;
                    bestLen = len;
                }
            }
        }
    }
    if (bestK < 0) {
        // ROS 1 indexes an empty view point here (undefined behaviour): end the solve
        noCell_ = true;
        return false;
    }
    lastCell_ = bestK;
    lastScore_ = best;
    // the node: cost = total + whole trochoid, parent = the previous node
    TreeNode n;
    n.pose = bestVp;
    n.parent = static_cast<int>(chain_.size()) - 1;
    n.depth = chain_.back().depth + 1;
    n.cost = total_ + bestLen;
    n.edge = bestDp;
    n.edgeLen = bestLen;
    n.arc = chain_.back().arc + bestLen;
    // append the trochoid such that we don't violate the budget
    const double step = std::max(s_.dubinsStep, 1e-3);
    const int nSteps = bestLen > 0.0 ? std::max(1, static_cast<int>(std::ceil(bestLen / step - 1e-9))) : 0;
    EdgeTracker et;
    bool budgetRemaining = true;
    Pose2 cur = pose_;
    double curS = 0.0;
    for (int i = 1; i <= nSteps; ++i) {
        const double sv = std::min(bestLen, i * step);
        const Pose2 next = bestDp.sample(sv);
        const double d = std::hypot(next.x - cur.x, next.y - cur.y);
        et.update(cur, curS, next);  // set before the budget test (189-192)
        if (total_ + d > budget_ - step) {  // budget - final_path_discretization_distance
            budgetRemaining = false;       // keepBefore commented out: the whole segment stays
            break;
        }
        total_ += d;
        cur = next;
        curS = sv;
    }
    et.writeTo(&n);
    n.budgetRemaining = budget_ - total_;
    chain_.push_back(n);
    pose_ = cur;  // next_state = current_state on the budget break, else the last state
    rewalk();     // local_map.search_information_gain(node, true, desired_speed)
    return budgetRemaining;
}

void GreedyPlanner::extras(std::map<std::string, double>& x) const {
    x["view_points"] = static_cast<double>(chain_.size()) - 1.0;
    x["nominal_range_m"] = rNom_;
    x["planned_length_m"] = chain_.empty() ? 0.0 : chain_.back().arc;
    x["counted_length_m"] = total_;
    x["no_positive_cell"] = noCell_ ? 1.0 : 0.0;
}

}  // namespace ipp_baselines_search
