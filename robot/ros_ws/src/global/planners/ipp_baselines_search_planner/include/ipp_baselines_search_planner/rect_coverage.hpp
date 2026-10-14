// =============================================================================
//  ipp_baselines_search_planner/rect_coverage.hpp — the `rect_coverage` planner:
//  CoveragePlanner + RectangleCoverage (ia-tigris/src/ipp_planners/include/ipp_planners/
//  CoveragePlanner.h), as written.
//
//  replan_setup (187-207): path = [start]; the area's bounding box.
//  replan_loop (209-228), once (returns false):
//    x_delta = h / tan(pitch + vg vfov / 2)          -> with the cone: h tan(tilt - vg fov / 2)
//    width   = 2 tan(hfov / 2) sqrt(x_delta^2 + h^2)  -> with the cone: 2 tan(fov / 2) h / cos(...)
//    (the cone's diameter at the aim point; modification 1). RectangleCoverage(
//    {-x_delta, 0, h}, width, R) throws when 2 R < width (27-30): kept behind
//    rect_coverage_enforce_turn_check, default OFF (with this mission 2 R = 24 m << 354 m).
//    Lawnmower over the corners {(xmin,ymin),(xmax,ymin),(xmax,ymax),(xmin,ymax)} (44-135):
//    if the first edge is longer, lanes run along it; otherwise (a square too) lanes run along
//    corner 1 -> 2 (+y) and step along +x; floor(edge / width) lanes, the first at width / 2;
//    each end point shifted by the sensor offset rotated into the lane heading; alternating. Always starts at lane 1 of corner 0; ignores the
//    budget (the receding glue cuts the published track at the budget).
//  get_best_path_segments (230-251): a Dubins path between consecutive way points.
// =============================================================================
#ifndef IPP_BASELINES_SEARCH_PLANNER_RECT_COVERAGE_HPP
#define IPP_BASELINES_SEARCH_PLANNER_RECT_COVERAGE_HPP

#include <vector>

#include "ipp_baselines_search_planner/planner_core.hpp"

namespace ipp_baselines_search {

/// RectangleCoverage (CoveragePlanner.h:12-157). Way points are {x, y, z, psi}.
class RectangleCoverage {
public:
    RectangleCoverage(std::vector<double> sensorOffset, double sensorWidth, double turnRadius, bool enforceTurnCheck);
    std::vector<std::vector<double>> rectCoverage(const std::vector<std::vector<double>>& corners) const;
    double rectCoverageLength(const std::vector<std::vector<double>>& corners) const;

private:
    std::vector<double> off_;
    double width_, turnRadius_;
};

class RectCoveragePlanner : public SearchPlanner {
public:
    using SearchPlanner::SearchPlanner;
    const char* name() const override { return "rect_coverage"; }

    double sensorWidth() const;  ///< the cone width at the aim point
    double xDelta() const;       ///< along-track offset of the aircraft behind the aim line
    const std::vector<std::vector<double>>& wayPoints() const { return path_; }

protected:
    bool setupSolve(const Pose2& start, double budget) override;
    bool loopOnce() override;
    std::vector<TreeNode> bestPath() override;
    int treeSize() const override { return static_cast<int>(path_.size()); }
    void extras(std::map<std::string, double>& x) const override;

private:
    std::vector<std::vector<double>> path_;  // planned_path
    int lanes_ = 0;
};

}  // namespace ipp_baselines_search

#endif  // IPP_BASELINES_SEARCH_PLANNER_RECT_COVERAGE_HPP
