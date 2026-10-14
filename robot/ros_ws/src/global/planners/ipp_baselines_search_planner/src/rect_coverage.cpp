// =============================================================================
//  rect_coverage.cpp — CoveragePlanner.h (see rect_coverage.hpp).
// =============================================================================
#include "ipp_baselines_search_planner/rect_coverage.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace ipp_baselines_search {

namespace {
constexpr double kPi = 3.14159265358979323846;
double wrapTo2Pi(double a) { return mod2pi(a); }  // ang::WrapTo2Pi
}  // namespace

RectangleCoverage::RectangleCoverage(std::vector<double> sensorOffset, double sensorWidth, double turnRadius,
                                     bool enforceTurnCheck)
    : off_(std::move(sensorOffset)), width_(sensorWidth), turnRadius_(turnRadius) {
    if (enforceTurnCheck && turnRadius_ * 2 < width_) {
        throw std::invalid_argument("Turn radius must two times greater than sensor width");
    }
}

std::vector<std::vector<double>> RectangleCoverage::rectCoverage(const std::vector<std::vector<double>>& c) const {
    std::vector<std::vector<double>> path;
    const double first = std::sqrt(std::pow(c[1][0] - c[0][0], 2) + std::pow(c[1][1] - c[0][1], 2));
    const double second = std::sqrt(std::pow(c[2][0] - c[1][0], 2) + std::pow(c[2][1] - c[1][1], 2));
    double psiF, ux, uy, x1, y1, x2, y2;
    int loops;
    if (first > second) {
        psiF = wrapTo2Pi(std::atan2(c[1][1] - c[0][1], c[1][0] - c[0][0]));
        ux = (c[2][0] - c[1][0]) / second;
        uy = (c[2][1] - c[1][1]) / second;
        x1 = c[0][0] + width_ * 0.5 * ux;
        y1 = c[0][1] + width_ * 0.5 * uy;
        x2 = c[1][0] + width_ * 0.5 * ux;
        y2 = c[1][1] + width_ * 0.5 * uy;
        loops = static_cast<int>(std::floor(second / width_));
    } else {
        psiF = wrapTo2Pi(std::atan2(c[2][1] - c[1][1], c[2][0] - c[1][0]));
        ux = (c[1][0] - c[0][0]) / first;
        uy = (c[1][1] - c[0][1]) / first;
        x1 = c[0][0] + width_ * 0.5 * ux;
        y1 = c[0][1] + width_ * 0.5 * uy;
        x2 = c[3][0] + width_ * 0.5 * ux;
        y2 = c[3][1] + width_ * 0.5 * uy;
        loops = static_cast<int>(std::floor(first / width_));
    }
    const double psiB = wrapTo2Pi(psiF + kPi);
    const double xf = off_[0] * std::cos(psiF) - off_[1] * std::sin(psiF);
    const double yf = off_[0] * std::sin(psiF) + off_[1] * std::cos(psiF);
    const double xb = off_[0] * std::cos(psiB) - off_[1] * std::sin(psiB);
    const double yb = off_[0] * std::sin(psiB) + off_[1] * std::cos(psiB);
    bool forward = true;
    for (int i = 0; i < loops; ++i) {
        if (forward) {
            path.push_back({x1 + xf, y1 + yf, off_[2], psiF});
            path.push_back({x2 + xf, y2 + yf, off_[2], psiF});
            forward = false;
        } else {
            path.push_back({x2 + xb, y2 + yb, off_[2], psiB});
            path.push_back({x1 + xb, y1 + yb, off_[2], psiB});
            forward = true;
        }
        x1 += width_ * ux;
        y1 += width_ * uy;
        x2 += width_ * ux;
        y2 += width_ * uy;
    }
    return path;
}

double RectangleCoverage::rectCoverageLength(const std::vector<std::vector<double>>& c) const {
    const double first = std::sqrt(std::pow(c[1][0] - c[0][0], 2) + std::pow(c[1][1] - c[0][1], 2));
    const double second = std::sqrt(std::pow(c[2][0] - c[1][0], 2) + std::pow(c[2][1] - c[1][1], 2));
    if (first > second) return std::floor(second / width_) * first;
    return std::floor(first / width_) * second;
}

double RectCoveragePlanner::xDelta() const {
    return s_.altitude * std::tan(s_.camera.tilt - p_.viewPointGoal * s_.camera.fov / 2.0);
}

double RectCoveragePlanner::sensorWidth() const {
    const double xd = xDelta();
    const double distance = std::sqrt(xd * xd + s_.altitude * s_.altitude);
    return std::tan(s_.camera.fov / 2.0) * distance * 2.0;
}

bool RectCoveragePlanner::setupSolve(const Pose2& start, double /*budget*/) {
    path_.clear();
    lanes_ = 0;
    path_.push_back({start.x, start.y, s_.altitude, start.yaw});
    return true;
}

bool RectCoveragePlanner::loopOnce() {
    const double xd = xDelta();
    const double width = sensorWidth();
    const RectangleCoverage rc({-xd, 0.0, s_.altitude}, width, s_.turnRadius, p_.coverageEnforceTurnCheck);
    const std::vector<std::vector<double>> corners = {
        {s_.xMin, s_.yMin}, {s_.xMax, s_.yMin}, {s_.xMax, s_.yMax}, {s_.xMin, s_.yMax}};
    const auto lawn = rc.rectCoverage(corners);
    lanes_ = static_cast<int>(lawn.size() / 2);
    for (const auto& w : lawn) path_.push_back(w);
    return false;
}

std::vector<TreeNode> RectCoveragePlanner::bestPath() {
    std::vector<TreeNode> out;
    if (path_.empty()) return out;
    TreeNode root;
    root.pose = {path_[0][0], path_[0][1], wrapPi(path_[0][3])};
    root.budgetRemaining = budget_;
    root.edge.q0 = root.pose;
    root.edge.rho = s_.turnRadius;
    out.push_back(root);
    const double step = std::max(s_.dubinsStep, 1e-3);
    for (std::size_t i = 1; i < path_.size(); ++i) {
        const TreeNode& prev = out.back();
        TreeNode n;
        n.parent = static_cast<int>(out.size()) - 1;
        n.depth = prev.depth + 1;
        const Pose2 goal{path_[i][0], path_[i][1], wrapPi(path_[i][3])};
        DubinsPath dp;
        dubinsShortest(prev.pose, goal, s_.turnRadius, &dp);  // `valid` unused, as in the original
        const double L = dp.length();
        // straight-edge poses for the reward yardstick (the coverage planner itself scores nothing)
        EdgeTracker et;
        const int nSteps = L > 0.0 ? std::max(1, static_cast<int>(std::ceil(L / step - 1e-9))) : 0;
        Pose2 cur = prev.pose;
        double curS = 0.0;
        for (int k = 1; k <= nSteps; ++k) {
            const double sv = std::min(L, k * step);
            const Pose2 next = dp.sample(sv);
            et.update(cur, curS, next);
            cur = next;
            curS = sv;
        }
        et.writeTo(&n);
        n.pose = L > 0.0 ? dp.sample(L) : prev.pose;
        n.edge = dp;
        n.edgeLen = L;
        n.arc = prev.arc + L;
        n.cost = prev.cost + L;
        n.budgetRemaining = budget_ - n.cost;
        out.push_back(n);
    }
    return out;
}

void RectCoveragePlanner::extras(std::map<std::string, double>& x) const {
    x["sensor_width_m"] = sensorWidth();
    x["x_delta_m"] = xDelta();
    x["lanes"] = lanes_;
    x["way_points"] = static_cast<double>(path_.size());
}

}  // namespace ipp_baselines_search
