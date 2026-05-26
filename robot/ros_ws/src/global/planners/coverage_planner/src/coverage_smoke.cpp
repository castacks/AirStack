#include "coverage_planner_logic.hpp"
#include <cmath>
#include <iostream>
#include <string>

namespace {

constexpr double kTol = 1e-6;

bool near_angle(double a, double b) {
  return std::fabs(std::atan2(std::sin(a - b), std::cos(a - b))) < 1e-6;
}

bool fail(const std::string &message) {
  std::cerr << message << "\n";
  return false;
}

bool contains(const std::string &text, const std::string &pattern) {
  return text.find(pattern) != std::string::npos;
}

bool path_stays_in_box(const std::vector<coverage_planner::Waypoint> &path,
                       double x_min, double x_max, double y_min,
                       double y_max) {
  for (std::size_t i = 0; i < path.size(); ++i) {
    const auto &wp = path[i];
    if (wp.x < x_min - kTol || wp.x > x_max + kTol ||
        wp.y < y_min - kTol || wp.y > y_max + kTol) {
      return fail("waypoint outside coverage box");
    }
    if (i == 0) continue;
    const auto &prev = path[i - 1];
    for (int s = 1; s <= 20; ++s) {
      const double t = static_cast<double>(s) / 20.0;
      const double x = prev.x + t * (wp.x - prev.x);
      const double y = prev.y + t * (wp.y - prev.y);
      if (x < x_min - kTol || x > x_max + kTol ||
          y < y_min - kTol || y > y_max + kTol) {
        return fail("path segment leaves coverage box");
      }
    }
  }
  return true;
}

bool check_containment_validator() {
  const coverage_planner::Polygon2D concave = {
      {0.0, 0.0}, {6.0, 0.0}, {6.0, 6.0}, {4.0, 6.0},
      {4.0, 2.0}, {2.0, 2.0}, {2.0, 6.0}, {0.0, 6.0}};

  const std::vector<coverage_planner::Waypoint> valid_path = {
      {1.0, 1.0, 5.0, 0.0}, {5.0, 1.0, 5.0, 0.0},
      {5.0, 5.0, 5.0, 0.0}};
  if (!coverage_planner::path_containment_error(valid_path, concave).empty()) {
    return fail("valid concave-polygon path was reported outside");
  }

  const std::vector<coverage_planner::Waypoint> outside_waypoint = {
      {1.0, 1.0, 5.0, 0.0}, {7.0, 1.0, 5.0, 0.0}};
  std::string error =
      coverage_planner::path_containment_error(outside_waypoint, concave);
  if (!contains(error, "waypoint 1")) {
    return fail("outside waypoint was not reported");
  }

  const std::vector<coverage_planner::Waypoint> crossing_segment = {
      {1.0, 4.0, 5.0, 0.0}, {5.0, 4.0, 5.0, 0.0}};
  error = coverage_planner::path_containment_error(crossing_segment, concave);
  if (!contains(error, "segment 0->1")) {
    return fail("out-of-polygon segment was not reported");
  }

  return true;
}

bool check_face_path_yaw(
    const std::vector<coverage_planner::Waypoint> &path) {
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double expected =
        std::atan2(path[i].y - path[i - 1].y, path[i].x - path[i - 1].x);
    if (!near_angle(path[i - 1].yaw, expected)) {
      return fail("face-path yaw does not match segment tangent");
    }
  }
  return true;
}

bool check_aligned_to_sweep_yaw(
    const std::vector<coverage_planner::Waypoint> &path, double heading_rad) {
  const double ch = std::cos(heading_rad);
  const double sh = std::sin(heading_rad);
  for (std::size_t i = 1; i < path.size(); ++i) {
    const double tangent =
        std::atan2(path[i].y - path[i - 1].y, path[i].x - path[i - 1].x);
    const bool forward = ch * std::cos(tangent) + sh * std::sin(tangent) >= 0.0;
    const double expected = forward ? heading_rad : heading_rad + M_PI;
    if (!near_angle(path[i - 1].yaw, expected)) {
      return fail("aligned-to-sweep yaw is not parallel to sweep heading");
    }
  }
  return true;
}

bool check_fixed_yaw(const std::vector<coverage_planner::Waypoint> &path,
                     double fixed_rad) {
  for (const auto &wp : path) {
    if (!near_angle(wp.yaw, fixed_rad)) {
      return fail("fixed yaw mode produced a non-fixed waypoint yaw");
    }
  }
  return true;
}

}  // namespace

int main() {
  coverage_planner::Polygon2D poly = {
      {-20, -20}, {20, -20}, {20, 20}, {-20, 20}};
  coverage_planner::Point2D start{0, 0};
  coverage_planner::CoverageParams p;
  p.line_spacing_m = 5.0;
  p.heading_deg = 0.0;
  p.altitude_m = 5.0;
  p.start_from_nearest = true;
  p.boundary_inset_m = 0.0;

  auto path = coverage_planner::generate_coverage_path(poly, start, p);
  if (!path || path->empty()) {
    std::cerr << "plan failed\n";
    return 1;
  }
  if (!path_stays_in_box(*path, -20.0, 20.0, -20.0, 20.0))
    return 1;
  if (!coverage_planner::path_containment_error(*path, poly).empty())
    return 1;
  if (!check_face_path_yaw(*path))
    return 1;
  if (!check_containment_validator())
    return 1;

  p.orientation_mode = coverage_planner::CoverageParams::kOrientAlignedToSweep;
  path = coverage_planner::generate_coverage_path(poly, start, p);
  if (!path || !check_aligned_to_sweep_yaw(*path, 0.0))
    return 1;

  p.orientation_mode = coverage_planner::CoverageParams::kOrientFixed;
  p.orientation_deg = 90.0;
  path = coverage_planner::generate_coverage_path(poly, start, p);
  if (!path || !check_fixed_yaw(*path, M_PI / 2.0))
    return 1;

  p.heading_deg = 45.0;
  p.orientation_mode = coverage_planner::CoverageParams::kOrientFacePath;
  path = coverage_planner::generate_coverage_path(poly, start, p);
  if (!path || !path_stays_in_box(*path, -20.0, 20.0, -20.0, 20.0) ||
      !coverage_planner::path_containment_error(*path, poly).empty() ||
      !check_face_path_yaw(*path))
    return 1;

  p.orientation_mode = coverage_planner::CoverageParams::kOrientAlignedToSweep;
  path = coverage_planner::generate_coverage_path(poly, start, p);
  if (!path || !path_stays_in_box(*path, -20.0, 20.0, -20.0, 20.0) ||
      !coverage_planner::path_containment_error(*path, poly).empty() ||
      !check_aligned_to_sweep_yaw(*path, M_PI / 4.0))
    return 1;

  std::cout << "waypoints: " << path->size() << "\n";
  std::cout << "length_m: " << coverage_planner::path_length(*path) << "\n";
  return 0;
}
