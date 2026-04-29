#include "coverage_planner_logic.hpp"
#include <cmath>
#include <iostream>

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
  std::cout << "waypoints: " << path->size() << "\n";
  std::cout << "length_m: " << coverage_planner::path_length(*path) << "\n";
  return 0;
}