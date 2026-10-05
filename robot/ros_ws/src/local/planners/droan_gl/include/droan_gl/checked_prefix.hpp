#pragma once
#include <droan_gl/gl_interface.hpp>
#include <cmath>
#include <stdexcept>
#include <limits>
#include <optional>

class SingletonGoalStop {
 public:
  void bind(size_t authored_waypoints, const tf2::Vector3& map_goal,
            const tf2::Vector3* physical_start = nullptr) {
    goal_.reset();
    height_stop_ = false;
    if (authored_waypoints == 1) {
      goal_ = map_goal;
      if (physical_start) {
        const auto delta = map_goal - *physical_start;
        const double xy = std::hypot(delta.x(), delta.y());
        height_stop_ = std::isfinite(xy) && std::isfinite(delta.z()) &&
            xy <= .05 && std::abs(delta.z()) > 2. * xy + 1e-6;
      }
    }
  }
  void route_replaced() { goal_.reset(); height_stop_ = false; }
  std::optional<tf2::Vector3> goal() const { return goal_; }
  bool height_stop() const { return height_stop_; }
 private:
  std::optional<tf2::Vector3> goal_;
  bool height_stop_ = false;
};

// Ordered diagnostic classification of the existing commanded-anchor gate.
inline const char* commanded_anchor_rejection(
    bool valid, bool map_frame, double source_age, double receipt_age, bool finite_xy)
{
  if (!valid) return "missing";
  if (!map_frame) return "frame";
  if (!std::isfinite(source_age)) return "nonfinite_source_age";
  if (source_age < 0.) return "source_future";
  if (source_age > 0.5) return "source_stale";
  if (receipt_age < 0.) return "receipt_future";
  if (receipt_age > 0.5) return "receipt_stale";
  if (!finite_xy) return "nonfinite_xy";
  return nullptr;
}

// Same predicate for publication and observational rejection diagnostics.
inline const char* checked_point_rejection(
    const TrajectoryPoint& p, double min_z, double max_z)
{
  if (!std::isfinite(p.v1.x) || !std::isfinite(p.v1.y) || !std::isfinite(p.v1.z) ||
      !std::isfinite(p.v2.x) || !std::isfinite(p.v2.y) || !std::isfinite(p.v2.z) ||
      !std::isfinite(p.v2.w)) return "nonfinite";
  if (p.v2.w < 0.) return "negative_speed";
  if (p.v1.z < min_z || p.v1.z > max_z) return "altitude";
  if (!(p.v2.z > 1.f)) return "unobserved";
  if (p.v2.x > 0.f && p.v2.x > p.v2.z) return "collision";
  return nullptr;
}

inline std::pair<double, double> navigation_start_corridor(
    double route_min, double route_max, double physical_z, double commanded_z)
{
  for (double z : {route_min, route_max, physical_z, commanded_z}) {
    if (!std::isfinite(z)) throw std::invalid_argument("Nonfinite navigation corridor");
  }
  if (route_min > route_max) throw std::invalid_argument("Inverted navigation corridor");
  return {std::min({route_min, physical_z, commanded_z}),
          std::max({route_max, physical_z, commanded_z})};
}

// A nearby singleton goal can lie inside a safe long horizon. Do not penalize
// forward motion for its distant endpoint; stop at the closest existing sample.
inline size_t single_goal_stop_count(
    const std::vector<TrajectoryPoint>& points, const tf2::Vector3& goal, double radius)
{
  if (points.size() < 2 || !std::isfinite(radius) || radius <= 0. ||
      !std::isfinite(goal.x()) || !std::isfinite(goal.y()) || !std::isfinite(goal.z()))
    return points.size();
  size_t nearest = 0;
  double closest = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < points.size(); ++i) {
    const double distance = points[i].position().distance(goal);
    if (distance < closest) { closest = distance; nearest = i; }
  }
  if (closest > radius || nearest == 0) return points.size();
  return nearest + 1;
}

// Receding-horizon segment: never command an unchecked remainder or later free island.
inline std::vector<TrajectoryPoint> checked_prefix(
    const std::vector<TrajectoryPoint>& points, size_t offset, size_t count,
    double min_z, double max_z, double reserve_m,
    const tf2::Vector3* singleton_goal = nullptr, size_t* before_goal_crop_count = nullptr,
    bool allow_goal_height_stop = false)
{
  if (before_goal_crop_count) *before_goal_crop_count = 0;
  std::vector<TrajectoryPoint> result;
  if (!std::isfinite(min_z) || !std::isfinite(max_z) || min_z > max_z ||
      !std::isfinite(reserve_m) || reserve_m <= 0. || offset > points.size() ||
      count > points.size() - offset) return result;
  std::vector<double> distance;
  for (size_t i = 0; i < count; ++i) {
    const auto& p = points[offset + i];
    if (checked_point_rejection(p, min_z, max_z)) break;
    const double traveled = result.empty() ? 0. : distance.back() +
        result.back().position().distance(p.position());
    result.push_back(p); distance.push_back(traveled);
  }
  if (result.size() < 2) return {};
  // A singleton goal plane is an intentional endpoint, not by itself a loss of
  // checked space. Never exempt a crossing that is also unseen/colliding/invalid.
  bool goal_height_stop = false;
  if (allow_goal_height_stop && singleton_goal && result.size() < count) {
    const auto& crossing = points[offset + result.size()];
    const double goal_z = singleton_goal->z();
    const bool goal_plane =
        (std::abs(goal_z - max_z) <= 1e-6 && crossing.v1.z > max_z) ||
        (std::abs(goal_z - min_z) <= 1e-6 && crossing.v1.z < min_z);
    const double infinity = std::numeric_limits<double>::infinity();
    goal_height_stop = goal_plane &&
        checked_point_rejection(crossing, -infinity, infinity) == nullptr &&
        result.back().position().distance(*singleton_goal) <= reserve_m &&
        result.back().position().distance(*singleton_goal) <
            result.front().position().distance(*singleton_goal);
  }
  if (result.size() < count && !goal_height_stop) {
    const double cutoff = distance.back() - reserve_m;
    while (!distance.empty() && distance.back() > cutoff) {
      result.pop_back(); distance.pop_back();
    }
  }
  if (result.size() < 2 || distance.back() <= 1e-6) return {};
  if (before_goal_crop_count) *before_goal_crop_count = result.size();
  if (singleton_goal) {
    const size_t stop_count = single_goal_stop_count(result, *singleton_goal, reserve_m);
    result.resize(stop_count); distance.resize(stop_count);
    if (result.size() < 2 || distance.back() <= 1e-6) return {};
  }
  // Taper the original raw checked speeds once, after all geometric cutoffs.
  for (size_t i = 0; i < result.size(); ++i) {
    const double remaining = distance.back() - distance[i];
    result[i].v2.w *= std::min(1., remaining / reserve_m);
  }
  result.back().v2.w = 0.f;
  return result;
}
