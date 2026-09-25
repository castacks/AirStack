// =============================================================================
//  mtl/core/dubins.hpp
//
//  Dubins paths: the shortest curvature-bounded path between two planar poses.
//  This replaces MATLAB's Navigation Toolbox dubinsConnection/interpolate, and
//  is the only geometry the budgeted planner ever measures a route with - the
//  same builder that produces the flown track produces the number the budget is
//  enforced on, so there is no second model to drift out of sync.
//
//  All six word types are enumerated (LSL RSR LSR RSL RLR LRL) and the shortest
//  valid one wins, which is what dubinsConnection does.
// =============================================================================
#ifndef MTL_CORE_DUBINS_HPP
#define MTL_CORE_DUBINS_HPP

#include <array>
#include <string>

#include "mtl/params.hpp"
#include "mtl/types.hpp"

namespace mtl::core {

/// A pose on the ground plane: position and heading.
struct Pose2 {
    Vec2   xy    = Vec2::Zero();
    double theta = 0.0;
};

enum class DubinsWord { LSL, LSR, RSL, RSR, RLR, LRL, Invalid };

/// One solved Dubins connection.  Segment lengths are in metres (the turning
/// radius is folded in), so `length` is the arc length the aircraft flies.
class DubinsPath {
public:
    DubinsPath() = default;

    /// Shortest Dubins path from `from` to `to` with the given turn radius.
    /// Always succeeds for a positive radius: at least one word is valid.
    static DubinsPath connect(const Pose2& from, const Pose2& to, double minTurnRadius);

    double     length() const { return length_; }
    DubinsWord word() const { return word_; }
    bool       valid() const { return word_ != DubinsWord::Invalid; }

    /// Pose at arc length s in [0, length()].  Values outside are clamped.
    Pose2 interpolate(double s) const;

    /// Sample the path every `stepSize` metres, always including the endpoint.
    Path2 sample(double stepSize) const;

    std::string wordName() const;

private:
    Pose2                 start_{};
    double                radius_ = 1.0;
    std::array<double, 3> seg_{{0.0, 0.0, 0.0}};  ///< segment lengths [m]
    DubinsWord            word_   = DubinsWord::Invalid;
    double                length_ = kInf;
};

// -----------------------------------------------------------------------------
/// Sampled Dubins path through a waypoint list, plus its cumulative arc length.
///
/// The heading at each waypoint is the BISECTOR of the incoming and outgoing
/// legs.  Pointing the heading at the next waypoint instead (the obvious
/// heuristic) forces each segment to ARRIVE already aimed at the waypoint after
/// it, and where the route turns sharply the only way to meet that with a
/// bounded radius is a full 360 loop just before the waypoint.  Splitting the
/// turn evenly between the two legs removes them: each segment then turns by at
/// most half the corner angle.
///
/// Duplicate samples are dropped, so `cumulativeDist` is strictly increasing and
/// usable as an interpolation abscissa.
///
/// @param waypoints      n-by-2 waypoint list (row 0 is the launch point)
/// @param minTurnRadius  [m]
/// @param opts           sampling resolution
/// @param[out] track     the sampled ground track
/// @param[out] arc       cumulative arc length, arc(0) = 0
// -----------------------------------------------------------------------------
void computeDubinsWaypoints(const Path2& waypoints, double minTurnRadius,
                            const DubinsParams& opts, Path2& track, VecX& arc);

/// Arc length of the Dubins path through `waypoints`, without keeping the track.
double dubinsLength(const Path2& waypoints, double minTurnRadius, const DubinsParams& opts);

/// Sampled Dubins path between two poses (used by the lane router).
Path2 dubinsSegment(const Pose2& from, const Pose2& to, double minTurnRadius, double stepSize);

}  // namespace mtl::core

#endif  // MTL_CORE_DUBINS_HPP
