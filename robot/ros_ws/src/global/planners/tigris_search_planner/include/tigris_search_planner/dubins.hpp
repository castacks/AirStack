// =============================================================================
//  tigris_search_planner/dubins.hpp — shortest Dubins paths (Dubins 1957; the
//  closed forms of A. Walker's dubins.c). TIGRIS steers with trochoids; with no
//  wind (the AirStack case) a trochoid IS a Dubins path, so this is the same
//  steering function without the wind terms.
// =============================================================================
#ifndef TIGRIS_SEARCH_PLANNER_DUBINS_HPP
#define TIGRIS_SEARCH_PLANNER_DUBINS_HPP

namespace tigris_search {

struct Pose2 {
    double x = 0.0, y = 0.0, yaw = 0.0;  ///< world ENU [m], ENU yaw [rad]
};

double wrapPi(double a);
double mod2pi(double a);

struct DubinsPath {
    enum Word { LSL = 0, LSR, RSL, RSR, RLR, LRL };
    Pose2  q0;
    double rho = 1.0;
    int    word = LSL;
    double seg[3] = {0.0, 0.0, 0.0};  ///< normalised segment lengths (multiply by rho for metres)

    double length() const { return (seg[0] + seg[1] + seg[2]) * rho; }
    /// Pose after `s` metres along the path (clamped to [0, length()]).
    Pose2 sample(double s) const;
};

/// Shortest of the six words from a to b with turn radius rho. Returns false
/// only for degenerate input (rho <= 0).
bool dubinsShortest(const Pose2& a, const Pose2& b, double rho, DubinsPath* out);

}  // namespace tigris_search

#endif  // TIGRIS_SEARCH_PLANNER_DUBINS_HPP
