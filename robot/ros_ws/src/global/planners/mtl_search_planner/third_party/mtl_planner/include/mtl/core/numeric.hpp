// =============================================================================
//  mtl/core/numeric.hpp
//
//  The small numeric primitives the MATLAB pipeline leaned on the language for:
//  linear interpolation, moving-average smoothing with MATLAB's conv(...,'same')
//  edge normalisation, phase unwrapping, arc length, headings and polyline
//  resampling.  They are gathered here so every stage uses ONE implementation -
//  the audit in eval/geometry_audit re-derives the geometry from the outputs and
//  only passes if the scheduler and the attitude model agree bit for bit on
//  what "the heading of this track" means.
// =============================================================================
#ifndef MTL_CORE_NUMERIC_HPP
#define MTL_CORE_NUMERIC_HPP

#include <algorithm>
#include <cmath>
#include <vector>

#include "mtl/types.hpp"

namespace mtl::core {

/// Symmetric clamp to [-lim, +lim].
inline double clampAbs(double x, double lim) { return std::max(std::min(x, lim), -lim); }

inline VecX clampAbs(const VecX& x, double lim) {
    return x.cwiseMax(-lim).cwiseMin(lim);
}

/// Wrap an angle to (-pi, pi].
inline double wrapPi(double a) {
    double m = std::fmod(a + kPi, 2.0 * kPi);
    if (m < 0.0) m += 2.0 * kPi;
    return m - kPi;
}

/// Straight-line length of a polyline.
double polylineLength(const Path2& p);
double polylineLength(const Path3& p);

/// Per-sample step length and its cumulative sum; ds(0) = 0, arc(0) = 0.
void arcOf(const Path2& xy, VecX& ds, VecX& arc);

/// MATLAB `conv(x, ones(win)/win, 'same') ./ conv(ones, ...)`: a moving average
/// that does not fade at the ends because the window is renormalised there.
VecX movingAverage(const VecX& x, int window);

/// MATLAB `unwrap`: remove 2*pi jumps larger than pi.
VecX unwrap(const VecX& x);

/// Unwrapped, lightly smoothed heading of a ground track, held through samples
/// where the aircraft did not move.  Shared by the scheduler and the attitude
/// model so both report the same yaw for the same track.
VecX headingOf(const Path2& xy, const VecX& ds);

/// Forward-difference slope dh/ds; the last sample repeats the one before it.
VecX slopeOf(const VecX& h, const VecX& ds);

/// Linear interpolation of samples (x, y) at the query points xq.  `x` must be
/// non-decreasing; queries outside the range are clamped to the end values
/// (MATLAB's interp1 returns NaN there, but every call site in this pipeline
/// queries strictly inside the range, and clamping is the safe rounding).
VecX interp1(const VecX& x, const VecX& y, const VecX& xq);

/// Even arc-length resampling of a polyline to exactly nSteps points.
Path2 resampleAlongPath(const Path2& path, Index nSteps);

/// Sorted-sample quantile, MATLAB-free: x[ceil(p*n)] on the sorted vector.
double quantile(std::vector<double> x, double p);

/// tan(roll) = V*yawRate/g, +ve roll = right wing down: the bank a coordinated
/// turn demands.  Shared by the scheduler and computeAirframeRPY.
VecX coordinatedRoll(const VecX& yaw, const VecX& ds, double dt, double maxRoll,
                     double rollSign, double rollSmoothSec);

/// Closest point to `pt` on the segment A->B.
Vec2 projectPointOnSegment(const Vec2& pt, const Vec2& a, const Vec2& b);

/// C1 interpolant through (knot, value) pairs, exact AT the knots, whose slope
/// never exceeds 1.5x the mean slope of the segment it is in.  Knots are
/// 0-based sample indices and must be strictly increasing.
VecX smoothstepProfile(const std::vector<Index>& knots, const std::vector<double>& values,
                       Index n);

/// y(k) = y(k-1) + clamp(x(k) - y(k-1), rate): a causal rate limiter.
VecX rateLimit(const VecX& x, double rate);

}  // namespace mtl::core

#endif  // MTL_CORE_NUMERIC_HPP
