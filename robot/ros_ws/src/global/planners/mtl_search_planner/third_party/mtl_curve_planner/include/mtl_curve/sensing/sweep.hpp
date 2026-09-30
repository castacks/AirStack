// =============================================================================
//  mtl_curve/sensing/sweep.hpp
//
//  The cross-track sweep an agent at altitude h can fly - the port of
//  computeSweepParams.m.
//
//  Single-axis gimbal on a mount tilted tau toward the forward horizon.
//  Rotating the gimbal by alpha about the tilted roll axis puts the boresight on
//  the ground at
//      along = h tan(tau)                 (a fixed stand-off)
//      cross = h tan(alpha) / cos(tau)    slant range h / (cos(tau) cos(alpha))
//  i.e. it sweeps ONE cross-track line - the geometry cpp_planner's gimbal
//  scheduler assumes.  alphaMax is the smallest of the slant-range limit
//  (rangeMargin * beta), the gimbal travel and the ground reach; the frequency
//  is capped so the peak slew stays inside gimbalRate (5% margin).
// =============================================================================
#ifndef MTLC_SENSING_SWEEP_HPP
#define MTLC_SENSING_SWEEP_HPP

#include "mtl_curve/params.hpp"
#include "mtl_curve/types.hpp"

namespace mtl::curve::sensing {

/// @throws std::invalid_argument when the boresight is out of range even at alpha = 0.
SweepParams computeSweepParams(double h, const SweepOptions& sweep, const GimbalParams& gimbal,
                               const SensorModelParams& sensor, double tiltAngle);

}  // namespace mtl::curve::sensing

#endif  // MTLC_SENSING_SWEEP_HPP
