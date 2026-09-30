// =============================================================================
//  mtl_curve/trajectory/curve_trajectory.hpp
//
//  Timed aircraft states and boresights from the optimised curves - the port
//  of generateCurveTrajectories.m (the plan's PHASE 6).  Per agent:
//    1. resample gamma at uniform arc length ds = V*dt;
//    2. drone = [x y h_a];
//    3. rpy from computeAirframeRPY (coordinated-turn bank, level pitch);
//    4. sensor: the single-axis gimbal sweeps its cross-track line
//           alpha(t) = alphaMax sin(2 pi freq t + phase_a)
//           s(t)     = gamma + h tan(tau) t + (h tan(alpha)/cos(tau)) n
//       the gimbal is assumed to take out the bank, so it is commanded to
//       alpha + roll (peak reported against gimbalMax / maxCrossAngle);
//    5. every agent padded with its final state to the longest step count.
// =============================================================================
#ifndef MTLC_TRAJECTORY_CURVE_TRAJECTORY_HPP
#define MTLC_TRAJECTORY_CURVE_TRAJECTORY_HPP

#include <vector>

#include "mtl_curve/params.hpp"
#include "mtl_curve/types.hpp"

namespace mtl::curve::trajectory {

/// @param sweepPhase per-agent phase offsets [rad] (empty = 0)
std::vector<AgentTrajectory> generateCurveTrajectories(const std::vector<VecX>& curves,
                                                       const std::vector<CurveRep>& reps,
                                                       const std::vector<double>& altitudes,
                                                       const std::vector<SweepParams>& sweeps, double V,
                                                       double dt, const GimbalParams& gimbal, VecX& timeVec,
                                                       const std::vector<double>& sweepPhase = {});

}  // namespace mtl::curve::trajectory

#endif  // MTLC_TRAJECTORY_CURVE_TRAJECTORY_HPP
