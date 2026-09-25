// =============================================================================
//  mtl/sensing/airframe.hpp
//
//  Airframe attitude implied by a flown trajectory.
//
//  WHY THIS EXISTS
//  ---------------
//  The attitude of a fixed wing is not a planning decision - it is READ OFF the
//  trajectory.  Yaw is the ground-track heading, bank is whatever a coordinated
//  turn at that speed and turn rate demands, and pitch is the flight-path angle
//  the altitude profile implies.  None of it needs a scheduler.
//
//  That matters for the MULTI-AXIS gimbal.  There the boresight can be pointed
//  along-track and cross-track independently of the airframe, so there is
//  nothing to schedule: the aircraft flies the nominal Dubins path and the
//  gimbal follows the micro-TSP sweep.  Calling the single-axis scheduler just
//  to get three angles back - and then throwing away the trajectory it bent and
//  the pitch it spent - is waste.  This is that one useful output on its own.
//
//  The single-axis solver computes bank with the SAME law from the same options,
//  so the two modes report attitude on one convention and the geometry audit's
//  checks C and D hold for both.
//
//  CONVENTIONS (world x-East, y-North, z-Up)
//    yaw   +ve counter-clockwise from East; the heading of the ground track.
//    roll  +ve right wing down.  tan(roll) = V*yawRate/g - a coordinated turn.
//    pitch +ve nose up.  pitch = aoa + atan(dh/ds), so level cruise is EXACTLY
//          zero and a climbing path still reports honestly.
// =============================================================================
#ifndef MTL_SENSING_AIRFRAME_HPP
#define MTL_SENSING_AIRFRAME_HPP

#include "mtl/params.hpp"
#include "mtl/types.hpp"

namespace mtl::sensing {

struct AirframeInfo {
    double maxRollDeg  = 0.0;
    double maxPitchDeg = 0.0;
    double speedMin    = 0.0;
    double speedMax    = 0.0;
};

/// @param droneTraj N-by-3 [x y h] (a 2-column track is accepted; altitude is
///                  then flat, so the pitch is zero)
/// @returns N-by-3 [roll pitch yaw] in radians
Path3 computeAirframeRPY(const Path3& droneTraj, const GimbalSchedulerParams& opts,
                         AirframeInfo* info = nullptr);

}  // namespace mtl::sensing

#endif  // MTL_SENSING_AIRFRAME_HPP
