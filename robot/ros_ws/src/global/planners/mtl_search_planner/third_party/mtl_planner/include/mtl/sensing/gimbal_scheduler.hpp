// =============================================================================
//  mtl/sensing/gimbal_scheduler.hpp
//
//  OBSERVE EVERY VALID-CELL CENTRE WITH A SINGLE-AXIS GIMBAL.
//
//  ONE SOLVER, BOTH MOUNTS
//  -----------------------
//    tiltAngle = 0   nadir mount; the boresight sweeps a cross-track line
//                    DIRECTLY BELOW the aircraft.
//    tiltAngle > 0   bracket tilted forward by tau about the PITCH axis; the
//                    swept line stands off along track by f0 = h*tan(tau).
//    tiltAngle < 0   the same, looking behind.
//
//  The bracket rotates about the same axis as pitch and sits outboard of the
//  gimbal joint, so the two rotations simply ADD: a tilted mount is
//  algebraically identical to flying a nadir camera at a constant pitch of -tau.
//  Every formula below is the nadir one with the pitch replaced by the EFFECTIVE
//  LOOK ANGLE  theta = tau - pitch  (+ve = boresight forward), and tau = 0
//  collapses each back to the nadir law exactly.
//
//  THE IDEA
//  --------
//  The camera hangs off a 1-DOF gimbal that only swings it left and right, so
//  the boresight can only be moved SIDEWAYS for free.  Moving it forward or
//  backward means pitching the aircraft, and pitch is the flight-path angle, so
//  along-track pointing is paid for in altitude.  (The MOUNT tilt is the
//  exception, and the reason to have one: it buys along-track stand-off for
//  nothing, because a bracket is not a manoeuvre.)
//
//  That single fact fixes WHEN a centre can be observed - the instant is handed
//  to you by the geometry.  The freedom that remains comes in two forms, and
//  this solver is built around enumerating both:
//
//    * OTHER PASSES.  A TSP ground track loops and doubles back, so the
//      along-track offset of one centre crosses the swept line several times
//      over the sortie.  Every crossing is a chance to observe it.
//    * A FEW METRES OF PITCH.  Pitching by p slides the boresight along-track,
//      df/dp = -h*sec^2(theta).  At 300 m and nadir, five degrees is 26 m of
//      slide - enough to shift a service off a contested instant.
//
//  So: enumerate every (instant, pitch) pair at which each centre is physically
//  observable, then pick one per centre such that the gimbal can slew between
//  consecutive services.  Nothing is ever pushed to an instant where it is NOT
//  observable, which is what a naive scheduler does and why it loses half the
//  centres to a runaway pitch correction.
//
//    0. GEOMETRY   nominal track, heading, coordinated bank
//    1. CANDIDATES every instant each centre is on the boresight line, with the
//                  pitch (possibly zero) that puts it there
//    2. REACH      yaw repair, then climb, for centres with no candidate at all;
//                  both accepted only if the candidate count improves, and both
//                  behind enableRepairLoop (off by default)
//    3. SCHEDULE   one candidate per centre, gimbal-slew feasible, cheapest
//                  pitch first, so pitch stays at zero wherever it can
//    4. PROFILE    a gimbal-angle track that is smooth, hits every service knot
//                  EXACTLY, and respects the slew rate by construction
//
//  GEOMETRY (world x-East, y-North, z-Up; pitch p +ve nose up, roll r +ve right
//  wing down, gimbal phi +ve look right, tilt tau +ve look forward).  From
//  Rz(yaw)*Ry_nosedown(tau - p)*Rx(r + phi)*nadir intersected with the ground:
//
//      theta = tau - p                    effective look angle   <- tilt + pitch
//      alpha = r + phi                    combined cross-track tilt
//      f     = h*tan(theta)               along-track offset     <- tilt + pitch ONLY
//      l     = h*tan(alpha)/cos(theta)    cross-track offset     <- roll + gimbal
//      look  = [x y] + f*u + l*v
//      slant = h/(cos(theta)*cos(alpha))  boresight range
//      dh/ds = tan(p - aoa)               pitch IS the flight-path angle, so the
//                                         stand-off costs no altitude whatsoever
//
//  eval::verifySensorGeometry re-derives all of them from the outputs alone.
//
//  NOTE  This solver is for the SINGLE-AXIS gimbal only.  With a multi-axis
//  gimbal the boresight can be pointed along-track for free, so there is nothing
//  to schedule: fly the nominal trajectory and take the attitude from
//  computeAirframeRPY.
// =============================================================================
#ifndef MTL_SENSING_GIMBAL_SCHEDULER_HPP
#define MTL_SENSING_GIMBAL_SCHEDULER_HPP

#include "mtl/params.hpp"
#include "mtl/types.hpp"

namespace mtl::sensing {

struct GimbalSchedule {
    Path3              droneTraj;  ///< N-by-3 [x y h] - the scheduler may bend it
    Path2              sensorTraj; ///< N-by-2 boresight ground point
    Path3              rpy;        ///< N-by-3 [roll pitch yaw]; pitch is the AIRFRAME
                                   ///  pitch, the mount tilt is NOT folded into it
    std::vector<Index> sensorIdx;  ///< N-by-1 index into `targets` being observed (-1 = none)
    GimbalDiagnostics  diagnostics;
};

/// @param droneTraj          N-by-3 nominal trajectory from generateTrajectories
/// @param sensorTraj         N-by-2 nominal sensor ground track.  ADVISORY - only
///                           `targets` is a hard constraint.
/// @param targets            M-by-2 valid-cell centres that MUST be observed
/// @param maxPitchChangeRate max |dp| per TIME STEP [rad] (per second when
///                           opts.pitchRateIsPerSecond)
/// @throws std::invalid_argument when the mount tilt is at or past the
///         look-angle stop, where the boresight would never reach the ground.
GimbalSchedule optimizeDroneSensorTraj(const Path3& droneTraj, const Path2& sensorTraj,
                                       const Path2& targets, double maxPitchChangeRate,
                                       const GimbalSchedulerParams& opts);

}  // namespace mtl::sensing

#endif  // MTL_SENSING_GIMBAL_SCHEDULER_HPP
