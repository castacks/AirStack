// =============================================================================
//  mtl/eval/geometry_audit.hpp   (library: mtl_eval)
//
//  INDEPENDENT AUDIT OF AN OPTIMIZED DRONE/SENSOR PAIR.
//
//  Recomputes the sensor geometry FROM THE OUTPUTS ALONE and checks that they
//  are physically consistent for a camera on a single roll-axis (left/right)
//  gimbal, mounted either at NADIR or on a bracket TILTED by tau toward the
//  forward horizon.  With the effective look angle theta = tau - pitch:
//
//    A  along-track offset of the look point  ==  h*tan(theta)
//       i.e. NOTHING but the MOUNT TILT, the pitch and the altitude can move the
//       look point forward or backward.  Roll and gimbal drop out exactly.
//       (tau = 0 recovers the nadir law, -h*tan(pitch).)
//    B  cross-track offset                    ==  h*tan(roll+gimbal)/cos(theta)
//    C  altitude increments                   ==  tan(pitch-aoa)*ds
//       the tilt is a MOUNT, not a manoeuvre, so it must NOT appear here - this
//       is the check that the along-track stand-off was bought for free.
//    D  yaw                                   ==  heading of the ground track
//    E  all angle / rate / altitude limits respected, look angle and slant range
//       included
//
//  Plus a counter-test: the correlation of the along-track residual with roll,
//  which must be nil - if bank could move the look point forward, the whole
//  single-axis argument would be wrong.
//
//  ONLY MEANINGFUL FOR THE SINGLE-AXIS GIMBAL.  In multi-axis mode the nominal
//  trajectory is flown and the attitude comes from computeAirframeRPY, so there
//  is no gimbal schedule to audit.
// =============================================================================
#ifndef MTL_EVAL_GEOMETRY_AUDIT_HPP
#define MTL_EVAL_GEOMETRY_AUDIT_HPP

#include <map>
#include <string>

#include "mtl/types.hpp"

namespace mtl::eval {

struct GeometryReport {
    double alongTrackErr        = 0.0;  ///< A [m]
    double crossTrackErr        = 0.0;  ///< B [m]
    double altitudeErr          = 0.0;  ///< C [m]
    double yawErrDeg            = 0.0;  ///< D [deg], median over moving samples
    double rollVsAlongTrackCorr = 0.0;  ///< the counter-test; 0 = roll cannot move it
    std::map<std::string, bool> limits;  ///< E, one flag per limit
    bool   pass = false;

    double maxPitchDeg = 0.0, maxRollDeg = 0.0, maxGimbalDeg = 0.0, maxCrossDeg = 0.0;
    Vec2   alongTrackRangeM = Vec2::Zero();
    Vec2   crossTrackRangeM = Vec2::Zero();
    double tiltAngleDeg = 0.0, nadirOffsetM = 0.0;
    Vec2   lookAngleRangeDeg = Vec2::Zero();
    double maxSlantRangeM = 0.0;
};

/// @param droneTraj  N-by-3 [x y h] as flown
/// @param sensorTraj N-by-2 boresight ground point as flown
/// @param rpy        N-by-3 [roll pitch yaw] as reported
/// @param dg         the scheduler's diagnostics (the mount tilt and the limits
///                   are read back out of it, so the audit shares no state with
///                   the solver beyond its published outputs)
GeometryReport verifySensorGeometry(const Path3& droneTraj, const Path2& sensorTraj,
                                    const Path3& rpy, const GimbalDiagnostics& dg,
                                    bool verbose = false);

}  // namespace mtl::eval

#endif  // MTL_EVAL_GEOMETRY_AUDIT_HPP
