#include "mtl/sensing/airframe.hpp"

#include <algorithm>
#include <cmath>

#include "mtl/core/numeric.hpp"

namespace mtl::sensing {

Path3 computeAirframeRPY(const Path3& droneTraj, const GimbalSchedulerParams& opts,
                         AirframeInfo* info) {
    const Index  n  = droneTraj.rows();
    const double dt = std::max(opts.dt, 1e-6);

    // A grounded agent is a single sample, and a two-sample track has no turn
    // rate to speak of.  Return level flight rather than a divide-by-nothing, so
    // the caller can invoke this unconditionally.
    if (n < 3) {
        Path3 rpy = Path3::Zero(std::max<Index>(n, 1), 3);
        if (n == 2) {
            const Vec2 d = droneTraj.block(1, 0, 1, 2).transpose() - droneTraj.block(0, 0, 1, 2).transpose();
            if (d.norm() > 1e-9) rpy.col(2).setConstant(std::atan2(d.y(), d.x()));
        }
        if (info) *info = AirframeInfo{};
        return rpy;
    }

    const Path2 xy = droneTraj.leftCols(2);
    const VecX  h  = droneTraj.col(2);

    VecX ds, arc;
    core::arcOf(xy, ds, arc);

    // --- yaw: the heading of the ground track ---
    const VecX yaw = core::headingOf(xy, ds);

    // --- roll: coordinated turn, tan(roll) = V*yawRate/g ---
    VecX roll = VecX::Zero(n);
    if (opts.computeRoll) {
        roll = core::coordinatedRoll(yaw, ds, dt, opts.maxRoll, opts.rollSign, opts.rollSmoothSec);
    }

    // --- pitch: the flight-path angle the altitude profile implies ---
    // Level cruise -> dh/ds = 0 -> pitch = aoa (zero by default).  It is
    // computed rather than assumed so a climbing path reports honestly, and so
    // the geometry audit's check C (h = int tan(pitch - aoa) ds) passes either
    // way.  FORWARD difference, unsmoothed, because the audit integrates
    // h(k+1) = h(k) + tan(pitch(k) - aoa)*ds(k+1): the slope pitch(k) reports
    // must be the one leading OUT of sample k.
    VecX pitch = VecX::Zero(n);
    if (opts.computePitch) {
        const VecX dhds = core::slopeOf(h, ds);
        for (Index k = 0; k < n; ++k)
            pitch(k) = core::clampAbs(opts.aoa + std::atan(dhds(k)), opts.maxPitch);
    }

    Path3 rpy(n, 3);
    rpy.col(0) = roll;
    rpy.col(1) = pitch;
    rpy.col(2) = yaw;

    if (info) {
        info->maxRollDeg  = rad2deg(roll.cwiseAbs().maxCoeff());
        info->maxPitchDeg = rad2deg(pitch.cwiseAbs().maxCoeff());
        const VecX v      = ds / dt;
        info->speedMin    = v.tail(n - 1).minCoeff();
        info->speedMax    = v.tail(n - 1).maxCoeff();
    }
    return rpy;
}

}  // namespace mtl::sensing
