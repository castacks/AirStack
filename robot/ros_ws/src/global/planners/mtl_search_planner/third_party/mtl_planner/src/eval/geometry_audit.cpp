#include "mtl/eval/geometry_audit.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "mtl/core/numeric.hpp"
#include "mtl/params.hpp"

namespace mtl::eval {
namespace {

const char* ok(bool c) { return c ? "ok " : "BAD"; }

double correlation(const VecX& a, const VecX& b) {
    const Index n = a.size();
    if (n < 2) return 0.0;
    const double ma = a.mean(), mb = b.mean();
    const VecX   da = a.array() - ma;
    const VecX   db = b.array() - mb;
    const double sa = std::sqrt(da.squaredNorm() / static_cast<double>(n - 1));
    const double sb = std::sqrt(db.squaredNorm() / static_cast<double>(n - 1));
    if (sa < 1e-9 || sb < 1e-12) return 0.0;
    return da.dot(db) / (static_cast<double>(n - 1) * sa * sb);
}

}  // namespace

GeometryReport verifySensorGeometry(const Path3& dTraj, const Path2& sTraj, const Path3& rpy,
                                    const GimbalDiagnostics& dg, bool verbose) {
    GeometryReport rep;
    const Index    n = dTraj.rows();
    if (n == 0) return rep;

    const double tau = dg.tiltAngle;

    const Path2 xy   = dTraj.leftCols(2);
    const VecX  h    = dTraj.col(2);
    const VecX  roll = rpy.col(0);
    const VecX  p    = rpy.col(1);
    const VecX  yaw  = rpy.col(2);
    // The gimbal profile must describe the trajectory being audited.  A short
    // one means the two came from different runs (or a padded trajectory whose
    // diagnostics were not padded with it), and quietly substituting zeros
    // there would turn check B into nonsense - so it is a failed check, not a
    // fallback.
    const bool gimbalOk = (dg.gimbalAngle.size() == n);
    const VecX  gim = gimbalOk ? dg.gimbalAngle : VecX::Zero(n);
    rep.limits["diagnosticsLength"] = gimbalOk;

    VecX ds, arc;
    core::arcOf(xy, ds, arc);

    // The frame implied by the REPORTED yaw - not by a recomputed heading, so
    // that D is a real test rather than a tautology.
    Path2 uh(n, 2), vh(n, 2);
    for (Index k = 0; k < n; ++k) {
        uh(k, 0) = std::cos(yaw(k));
        uh(k, 1) = std::sin(yaw(k));
        vh(k, 0) = uh(k, 1);
        vh(k, 1) = -uh(k, 0);
    }

    VecX fA(n), cA(n), theta(n), fW(n), cW(n), alpha(n), slant(n);
    for (Index k = 0; k < n; ++k) {
        const double rx = sTraj(k, 0) - xy(k, 0);
        const double ry = sTraj(k, 1) - xy(k, 1);
        fA(k)    = rx * uh(k, 0) + ry * uh(k, 1);   // achieved along-track offset
        cA(k)    = rx * vh(k, 0) + ry * vh(k, 1);   // achieved cross-track offset
        theta(k) = tau - p(k);                      // effective look angle, +ve forward
        fW(k)    = h(k) * std::tan(theta(k));
        alpha(k) = roll(k) + gim(k);
        cW(k)    = h(k) * std::tan(alpha(k)) / std::cos(theta(k));
        slant(k) = h(k) / std::max(std::cos(theta(k)) * std::cos(alpha(k)), 1e-9);
    }

    // ---- A: along-track is the MOUNT TILT and pitch (and altitude) only ----
    rep.alongTrackErr = (fA - fW).cwiseAbs().maxCoeff();

    // ---- B: cross-track is roll + gimbal ----------------------------------
    rep.crossTrackErr = (cA - cW).cwiseAbs().maxCoeff();

    // ---- the counter-test: could roll alone be moving the look forward? ----
    rep.rollVsAlongTrackCorr = correlation(roll, (fA - fW).eval());

    // ---- C: altitude is the integral of the flight-path angle -------------
    VecX hW(n);
    hW(0) = h(0);
    for (Index k = 0; k + 1 < n; ++k) {
        hW(k + 1) = std::min(std::max(hW(k) + std::tan(p(k) - dg.aoa) * ds(k + 1), dg.hMin),
                             dg.hMax);
    }
    rep.altitudeErr = (hW - h).cwiseAbs().maxCoeff();

    // ---- D: yaw is the ground-track heading -------------------------------
    // Median, not max: Dubins corner samples are noisy.
    std::vector<double> yawErr;
    for (Index k = 0; k < n; ++k) {
        if (ds(k) <= 1e-6) continue;
        const double dx = (k == 0) ? 1.0 : xy(k, 0) - xy(k - 1, 0);
        const double dy = (k == 0) ? 0.0 : xy(k, 1) - xy(k - 1, 1);
        yawErr.push_back(std::abs(core::wrapPi(yaw(k) - std::atan2(dy, dx))));
    }
    if (!yawErr.empty()) {
        std::sort(yawErr.begin(), yawErr.end());
        rep.yawErrDeg = rad2deg(yawErr[yawErr.size() / 2]);
    }

    // ---- E: limits --------------------------------------------------------
    const double pitchRateLim = deg2rad(dg.pitchRateLimitDeg);
    auto maxDiff = [n](const VecX& v) {
        return (n > 1) ? (v.tail(n - 1) - v.head(n - 1)).cwiseAbs().maxCoeff() : 0.0;
    };

    rep.limits["pitch"]      = p.cwiseAbs().maxCoeff() <= dg.maxPitch + 1e-6;
    rep.limits["pitchRate"]  = maxDiff(p) <= pitchRateLim + 1e-9;
    rep.limits["gimbal"]     = gim.cwiseAbs().maxCoeff() <= dg.gimbalMax + 1e-9;
    rep.limits["gimbalRate"] = maxDiff(gim) / dg.dt <= dg.gimbalRate + 1e-6;
    rep.limits["cross"]      = alpha.cwiseAbs().maxCoeff() <= dg.maxCrossAngle + 1e-9;
    rep.limits["altitude"]   = (h.array() >= dg.hMin - 1e-6).all() && (h.array() <= dg.hMax + 1e-6).all();
    rep.limits["roll"]       = roll.cwiseAbs().maxCoeff() <= dg.maxRoll + 1e-9;
    rep.limits["finite"]     = dTraj.allFinite() && sTraj.allFinite() && rpy.allFinite();
    // The tilted mount adds two limits of its own: the boresight must stay below
    // the horizon, and a real sensor runs out of RANGE (which grows as
    // 1/cos(theta)) long before the geometry runs out of reach.
    rep.limits["lookAngle"]  = theta.cwiseAbs().maxCoeff() <= dg.maxLookAngle + 1e-9;
    if (std::isfinite(dg.maxSlantRange))
        rep.limits["slantRange"] = slant.maxCoeff() <= dg.maxSlantRange * (1.0 + 1e-6);

    bool allLimits = true;
    for (const auto& kv : rep.limits) allLimits = allLimits && kv.second;

    rep.pass = rep.alongTrackErr < 1e-6 && rep.crossTrackErr < 1e-6 && rep.altitudeErr < 1e-6 &&
               rep.yawErrDeg < 1.0 && std::abs(rep.rollVsAlongTrackCorr) < 1e-6 && allLimits;

    rep.maxPitchDeg      = rad2deg(p.cwiseAbs().maxCoeff());
    rep.maxRollDeg       = rad2deg(roll.cwiseAbs().maxCoeff());
    rep.maxGimbalDeg     = rad2deg(gim.cwiseAbs().maxCoeff());
    rep.maxCrossDeg      = rad2deg(alpha.cwiseAbs().maxCoeff());
    rep.alongTrackRangeM = Vec2(fA.minCoeff(), fA.maxCoeff());
    rep.crossTrackRangeM = Vec2(cA.minCoeff(), cA.maxCoeff());
    rep.tiltAngleDeg     = rad2deg(tau);
    rep.nadirOffsetM     = h.mean() * std::tan(tau);
    rep.lookAngleRangeDeg = Vec2(rad2deg(theta.minCoeff()), rad2deg(theta.maxCoeff()));
    rep.maxSlantRangeM   = slant.maxCoeff();

    if (verbose) {
        std::printf("\n--- sensor geometry audit (%lld steps, mount tilt %.1f deg) ---\n",
                    static_cast<long long>(n), rep.tiltAngleDeg);
        std::printf("A  along-track  = h*tan(tilt-pitch)        max err %8.2e m\n",
                    rep.alongTrackErr);
        std::printf("   ... residual vs roll, correlation       %8.2e  (0 = roll cannot move it)\n",
                    rep.rollVsAlongTrackCorr);
        std::printf("B  cross-track  = h*tan(roll+gim)/cos th   max err %8.2e m\n",
                    rep.crossTrackErr);
        std::printf("C  altitude     = int tan(pitch-aoa) ds    max err %8.2e m  (tilt is FREE)\n",
                    rep.altitudeErr);
        std::printf("D  yaw          = ground-track heading     median  %8.2e deg\n", rep.yawErrDeg);
        std::printf("E  limits       pitch %s  pitchRate %s  gimbal %s  gimbalRate %s\n",
                    ok(rep.limits["pitch"]), ok(rep.limits["pitchRate"]), ok(rep.limits["gimbal"]),
                    ok(rep.limits["gimbalRate"]));
        std::printf("                cross %s  altitude  %s  roll   %s  finite     %s\n",
                    ok(rep.limits["cross"]), ok(rep.limits["altitude"]), ok(rep.limits["roll"]),
                    ok(rep.limits["finite"]));
        std::printf("                lookAngle %s  slantRange %s\n", ok(rep.limits["lookAngle"]),
                    rep.limits.count("slantRange") ? ok(rep.limits["slantRange"]) : "n/a");
        std::printf(
            "   pitch %.1f deg | roll %.1f deg | gimbal %.1f deg | total cross tilt %.1f deg\n",
            rep.maxPitchDeg, rep.maxRollDeg, rep.maxGimbalDeg, rep.maxCrossDeg);
        std::printf(
            "   look angle %.1f..%.1f deg | stand-off at gimbal centre %.0f m | slant max %.0f m\n",
            rep.lookAngleRangeDeg.x(), rep.lookAngleRangeDeg.y(), rep.nadirOffsetM,
            rep.maxSlantRangeM);
        std::printf("   look offsets: along-track %.0f..%.0f m, cross-track %.0f..%.0f m\n",
                    rep.alongTrackRangeM.x(), rep.alongTrackRangeM.y(), rep.crossTrackRangeM.x(),
                    rep.crossTrackRangeM.y());
        std::printf("%s\n\n", rep.pass ? "AUDIT PASSED" : "AUDIT FAILED");
    }
    return rep;
}

}  // namespace mtl::eval
