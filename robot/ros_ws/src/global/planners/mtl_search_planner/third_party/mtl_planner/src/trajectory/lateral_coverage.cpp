#include "mtl/trajectory/lateral_coverage.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "mtl/sensing/abeam.hpp"

namespace mtl::trajectory {
namespace {

/// Final position and heading of a sampled ground track.
void endPose(const Path2& xy, Vec2& pos, double& psi) {
    pos = xy.row(xy.rows() - 1).transpose();
    Index k = xy.rows() - 2;
    while (k >= 0 && (xy.row(xy.rows() - 1) - xy.row(k)).norm() < 1e-9) --k;
    if (k < 0) {
        psi = 0.0;
    } else {
        const Vec2 d = xy.row(xy.rows() - 1).transpose() - xy.row(k).transpose();
        psi = std::atan2(d.y(), d.x());
    }
}

}  // namespace

ExtensionResult extendTrajForLateralCoverage(const Path2& droneTraj2D, const Path2& cellCenters,
                                             double droneAltitude, double /*minTurnRadius*/,
                                             const ExtensionParams& o) {
    ExtensionResult res;
    res.extXY.resize(0, 2);
    res.info.note = "not run";

    if (cellCenters.rows() == 0 || droneTraj2D.rows() < 2) {
        res.info.note = "nothing to do";
        return res;
    }
    const Path2& T = cellCenters;
    const Index  M = T.rows();

    // --- the audit envelope, for REPORTING only -----------------------------
    // Cross-track ground offset the boresight can reach at cruise with the
    // aircraft level.  It says which centres the nominal track abandoned and
    // which the run-out rescued; it does NOT set how far the run-out goes.
    const double reachGeom = droneAltitude * std::tan(std::min(o.gimbalMax, o.maxCrossAngle));
    const double reachEff  = o.reachMargin * std::min(o.maxSensorReach, reachGeom);
    const double alongTol  = droneAltitude * std::tan(deg2rad(o.alongTolDeg));

    res.info.reachEff = reachEff;
    res.info.alongTol = alongTol;

    const sensing::AbeamResult obs0 =
        sensing::abeamObservable(droneTraj2D, T, reachEff, alongTol, o.alongOffset);
    for (Index j = 0; j < M; ++j)
        if (!obs0.observable[static_cast<std::size_t>(j)]) res.info.residualBefore.push_back(j);

    if (res.info.residualBefore.empty()) {
        res.info.note = "every centre already comes abeam on the nominal track";
        if (o.verbose) {
            std::printf(
                "  extendTraj: all %lld centres already come abeam within %.0f m - no run-out "
                "added.\n",
                static_cast<long long>(M), reachEff);
        }
        return res;
    }

    // --- the straight run-out ----------------------------------------------
    // Hold the heading the route ended on and keep flying, for a SET distance.
    // The cells left over lie ahead of that last leg: their offset along it
    // falls and then rises as the aircraft flies on, so they cross the swept
    // line on the way past.  A fixed length means the extension costs the same
    // on every sortie, so the budget left for the route itself does not move
    // around underneath the planner.
    const double L = std::min(std::max(o.extendDist, 0.0), o.maxExtraDist);
    if (L <= o.stepSize) {
        res.info.residualAfter = res.info.residualBefore;
        res.info.note          = "run-out distance is zero";
        return res;
    }

    Vec2   p0;
    double psi = 0.0;
    endPose(droneTraj2D, p0, psi);
    const Vec2 u(std::cos(psi), std::sin(psi));

    std::vector<double> s;
    for (double t = o.stepSize; t < L; t += o.stepSize) s.push_back(t);
    if (s.empty() || (L - s.back()) > 1e-6) s.push_back(L);

    res.extXY.resize(static_cast<Index>(s.size()), 2);
    for (std::size_t i = 0; i < s.size(); ++i)
        res.extXY.row(static_cast<Index>(i)) = (p0 + s[i] * u).transpose();

    res.info.extendDist = L;
    res.info.extraDist  = L;

    // --- re-audit, again only to report -------------------------------------
    Path2 fullXY(droneTraj2D.rows() + res.extXY.rows(), 2);
    fullXY << droneTraj2D, res.extXY;
    const sensing::AbeamResult obs =
        sensing::abeamObservable(fullXY, T, reachEff, alongTol, o.alongOffset);
    for (Index j = 0; j < M; ++j)
        if (!obs.observable[static_cast<std::size_t>(j)]) res.info.residualAfter.push_back(j);

    res.info.note =
        std::to_string(static_cast<long long>(std::lround(L))) + " m straight run-out";

    if (o.verbose) {
        std::printf(
            "  extendTraj: %lld of %lld centres never come abeam within %.0f m - flying on %.0f m "
            "in the final heading; never abeam %lld -> %lld.\n",
            static_cast<long long>(res.info.residualBefore.size()), static_cast<long long>(M),
            reachEff, L, static_cast<long long>(res.info.residualBefore.size()),
            static_cast<long long>(res.info.residualAfter.size()));
    }
    return res;
}

}  // namespace mtl::trajectory
