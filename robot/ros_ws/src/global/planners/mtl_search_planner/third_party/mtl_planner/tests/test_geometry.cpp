// The sensor geometry: the abeam gate, the attitude model, and the scheduler
// audited from its own outputs.
#include <cmath>

#include "mtl/core/numeric.hpp"
#include "mtl/eval/geometry_audit.hpp"
#include "mtl/sensing/abeam.hpp"
#include "mtl/sensing/airframe.hpp"
#include "mtl/sensing/gimbal_scheduler.hpp"
#include "mtl/trajectory/lateral_coverage.hpp"
#include "test_util.hpp"

using namespace mtl;

namespace {

/// A straight, level track heading east.
Path3 straightTrack(Index n, double x0, double y0, double step, double alt) {
    Path3 t(n, 3);
    for (Index i = 0; i < n; ++i) {
        t(i, 0) = x0 + static_cast<double>(i) * step;
        t(i, 1) = y0;
        t(i, 2) = alt;
    }
    return t;
}

}  // namespace

int main() {
    // --- abeamObservable: the gate a single-axis gimbal actually imposes ----
    {
        const Path3 tr = straightTrack(400, 0.0, 0.0, 2.0, 300.0);
        const Path2 xy = tr.leftCols(2);

        Path2 pts(4, 2);
        pts << 400, 200,     // beside the track, within reach -> observable
               400, 900,     // beside the track, too far out  -> not
               2000, 0,      // far ahead, the track ends first -> not
               100, -300;    // beside the track on the other side -> observable
        const sensing::AbeamResult r = sensing::abeamObservable(xy, pts, 600.0, 0.0, 0.0);
        CHECK(r.observable[0]);
        CHECK(!r.observable[1]);
        CHECK(!r.observable[2]);
        CHECK(r.observable[3]);

        // A forward-tilted mount sweeps a line standing off ahead, so a centre
        // past the end of the track comes onto the line while the aircraft is
        // still on it.
        const double standoff = 300.0 * std::tan(deg2rad(50.0));
        Path2 ahead(1, 2);
        ahead << 800.0 + standoff * 0.5, 100;
        CHECK(sensing::abeamObservable(xy, ahead, 600.0, 0.0, standoff).observable[0]);
    }

    // --- computeAirframeRPY: level cruise is exactly level ------------------
    {
        const Path3 tr = straightTrack(200, 0.0, 0.0, 2.0, 300.0);
        GimbalSchedulerParams o;
        const Path3 rpy = sensing::computeAirframeRPY(tr, o);
        CHECK(rpy.rows() == tr.rows());
        CHECK(rpy.col(0).cwiseAbs().maxCoeff() < 1e-6);   // wings level on a straight leg
        CHECK(rpy.col(1).cwiseAbs().maxCoeff() < 1e-9);   // level cruise -> pitch exactly zero
        CHECK_NEAR(rpy(100, 2), 0.0, 1e-9);               // heading due east
    }

    // --- a turn banks INTO the turn, and within the limit ------------------
    {
        const Index n = 400;
        Path3 tr(n, 3);
        const double R = 200.0, step = 2.0;
        for (Index i = 0; i < n; ++i) {
            const double s = static_cast<double>(i) * step / R;
            tr(i, 0) = R * std::sin(s);
            tr(i, 1) = R * (1.0 - std::cos(s));  // left turn
            tr(i, 2) = 300.0;
        }
        GimbalSchedulerParams o;
        sensing::AirframeInfo info;
        const Path3 rpy = sensing::computeAirframeRPY(tr, o, &info);
        CHECK(info.maxRollDeg > 1.0);                 // it is banking
        CHECK(info.maxRollDeg <= rad2deg(o.maxRoll) + 1e-6);
        CHECK(rpy.col(0).mean() < 0.0);               // into a left turn, per the sign convention
    }

    // --- the scheduler, audited from its own outputs -----------------------
    // The audit re-derives the geometry independently, so this is the test that
    // the whole single-axis argument holds for the code as written.
    for (const double tiltDeg : {0.0, 30.0, 50.0}) {
        const Path3 tr = straightTrack(600, 0.0, 0.0, 2.0, 300.0);
        const Path2 nominalSensor = tr.leftCols(2);

        // Centres strung along beside the track, inside the gimbal's reach.
        Path2 tgts(6, 2);
        tgts << 200, 150, 400, -200, 600, 250, 800, -150, 1000, 100, 200, -250;

        GimbalSchedulerParams o;
        o.tiltAngle = deg2rad(tiltDeg);
        o.verbose   = false;

        const sensing::GimbalSchedule s =
            sensing::optimizeDroneSensorTraj(tr, nominalSensor, tgts, deg2rad(1.0), o);

        CHECK(s.droneTraj.rows() == tr.rows());
        CHECK(s.sensorTraj.allFinite());
        CHECK(s.rpy.allFinite());
        CHECK(s.diagnostics.gimbalMaxDeg <= rad2deg(o.gimbalMax) + 1e-6);
        CHECK(s.diagnostics.gimbalRateMaxDeg <= rad2deg(o.gimbalRate) + 1e-3);

        const eval::GeometryReport rep =
            eval::verifySensorGeometry(s.droneTraj, s.sensorTraj, s.rpy, s.diagnostics, false);
        CHECK(rep.alongTrackErr < 1e-6);
        CHECK(rep.crossTrackErr < 1e-6);
        CHECK(rep.altitudeErr < 1e-6);
        CHECK(rep.yawErrDeg < 1.0);
        CHECK(std::abs(rep.rollVsAlongTrackCorr) < 1e-6);
        CHECK(rep.pass);

        // The tilt must buy stand-off for free: no altitude spent, and the look
        // point genuinely ahead.
        if (tiltDeg > 0.0) {
            CHECK(rep.nadirOffsetM > 100.0);
            CHECK(s.diagnostics.altRmsDev < 1e-6);
        }
    }

    // --- the reach repair loop: off by default, and it works when enabled ---
    // Centres placed just beyond the cross-track reach, which is what the yaw
    // lever exists for: bending the track is the only thing that can help,
    // because the gimbal is already at its stop.
    {
        const Path3 tr = straightTrack(800, 0.0, 0.0, 2.0, 300.0);
        Path2 tgts(6, 2);
        tgts << 300, 560, 600, -570, 900, 565, 1200, -555, 400, 200, 800, -200;

        GimbalSchedulerParams o;
        o.tiltAngle = deg2rad(30);
        o.verbose   = false;

        o.enableRepairLoop = false;
        const sensing::GimbalSchedule off =
            sensing::optimizeDroneSensorTraj(tr, tr.leftCols(2), tgts, deg2rad(1.0), o);
        CHECK(off.diagnostics.yawRepairMaxDeg == 0.0);   // off means off
        CHECK(off.diagnostics.pathShiftMax == 0.0);
        CHECK(off.diagnostics.nTargetsHit < 6);          // the centres really are out of reach

        o.enableRepairLoop = true;
        const sensing::GimbalSchedule on =
            sensing::optimizeDroneSensorTraj(tr, tr.leftCols(2), tgts, deg2rad(1.0), o);
        CHECK(on.diagnostics.nTargetsHit > off.diagnostics.nTargetsHit);
        CHECK(on.diagnostics.yawRepairMaxDeg > 0.0);
        // Bending the track must not break the geometry it was bent to serve.
        const eval::GeometryReport rep =
            eval::verifySensorGeometry(on.droneTraj, on.sensorTraj, on.rpy, on.diagnostics, false);
        CHECK(rep.pass);
        // The repair perturbs the HEADING and re-integrates at the same speed,
        // so the timeline is an invariant: same sample count, same arc length.
        CHECK(on.droneTraj.rows() == tr.rows());
        CHECK_NEAR(core::polylineLength(on.droneTraj), core::polylineLength(tr), 1e-6);
    }

    // --- a degenerate two-sample track must not divide by nothing ----------
    {
        Path3 tr(2, 3);
        tr << 0, 0, 300, 10, 0, 300;
        Path2 sensor = tr.leftCols(2);
        Path2 tgts(1, 2);
        tgts << 5, 50;
        GimbalSchedulerParams o;
        const sensing::GimbalSchedule s =
            sensing::optimizeDroneSensorTraj(tr, sensor, tgts, deg2rad(1.0), o);
        CHECK(s.rpy.rows() == 2);
        CHECK(s.rpy.allFinite());
    }

    // --- the lateral-coverage extension: a straight run-out past the end ---
    // The route stops dead at the last centroid, so centres ahead of that last
    // heading never come abeam.  Flying on in the same direction carries them
    // past: their along-track offset falls through zero as the aircraft goes by.
    {
        const Path3 tr = straightTrack(400, 0.0, 0.0, 2.0, 300.0);
        const Path2 xy = tr.leftCols(2);

        Path2 cells(3, 2);
        cells << 1000, 0,      // straight ahead
                 1100, 300,    // ahead and off to one side, inside the reach
                 1200, -250;   // ahead and off the other
        ExtensionParams o;
        o.verbose    = false;
        o.extendDist = 600.0;
        const trajectory::ExtensionResult ext =
            trajectory::extendTrajForLateralCoverage(xy, cells, 300.0, 100.0, o);

        CHECK(!ext.info.residualBefore.empty());   // the nominal track abandons them
        CHECK(ext.info.residualAfter.empty());     // the run-out carries them past
        CHECK(ext.extXY.rows() > 0);
        // The length is the SET distance and nothing else - not the cells, not
        // the reach, not how many were missed.
        CHECK_NEAR(ext.info.extraDist, 600.0, 1e-9);
        CHECK_NEAR(core::polylineLength(ext.extXY), 600.0 - o.stepSize, 1e-6);
        // and it is straight: it holds the heading the route ended on.
        const Vec2 step = ext.extXY.row(ext.extXY.rows() - 1) - ext.extXY.row(0);
        CHECK_NEAR(std::atan2(step.y(), step.x()), 0.0, 1e-9);
    }

    // --- nothing is appended when nothing needs it -------------------------
    {
        const Path3 tr = straightTrack(400, 0.0, 0.0, 2.0, 300.0);
        const Path2 xy = tr.leftCols(2);
        Path2 cells(2, 2);
        cells << 400, 200, 100, -300;   // both come abeam on the nominal track
        ExtensionParams o;
        o.verbose    = false;
        o.extendDist = 600.0;
        const trajectory::ExtensionResult ext =
            trajectory::extendTrajForLateralCoverage(xy, cells, 300.0, 100.0, o);
        CHECK(ext.extXY.rows() == 0);
        CHECK_NEAR(ext.info.extraDist, 0.0, 1e-12);
    }

    return test::report("test_geometry");
}
