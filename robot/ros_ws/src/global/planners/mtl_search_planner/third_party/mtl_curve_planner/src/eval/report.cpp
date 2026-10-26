#include "mtl_curve/eval/report.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>

#include "mtl_curve/eval/detection.hpp"
#include "mtl_curve/quadrotor/flatness.hpp"

namespace mtl::curve::eval {

bool reportCurveSummary(std::ostream& os, const PlanningResult& r, const PlannerParams& P) {
    os << "\n=================== PARAMETERIZED CURVE PLANNER ===================\n";
    os << "Representation: " << toString(P.curve.representation) << "   endpoints:";
    for (int a = 0; a < P.numAgents; ++a) {
        if (P.curve.endpointModes.size() == 1 && a > 0) break;
        os << ' ' << toString(P.curve.modeOf(a));
    }
    os << "\nSensor mount:   single-axis gimbal, tilt " << std::fixed << std::setprecision(1)
       << rad2deg(P.sensorTiltAngle) << " deg (stand-off " << std::setprecision(0) << P.sensorStandOff() << " m)\n";
    os << std::left << std::setw(6) << "agent" << std::right << std::setw(8) << "alt" << std::setw(10) << "L [m]"
       << std::setw(9) << "dL [%]" << std::setw(12) << "max k" << std::setw(7) << "k ok" << std::setw(10)
       << "end err" << std::setw(9) << "gimbal" << std::setw(8) << "sweep" << std::setw(8) << "W/2"
       << "  seed" << "\n";
    bool ok = true;
    for (std::size_t a = 0; a < r.plans.size(); ++a) {
        const AgentPlan& p = r.plans[a];
        const AgentTrajectory& t = r.trajectories[a];
        const double dL = 100.0 * (p.flownLength - p.budget) / p.budget;
        const bool kOK = p.maxKappa <= P.curve.maxCurvature + 1e-4;
        ok = ok && p.feasible;
        os << std::left << std::setw(6) << (a + 1) << std::right << std::setw(8) << std::setprecision(0)
           << p.altitude << std::setw(10) << std::setprecision(1) << p.flownLength << std::setw(9)
           << std::setprecision(3) << dL << std::setw(12) << std::setprecision(5) << p.maxKappa << std::setw(7)
           << (kOK ? "true" : "FALSE");
        if (p.rep.pGoal) os << std::setw(10) << std::setprecision(3) << p.endpointError;
        else os << std::setw(10) << "-";
        os << std::setw(8) << std::setprecision(1) << t.maxGimbalCmdDeg << "d" << std::setw(7)
           << rad2deg(p.sweep.alphaMax) << "d" << std::setw(8) << std::setprecision(0) << p.swathHalfWidth
           << "  " << p.initStrategy << "\n";
    }
    os << "Kinematic / length / endpoint acceptance: " << (ok ? "true" : "FALSE") << "\n";
    os << std::setprecision(6) << "Fast-model team residual: static " << r.team.Jstatic << " -> final "
       << r.team.Jfinal << "\n";
    os << "Reallocation: " << r.team.log.size() << " trial(s), " << r.team.acceptedTrials()
       << " accepted; unserviced clusters now " << r.team.audit.unserviced.size() << " of " << r.clusters.size()
       << "\n";
    os << "====================================================================\n";
    return ok;
}

void reportDetectionSummary(std::ostream& os, const std::vector<Target>& targets, const PlannerParams& params) {
    Index detected = 0;
    for (const Target& t : targets)
        if (t.detected()) ++detected;
    os << "\n--- Target Detection Summary ---\n";
    os << "Sensor mount:     single-axis gimbal, tilt " << std::fixed << std::setprecision(1)
       << rad2deg(params.sensorTiltAngle) << " deg (stand-off " << std::setprecision(0)
       << params.sensorStandOff() << " m)\n";
    os << "Targets Detected: " << detected << "\n";
    os << "Targets Missed:   " << (static_cast<Index>(targets.size()) - detected) << "\n";
    os << "--------------------------------\n";
}

void reportResidualBelief(std::ostream& os, const ResidualBelief& rb) {
    os << "\n--- Residual Belief (lower is better) ---\n" << std::fixed << std::setprecision(6);
    os << "Prior belief mass:      " << rb.priorMass << "\n";
    os << "Residual belief mass:   " << rb.residualMass << "   <- sum over the map of P(target here & missed)\n";
    os << "Belief mass searched:   " << rb.detectedMass << "   (" << std::setprecision(1)
       << (100.0 * rb.detectedMass / std::max(rb.priorMass, 1e-300)) << "% of the prior)\n";
    os << "-----------------------------------------\n";
}

bool reportQuadSummary(std::ostream& os, const PlanningResult& r, const PlannerParams& P) {
    if (r.quad.empty()) return true;
    const QuadParams q = quadrotor::resolveQuadParams(P.quad, P.sensorTiltAngle);
    const char* mount = q.cameraMount == CameraMount::Matched ? "matched"
                        : (q.cameraMount == CameraMount::Fixed ? "fixed" : "nadir");
    os << "\n=================== QUADROTOR CONVERSION (body-fixed camera) ===================\n"
       << std::fixed << std::setprecision(0) << "Vehicle: " << q.speed << " m/s cruise (plan flown "
       << std::setprecision(2) << P.avgDroneSpeed / q.speed << "x slower), tilt <= " << std::setprecision(0)
       << rad2deg(q.maxTilt) << " deg, roll/pitch rate <= " << rad2deg(q.maxRollPitchRate) << " deg/s,\n"
       << "         yaw " << (q.useYaw ? "free" : "locked to the track") << " (rate <= " << rad2deg(q.maxYawRate)
       << " deg/s), drag trim " << std::setprecision(1) << rad2deg(q.trimPitch) << " deg; camera " << mount
       << ", tilt " << rad2deg(q.camTilt) << " deg from body -z\n";
    os << std::left << std::setw(6) << "agent" << std::right << std::setw(7) << "T [s]" << std::setw(10)
       << "bore rms" << std::setw(10) << "bore p95" << std::setw(10) << "bore max" << std::setw(10) << "path rms"
       << std::setw(10) << "cross mx" << std::setw(8) << "tilt" << std::setw(9) << "pq rate" << std::setw(9)
       << "yaw rt" << std::setw(8) << "look" << std::setw(7) << "ok" << std::setw(8) << "solve" << "\n";
    bool ok = true;
    for (std::size_t a = 0; a < r.quad.size(); ++a) {
        const QuadTrajectory& Q = r.quad[a];
        const QuadStats& S = Q.stats;
        ok = ok && S.feasible;
        os << std::left << std::setw(6) << a + 1 << std::right << std::setprecision(0) << std::setw(7)
           << Q.t(Q.t.size() - 1) << std::setprecision(1) << std::setw(9) << S.boreRms << 'm' << std::setw(9)
           << S.boreP95 << 'm' << std::setw(9) << S.boreMax << 'm' << std::setw(9) << S.pathRms << 'm'
           << std::setw(9) << S.crossMax << 'm' << std::setw(7) << S.maxTiltDeg << 'd' << std::setprecision(0)
           << std::setw(5) << S.maxRateXYDegS << "d/s" << std::setw(5) << S.maxYawRateDegS << "d/s"
           << std::setprecision(1) << std::setw(7) << S.maxLookDeg << 'd' << std::setw(7)
           << (S.feasible ? "true" : "FALSE") << std::setprecision(2) << std::setw(7) << S.solveTime << "s\n";
    }
    os << "All attitude / rate / speed / thrust limits hold: " << (ok ? "true" : "false") << "\n";
    return ok;
}

}  // namespace mtl::curve::eval
