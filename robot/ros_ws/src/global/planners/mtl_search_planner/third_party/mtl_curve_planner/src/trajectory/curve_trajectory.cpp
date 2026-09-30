#include "mtl_curve/trajectory/curve_trajectory.hpp"

#include <algorithm>
#include <cmath>
#include <type_traits>
#include <utility>

#include "mtl_curve/curve_planning/arc_length.hpp"
#include "mtl_curve/sensing/airframe.hpp"

namespace mtl::curve::trajectory {

std::vector<AgentTrajectory> generateCurveTrajectories(const std::vector<VecX>& curves,
                                                       const std::vector<CurveRep>& reps,
                                                       const std::vector<double>& altitudes,
                                                       const std::vector<SweepParams>& sweeps, double V,
                                                       double dt, const GimbalParams& gimbal, VecX& timeVec,
                                                       const std::vector<double>& sweepPhase) {
    const std::size_t N = curves.size();
    const double ds = V * dt;
    std::vector<AgentTrajectory> out(N);
    Index maxSteps = 1;

    for (std::size_t a = 0; a < N; ++a) {
        const curve_planning::ArcLengthSamples R = curve_planning::reparameterizeArcLength(curves[a], reps[a], ds);
        const Index M1 = R.pts.rows();
        const double h = altitudes[a];
        AgentTrajectory& t = out[a];
        t.drone.resize(M1, 3);
        t.drone.leftCols(2) = R.pts;
        t.drone.col(2).setConstant(h);
        GimbalParams g = gimbal;
        g.dt = dt;
        t.rpy = sensing::computeAirframeRPY(t.drone, g);

        const SweepParams& sw = sweeps[a];
        const double ph = a < sweepPhase.size() ? sweepPhase[a] : 0.0;
        t.sensor.resize(M1, 2);
        t.gimbalAngle.resize(M1);
        t.gimbalCmd.resize(M1);
        for (Index k = 0; k < M1; ++k) {
            const double tk = static_cast<double>(k) * dt;
            const double alpha = sw.alphaMax * std::sin(2.0 * kPi * sw.freq * tk + ph);
            const double cross = h * std::tan(alpha) / std::cos(sw.tiltAngle);
            t.sensor(k, 0) = R.pts(k, 0) + sw.standOff * R.tan(k, 0) + cross * R.nrm(k, 0);
            t.sensor(k, 1) = R.pts(k, 1) + sw.standOff * R.tan(k, 1) + cross * R.nrm(k, 1);
            t.gimbalAngle(k) = alpha;
            t.gimbalCmd(k) = alpha + t.rpy(k, 0);
        }
        t.steps = M1;
        double len = 0.0, kmax = 0.0;
        for (Index k = 1; k < M1; ++k) len += (R.pts.row(k) - R.pts.row(k - 1)).norm();
        // discrete curvature of the flown polyline: a check, not a definition
        for (Index k = 1; k + 1 < M1; ++k) {
            const Vec2 d1 = (R.pts.row(k) - R.pts.row(k - 1)).transpose();
            const Vec2 d2 = (R.pts.row(k + 1) - R.pts.row(k)).transpose();
            const double h1 = std::atan2(d1.y(), d1.x()), h2 = std::atan2(d2.y(), d2.x());
            const double dh = std::atan2(std::sin(h2 - h1), std::cos(h2 - h1));
            const double seg = 0.5 * (d1.norm() + d2.norm());
            if (seg > 0.0) kmax = std::max(kmax, std::abs(dh) / seg);
        }
        t.length = len;
        t.maxKappaDiscrete = kmax;
        t.maxGimbalCmdDeg = rad2deg(t.gimbalCmd.cwiseAbs().maxCoeff());
        t.maxRollDeg = rad2deg(t.rpy.col(0).cwiseAbs().maxCoeff());
        t.startPt = R.pts.row(0).transpose();
        t.endPt = R.pts.row(M1 - 1).transpose();
        maxSteps = std::max(maxSteps, M1);
    }

    for (AgentTrajectory& t : out) {
        const Index n = t.drone.rows();
        if (n >= maxSteps || n == 0) continue;
        const Index pad = maxSteps - n;
        auto padRows = [pad](auto& m) {
            using M = std::decay_t<decltype(m)>;
            M p(m.rows() + pad, m.cols());
            p.topRows(m.rows()) = m;
            p.bottomRows(pad).rowwise() = m.row(m.rows() - 1);
            m = std::move(p);
        };
        padRows(t.drone);
        padRows(t.sensor);
        padRows(t.rpy);
        auto padVec = [pad](VecX& v) {
            VecX p(v.size() + pad);
            p.head(v.size()) = v;
            p.tail(pad).setConstant(v(v.size() - 1));
            v = std::move(p);
        };
        padVec(t.gimbalAngle);
        padVec(t.gimbalCmd);
    }
    timeVec.resize(maxSteps);
    for (Index i = 0; i < maxSteps; ++i) timeVec(i) = static_cast<double>(i) * dt;
    return out;
}

}  // namespace mtl::curve::trajectory
