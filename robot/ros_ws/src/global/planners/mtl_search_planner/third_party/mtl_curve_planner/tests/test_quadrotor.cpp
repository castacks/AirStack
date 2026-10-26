// Quadrotor (body-fixed camera) conversion - the C++ twin of test_quad_flatness.m.
//
// Builds a synthetic fixed-wing plan the way generateCurveTrajectories does
// (straight, a minimum-radius U-turn, straight; 50 deg mount, +/-38.7 deg sweep
// at 0.25 Hz, 300 m) and checks:
//   1 B-spline basis: partition of unity, time derivatives = finite differences
//   2 flatness map: [xb yb zb] = Rz(yaw) Ry(-pitch) Rx(roll), boresight from R
//   3 open-loop rigid-body integration of the flat thrust + body rates
//     reproduces the planned position and attitude (RK4, 30 s)
//   4 every limit holds and the solve reports feasible
//   5 boresight rms < 10 m, path rms < 15 m
//   6 yaw locked: yaw = track, limits hold, tracking worse than with yaw
//   7 team wrapper / matched looks: evaluator-ready shapes and timelines
#include <cmath>
#include <cstdio>
#include <vector>

#include "mtl_curve/params.hpp"
#include "mtl_curve/quadrotor/flatness.hpp"
#include "mtl_curve/quadrotor/quad_trajectory.hpp"
#include "test_util.hpp"

#include <Eigen/Geometry>
#include <Eigen/SVD>

namespace mc = mtl::curve;
namespace mq = mtl::curve::quadrotor;
using mc::Index;
using mc::Vec2;
using mc::Vec3;
using mc::VecX;
using Mat3 = Eigen::Matrix3d;

namespace {

mc::AgentTrajectory syntheticPlan(double& curveLength) {
    const double Vfw = 20, dt = 0.1, h = 300, tau = mc::deg2rad(50), Rmin = 100;
    const double ds = Vfw * dt, L1 = 600, Lt = mc::kPi * Rmin, L2 = 600;
    const Index n = static_cast<Index>(std::floor((L1 + Lt + L2) / ds + 1e-9)) + 1;
    std::vector<double> th(static_cast<std::size_t>(n), 0.0);
    mc::Path2 P(n, 2);
    P.row(0) = Vec2(0, 0).transpose();
    for (Index k = 1; k < n; ++k) {
        const double sm = (static_cast<double>(k) - 0.5) * ds;
        const double kap = (sm > L1 && sm <= L1 + Lt) ? 1.0 / Rmin : 0.0;
        const std::size_t ku = static_cast<std::size_t>(k);
        th[ku] = th[ku - 1] + kap * ds;
        const double tm = 0.5 * (th[ku] + th[ku - 1]);
        P.row(k) = P.row(k - 1) + ds * Vec2(std::cos(tm), std::sin(tm)).transpose();
    }
    mc::AgentTrajectory A;
    A.drone.resize(n, 3);
    A.sensor.resize(n, 2);
    const double aMax = mc::deg2rad(38.7), f = 0.25;
    for (Index k = 0; k < n; ++k) {
        const double t = static_cast<double>(k) * dt, thk = th[static_cast<std::size_t>(k)];
        const Vec2 tg(std::cos(thk), std::sin(thk)), nr(-tg.y(), tg.x());
        const Vec2 p = P.row(k).transpose() + Vec2(1000, 1000);
        const double alpha = aMax * std::sin(2 * mc::kPi * f * t);
        A.drone.row(k) << p.x(), p.y(), h;
        A.sensor.row(k) = (p + h * std::tan(tau) * tg + (h * std::tan(alpha) / std::cos(tau)) * nr).transpose();
    }
    A.steps = n;
    curveLength = static_cast<double>(n - 1) * ds;
    return A;
}

Mat3 hat(const Vec3& w) {
    Mat3 m;
    m << 0, -w.z(), w.y(), w.z(), 0, -w.x(), -w.y(), w.x(), 0;
    return m;
}

}  // namespace

int main() {
    const double Vfw = 20, dt = 0.1, tau = mc::deg2rad(50), beta = 610;
    double Lc = 0;
    const std::vector<mc::AgentTrajectory> fw{syntheticPlan(Lc)};

    // ---- 1. B-spline basis ------------------------------------------------------
    {
        const double T = 20, Dl = 0.5, hs = 0.01;
        const Index K = static_cast<Index>(T / hs + 1e-9) + 1;
        VecX tt(K);
        for (Index k = 0; k < K; ++k) tt(k) = static_cast<double>(k) * hs;
        const mq::UniformBspline B = mq::uniformBspline(tt, Dl, T, 5, 4);
        VecX c(B.nCP);
        for (Index i = 0; i < B.nCP; ++i) c(i) = std::sin(1.3 * static_cast<double>(i)) + 0.1 * static_cast<double>(i % 7);
        double pu = 0;
        for (Index k = 0; k < K; ++k) pu = std::max(pu, std::abs(B.b[0].row(k).sum() - 1.0));
        CHECK(pu < 1e-12);
        for (int d = 0; d < 4; ++d) {
            const VecX f0 = B.eval(d, c), f1 = B.eval(d + 1, c);
            double e = 0, sc = 1;
            for (Index k = 50; k < K - 50; ++k) {
                e = std::max(e, std::abs((f0(k + 1) - f0(k - 1)) / (2 * hs) - f1(k)));
                sc = std::max(sc, std::abs(f1(k)));
            }
            CHECK(e / sc < (d < 3 ? 2e-3 : 0.05));   // snap of a quintic is only C0
        }
        std::printf("  1 B-spline basis: |sum - 1| %.1e\n", pu);
    }

    // ---- the conversion, yaw on -------------------------------------------------------
    mc::QuadParams qp;
    qp.speed = 6;
    VecX tv;
    const std::vector<mc::QuadTrajectory> quad = mq::generateQuadTrajectories(fw, Vfw, dt, tau, beta, qp, tv);
    const mc::QuadParams q = mq::resolveQuadParams(qp, tau);
    const mc::QuadTrajectory& Q = quad[0];
    const mc::QuadStats& S = Q.stats;
    const double h = Q.altitude;

    // ---- 2. flatness map: RPY convention and boresight ----------------------------------
    {
        const mq::UniformBspline B = mq::uniformBspline(Q.t, Q.knotSpacing, Q.horizon, Q.degree, 3);
        double eR = 0, eB = 0;
        for (Index k = 0; k < Q.t.size(); k += 25) {
            const Vec2 v(B.eval(1, k, Q.cx), B.eval(1, k, Q.cy)), a(B.eval(2, k, Q.cx), B.eval(2, k, Q.cy));
            const Vec2 j(B.eval(3, k, Q.cx), B.eval(3, k, Q.cy));
            const mq::FlatState M = mq::flatMap(v, a, j, B.eval(0, k, Q.cpsi), B.eval(1, k, Q.cpsi), h, q);
            const double ph = Q.traj.rpy(k, 0), thF = -Q.traj.rpy(k, 1), ps = Q.traj.rpy(k, 2);
            const Mat3 R = (Eigen::AngleAxisd(ps, Vec3::UnitZ()) * Eigen::AngleAxisd(thF, Vec3::UnitY()) *
                                   Eigen::AngleAxisd(ph, Vec3::UnitX())).toRotationMatrix();
            Mat3 Rm;
            Rm.col(0) = M.xb; Rm.col(1) = M.yb; Rm.col(2) = M.zb;
            eR = std::max(eR, (R - Rm).norm());
            const Vec3 d = R * Vec3(std::sin(q.camTilt), 0, -std::cos(q.camTilt));
            const Vec2 bk = Q.traj.drone.row(k).head<2>().transpose() + (h / -d.z()) * d.head<2>();
            eB = std::max(eB, (bk - Q.traj.sensor.row(k).transpose()).norm());
        }
        CHECK(eR < 1e-9);
        CHECK(eB < 1e-6);
        std::printf("  2 flatness map: |dR| %.1e, |db| %.1e m\n", eR, eB);
    }

    // ---- 3. open-loop rigid-body integration with the flat inputs --------------------------
    {
        const double hs = 0.005, Tsim = 30;
        const Index K = static_cast<Index>(Tsim / hs + 1e-9) + 1;
        VecX ts(K);
        for (Index k = 0; k < K; ++k) ts(k) = static_cast<double>(k) * hs;
        const mq::UniformBspline B = mq::uniformBspline(ts, Q.knotSpacing, Q.horizon, Q.degree, 3);
        std::vector<mq::FlatState> M(static_cast<std::size_t>(K));
        for (Index k = 0; k < K; ++k)
            M[static_cast<std::size_t>(k)] = mq::flatMap(Vec2(B.eval(1, k, Q.cx), B.eval(1, k, Q.cy)),
                                                        Vec2(B.eval(2, k, Q.cx), B.eval(2, k, Q.cy)),
                                                        Vec2(B.eval(3, k, Q.cx), B.eval(3, k, Q.cy)),
                                                        B.eval(0, k, Q.cpsi), B.eval(1, k, Q.cpsi), h, q);
        Vec3 x(B.eval(0, 0, Q.cx), B.eval(0, 0, Q.cy), h), v(B.eval(1, 0, Q.cx), B.eval(1, 0, Q.cy), 0);
        Mat3 R;
        R.col(0) = M[0].xb; R.col(1) = M[0].yb; R.col(2) = M[0].zb;
        const double g = q.gravity;
        auto drag = [&](const Vec3& vv) { return Vec3((q.dragQuadResolved * vv.norm() + q.dragLin) * vv); };
        double maxPe = 0, maxRe = 0;
        for (Index k = 0; k + 1 < K; ++k) {
            const mq::FlatState &M0 = M[static_cast<std::size_t>(k)], &M1 = M[static_cast<std::size_t>(k + 1)];
            const double f[3] = {M0.f, 0.5 * (M0.f + M1.f), M1.f};
            const Vec3 w[3] = {M0.rates, 0.5 * (M0.rates + M1.rates), M1.rates};
            auto dv = [&](const Vec3& vv, const Mat3& RR, int s) {
                return Vec3(f[s] * RR.col(2) - g * Vec3::UnitZ() - drag(vv));
            };
            const Vec3 k1x = v, k1v = dv(v, R, 0);
            const Mat3 k1R = R * hat(w[0]);
            const Vec3 k2x = v + hs / 2 * k1v, k2v = dv(v + hs / 2 * k1v, R + hs / 2 * k1R, 1);
            const Mat3 k2R = (R + hs / 2 * k1R) * hat(w[1]);
            const Vec3 k3x = v + hs / 2 * k2v, k3v = dv(v + hs / 2 * k2v, R + hs / 2 * k2R, 1);
            const Mat3 k3R = (R + hs / 2 * k2R) * hat(w[1]);
            const Vec3 k4x = v + hs * k3v, k4v = dv(v + hs * k3v, R + hs * k3R, 2);
            const Mat3 k4R = (R + hs * k3R) * hat(w[2]);
            x += hs / 6 * (k1x + 2 * k2x + 2 * k3x + k4x);
            v += hs / 6 * (k1v + 2 * k2v + 2 * k3v + k4v);
            R += hs / 6 * (k1R + 2 * k2R + 2 * k3R + k4R);
            Eigen::JacobiSVD<Mat3> svd(R, Eigen::ComputeFullU | Eigen::ComputeFullV);
            R = svd.matrixU() * svd.matrixV().transpose();
            const Vec3 xp(B.eval(0, k + 1, Q.cx), B.eval(0, k + 1, Q.cy), h);
            maxPe = std::max(maxPe, (x - xp).norm());
            Mat3 Rp;
            Rp.col(0) = M1.xb; Rp.col(1) = M1.yb; Rp.col(2) = M1.zb;
            const double cth = std::min(1.0, ((R.transpose() * Rp).trace() - 1) / 2);
            maxRe = std::max(maxRe, mc::rad2deg(std::acos(cth)));
        }
        CHECK(maxPe < 0.5);
        CHECK(maxRe < 0.5);
        std::printf("  3 open-loop integration (30 s): pos drift %.3f m, att drift %.3f deg\n", maxPe, maxRe);
    }

    // ---- 4. limits ---------------------------------------------------------------------
    CHECK(S.feasible);
    CHECK(S.maxTiltDeg <= mc::rad2deg(q.maxTilt) + 1e-6);
    CHECK(S.maxRateXYDegS <= mc::rad2deg(q.maxRollPitchRate));
    CHECK(S.maxYawRateDegS <= mc::rad2deg(q.maxYawRate));
    CHECK(S.maxYawAccDegS2 <= mc::rad2deg(q.maxYawAcc) + 1e-6);
    CHECK(S.maxSpeed <= q.maxSpeed);
    CHECK(S.maxLookDeg <= S.lookLimitDeg + 1e-6);
    CHECK(S.maxThrustToWeight <= q.thrustToWeight);
    std::printf("  4 limits: tilt %.1f/%.0f, pq %.0f/%.0f, r %.0f/%.0f deg/s, look %.1f/%.1f deg, T/W %.2f\n",
                S.maxTiltDeg, mc::rad2deg(q.maxTilt), S.maxRateXYDegS, mc::rad2deg(q.maxRollPitchRate),
                S.maxYawRateDegS, mc::rad2deg(q.maxYawRate), S.maxLookDeg, S.lookLimitDeg, S.maxThrustToWeight);

    // ---- 5. tracking --------------------------------------------------------------------
    CHECK(S.boreRms < 10.0);
    CHECK(S.pathRms < 15.0);
    std::printf("  5 tracking: boresight rms %.2f (p95 %.2f, max %.2f) m, path rms %.2f m, %d LM it, %.2f s\n",
                S.boreRms, S.boreP95, S.boreMax, S.pathRms, S.iterations, S.solveTime);

    // ---- 6. yaw locked ------------------------------------------------------------------
    {
        mc::QuadParams q2 = qp;
        q2.useYaw = false;
        VecX tv2;
        const std::vector<mc::QuadTrajectory> qd2 = mq::generateQuadTrajectories(fw, Vfw, dt, tau, beta, q2, tv2);
        const mc::QuadTrajectory& Q2 = qd2[0];
        double yawDev = 0;
        for (Index k = 0; k < Q2.t.size(); ++k) yawDev = std::max(yawDev, std::abs(Q2.yaw(k) - Q2.trackHeading(k)));
        CHECK(Q2.stats.feasible);
        CHECK(Q2.stats.maxTiltDeg <= 30.0 + 1e-6);
        CHECK(yawDev < mc::deg2rad(3));
        CHECK(Q2.stats.boreRms > S.boreRms);
        std::printf("  6 yaw locked: boresight rms %.1f m (yaw on %.1f), |yaw - track| %.2f deg\n", Q2.stats.boreRms,
                    S.boreRms, mc::rad2deg(yawDev));
    }

    // ---- 7. shapes ----------------------------------------------------------------------------
    {
        CHECK(Q.traj.drone.rows() == tv.size());
        CHECK(Q.traj.sensor.rows() == tv.size());
        CHECK(std::abs(tv(1) - dt) < 1e-12);
        CHECK(std::abs(Q.t(Q.t.size() - 1) - Lc / qp.speed) < dt + 1e-9);
        VecX tm;
        const std::vector<mc::AgentTrajectory> M = mq::quadMatchedLooks(quad, fw, Vfw, dt, qp.speed, tm);
        CHECK(M[0].drone.rows() == fw[0].drone.rows());
        CHECK((M[0].drone.row(0) - Q.traj.drone.row(0)).norm() < 1e-9);
        std::printf("  7 shapes: %lld quad steps (T %.1f s), matched looks %lld steps\n",
                    static_cast<long long>(tv.size()), Q.t(Q.t.size() - 1), static_cast<long long>(M[0].drone.rows()));
    }
    return mc::test::report("test_quadrotor");
}
