// Acceptance checks 1-3 of the plan (the C++ counterpart of
// test_curve_planner.m): one agent through every representation x endpoint
// mode, checked on the FLOWN trajectory (2 m steps):
//   1. max |kappa| <= 1/minTurnRadius + 1e-4
//   2. |L - budget| <= 0.2 %
//   3. return_home |gamma(1) - start| < 1 m, fixed_dest |gamma(1) - dest| < 1 m
// and the optimiser never makes the objective worse than the seed.
#include <cmath>

#include "mtl_curve/curve_planning/init_spline.hpp"
#include "mtl_curve/mapgen/scenario.hpp"
#include "mtl_curve/optimization/fast_grid.hpp"
#include "mtl_curve/optimization/objective.hpp"
#include "mtl_curve/optimization/optimizer.hpp"
#include "mtl_curve/optimization/swath_kernel.hpp"
#include "mtl_curve/sensing/sweep.hpp"
#include "mtl_curve/trajectory/curve_trajectory.hpp"
#include "test_util.hpp"

using namespace mtl::curve;
using namespace mtl::curve::optimization;

int main() {
    PlannerParams P;
    P.cellSize = 10.0;
    P.optimizer.maxIter = 80;       // a test, not a mission
    P.optimizer.exploreIter = 40;
    P.finalize();

    const BeliefField belief = mapgen::generateBeliefMap(P.mapSize, P.cellSize, BeliefMapParams{}, P.rngSeed);
    const double h = P.droneAltitude;
    const SweepParams sw = sensing::computeSweepParams(h, P.sweep, P.gimbal, P.sensor, P.sensorTiltAngle);
    const SwathKernelSet ks = calibrateSwathKernel(h, sw, P.sensor, P.fov, P.avgDroneSpeed, P.dt, P.kernel);
    const FastGrid G = buildFastGrid(belief, 3, ks.accurate.Rk);
    const FastGrid Gx = buildFastGrid(belief, 5, ks.explore.Rk);
    const AgentProblem prob{&G, &ks.accurate, nullptr};
    const AgentProblem probX{&Gx, &ks.explore, nullptr};

    const Vec2 start(2000, 2000), dest(4500, 4500);
    const double L = P.budgetDist();
    std::printf("  %-10s %-12s %9s %9s %11s %9s %9s\n", "rep", "mode", "J0", "J", "max k", "dL [%]", "end err");
    for (const CurveRepresentation type : {CurveRepresentation::Curvature, CurveRepresentation::BSpline}) {
        for (const EndpointMode mode : {EndpointMode::Open, EndpointMode::ReturnHome, EndpointMode::FixedDest}) {
            CurveParams cp = P.curve;
            cp.representation = type;
            const curve_planning::InitResult init =
                curve_planning::initParametricSpline(start, Path2(0, 2), L, mode,
                                                     mode == EndpointMode::FixedDest ? std::optional<Vec2>(dest)
                                                                                     : std::nullopt,
                                                     cp, prob);
            CHECK_NEAR((init.path.row(0).transpose() - start).norm(), 0.0, 1e-12);
            OptimizeInfo info;
            const VecX v = optimizeAgentCurve(init.v0, init.rep, prob, &probX, P.optimizer, P.optimizer.exploreIter,
                                              P.curve.denseStep, &info);
            VecX tv;
            const std::vector<AgentTrajectory> tr = trajectory::generateCurveTrajectories(
                {v}, {init.rep}, {h}, {sw}, P.avgDroneSpeed, P.dt, P.gimbal, tv);
            const AgentTrajectory& t = tr[0];
            const double dL = 100.0 * (t.length - L) / L;
            double endErr = 0.0;
            if (mode == EndpointMode::ReturnHome) endErr = (t.endPt - start).norm();
            if (mode == EndpointMode::FixedDest) endErr = (t.endPt - dest).norm();
            std::printf("  %-10s %-12s %9.5f %9.5f %11.6f %9.4f %9.4f\n", toString(type), toString(mode), info.J0,
                        info.J, t.maxKappaDiscrete, dL, endErr);
            CHECK(t.maxKappaDiscrete <= P.curve.maxCurvature + 1e-4);
            CHECK(std::abs(dL) <= 0.2);
            CHECK(endErr < 1.0);
            CHECK((t.startPt - start).norm() < 1e-6);
            CHECK(info.J <= info.J0 + 1e-9);
            CHECK(t.drone.allFinite() && t.sensor.allFinite() && t.rpy.allFinite());
            CHECK(t.maxGimbalCmdDeg <= rad2deg(P.gimbal.gimbalMax) + 1e-9);
        }
    }
    return test::report("test_optimizer");
}
