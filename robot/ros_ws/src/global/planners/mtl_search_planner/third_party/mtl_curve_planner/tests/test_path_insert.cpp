// The reallocation Path seed (curve_planning::pathInsertSeed + the prefix
// replay in initParametricSpline):
//   1. the budget repair brings the waypoint path back inside L (overrun <= 0);
//   2. the replayed prefix of the flown seed is the old curve, sample for sample;
//   3. the curvature coefficients of the hats inside the prefix are copied, so
//      the fitted seed reproduces the old curve exactly up to the first
//      re-fitted hat;
//   4. the claimed centroid is captured by the flown seed;
//   5. the seed, optimised, stays feasible (curvature, length).
#include <cmath>

#include "mtl_curve/curve_planning/init_spline.hpp"
#include "mtl_curve/curve_planning/parametric_curve.hpp"
#include "mtl_curve/curve_planning/path_insert.hpp"
#include "mtl_curve/mapgen/scenario.hpp"
#include "mtl_curve/optimization/fast_grid.hpp"
#include "mtl_curve/optimization/objective.hpp"
#include "mtl_curve/optimization/optimizer.hpp"
#include "mtl_curve/optimization/swath_kernel.hpp"
#include "mtl_curve/sensing/sweep.hpp"
#include "test_util.hpp"

using namespace mtl::curve;
using namespace mtl::curve::optimization;
using namespace mtl::curve::curve_planning;

int main() {
    PlannerParams P;
    P.cellSize = 10.0;
    P.optimizer.maxIter = 60;
    P.optimizer.exploreIter = 30;
    P.optimizer.exploreIterWarm = 20;
    P.finalize();

    const BeliefField belief = mapgen::generateBeliefMap(P.mapSize, P.cellSize, BeliefMapParams{}, P.rngSeed);
    const double h = P.droneAltitude;
    const SweepParams sw = sensing::computeSweepParams(h, P.sweep, P.gimbal, P.sensor, P.sensorTiltAngle);
    const SwathKernelSet ks = calibrateSwathKernel(h, sw, P.sensor, P.fov, P.avgDroneSpeed, P.dt, P.kernel);
    const FastGrid G = buildFastGrid(belief, 3, ks.accurate.Rk);
    const FastGrid Gx = buildFastGrid(belief, 5, ks.explore.Rk);
    const MatX zero = MatX::Zero(G.ny, G.nx), zeroX = MatX::Zero(Gx.ny, Gx.nx);
    const AgentProblem prob{&G, &ks.accurate, &zero};
    const AgentProblem probX{&Gx, &ks.explore, &zeroX};
    const Vec2 start(2000, 2000);
    const double L = P.budgetDist();
    const double capture = P.curve.captureFrac * ks.accurate.halfWidth;

    for (const EndpointMode mode : {EndpointMode::Open, EndpointMode::ReturnHome}) {
        // an optimised curve to splice into
        const InitResult init0 = initParametricSpline(start, Path2(0, 2), L, mode, std::nullopt, P.curve, prob);
        const VecX v = optimizeAgentCurve(init0.v0, init0.rep, prob, &probX, P.optimizer, P.optimizer.exploreIter,
                                          P.curve.denseStep, nullptr);
        const CurveRep& rep = init0.rep;
        const CurveSamples C = evalParametricCurve(v, rep);
        const MatX lam = agentDeposit(v, rep, G, ks.accurate);
        const MatX W = (G.prior.array() * lam.array().exp()).matrix();

        // a centroid well off the curve: the grid point farthest from it that a
        // closed loop can still reach (within 0.3 L of the start)
        Vec2 c = start;
        double far = 0.0;
        for (double x = 250; x < P.mapSize; x += 250)
            for (double y = 250; y < P.mapSize; y += 250) {
                const Vec2 q(x, y);
                if (rep.hasEq() && (q - start).norm() > 0.3 * L) continue;
                double dmin = kInf;
                for (Index j = 0; j < C.pts.rows(); ++j)
                    dmin = std::min(dmin, (Vec2(C.pts.row(j).transpose()) - q).norm());
                if (dmin > far) {
                    far = dmin;
                    c = q;
                }
            }
        const PathInsertSeed ps = pathInsertSeed(v, rep, c, ks.accurate, G, W, P.team);
        std::printf("  %-12s c=(%.0f,%.0f) %.0f m off: insert after wp %lld, detour %.0f m, overrun %.0f -> %.0f m, "
                    "%d cut(s), %zu piece(s), prefix %.0f m\n",
                    toString(mode), c.x(), c.y(), far, static_cast<long long>(ps.insertAfter), ps.detour, ps.overrun0,
                    ps.overrun, ps.cuts, ps.pieces.size(), ps.sPrefix);
        CHECK(ps.overrun <= 0.0);
        CHECK(!ps.pieces.empty());

        const Splice sp{&v, &rep, &ps.pieces};
        const InitResult seed = initParametricSpline(start, Path2(0, 2), L, mode, std::nullopt, P.curve, prob, &sp);
        CHECK(seed.splicePrefix >= ps.prefixSamples);
        CHECK(seed.splicePieceReached == static_cast<Index>(ps.pieces.size()));   // every piece flown
        // 2. the replayed prefix is the old curve
        double errPath = 0.0;
        for (Index j = 0; j < seed.splicePrefix; ++j)
            errPath = std::max(errPath, (seed.path.row(j + 1) - C.pts.row(j)).norm());
        CHECK(errPath < 1e-9);
        // 3. copied coefficients -> the fitted seed matches exactly up to the first re-fitted hat
        const double hk = rep.knots(1) - rep.knots(0);
        Index nCopied = 0;
        for (Index m = 0; m < rep.Nk; ++m)
            if (rep.knots(m) + hk <= static_cast<double>(seed.splicePrefix) * C.ds(0) + 1e-9) {
                CHECK_NEAR(seed.v0(m), v(m), 1e-15);
                ++nCopied;
            }
        if (seed.splicePrefix > 0) CHECK_NEAR(seed.v0(rep.Nk), v(rep.Nk), 1e-15);
        const CurveSamples Cs = evalParametricCurve(seed.v0, seed.rep);
        const double sExact = nCopied > 0 ? rep.knots(nCopied) - hk : 0.0;   // first re-fitted hat's support
        double errFit = 0.0;
        for (Index j = 0; j < Cs.pts.rows(); ++j)
            if (Cs.s(j) + 0.5 * Cs.ds(j) <= sExact) errFit = std::max(errFit, (Cs.pts.row(j) - C.pts.row(j)).norm());
        CHECK(errFit < 1e-6);
        // 4. the centroid is captured
        bool hitC = false;
        for (Index j = 0; j < seed.path.rows(); ++j) hitC = hitC || (Vec2(seed.path.row(j).transpose()) - c).norm() < capture + 1.0;
        double dminC = kInf;
        for (Index j = 0; j < seed.path.rows(); ++j) dminC = std::min(dminC, (Vec2(seed.path.row(j).transpose()) - c).norm());
        Index jmin = 0;
        for (Index j = 0; j < seed.path.rows(); ++j)
            if ((Vec2(seed.path.row(j).transpose()) - c).norm() <= dminC + 1e-9) jmin = j;
        std::printf("  %-12s closest approach to the centroid %.0f m (capture %.0f m) at sample %lld of %lld\n",
                    toString(mode), dminC, capture, static_cast<long long>(jmin), static_cast<long long>(seed.path.rows()));
        CHECK(hitC);
        const double Jold = objectiveResidualBelief(v, rep, prob);
        const double Jseed = objectiveResidualBelief(seed.v0, seed.rep, prob);
        // 5. optimised seed stays feasible
        OptimizeInfo info;
        const VecX v2 = optimizeAgentCurve(seed.v0, seed.rep, prob, &probX, P.optimizer, P.optimizer.exploreIterWarm,
                                           P.curve.denseStep, &info);
        const Index Md = static_cast<Index>(std::lround(L / P.curve.denseStep));
        const CurveSamples Cd = evalParametricCurve(v2, seed.rep, Md);
        CHECK(Cd.kappa.cwiseAbs().maxCoeff() <= rep.kappaMax + 1e-4);
        CHECK(std::abs(Cd.ds.sum() - L) <= 0.002 * L);
        if (rep.hasEq()) CHECK((Cd.endPt - *rep.pGoal).norm() < 1.0);
        std::printf("  %-12s prefix replay err %.2e m, fitted-prefix err %.2e m (%lld hats copied); J old %.5f, "
                    "seed %.5f -> optimised %.5f\n",
                    toString(mode), errPath, errFit, static_cast<long long>(nCopied), Jold, Jseed, info.J);
    }
    {
        // a claim no closed loop of length L can make: reported, not hidden
        const InitResult init0 = initParametricSpline(start, Path2(0, 2), L, EndpointMode::ReturnHome, std::nullopt,
                                                      P.curve, prob);
        const MatX lam = agentDeposit(init0.v0, init0.rep, G, ks.accurate);
        const MatX W = (G.prior.array() * lam.array().exp()).matrix();
        const Vec2 cFar = start + Vec2(0.6 * L / std::sqrt(2.0), 0.6 * L / std::sqrt(2.0));
        const PathInsertSeed ps = pathInsertSeed(init0.v0, init0.rep, cFar, ks.accurate, G, W, P.team);
        std::printf("  unreachable  overrun %.0f -> %.0f m\n", ps.overrun0, ps.overrun);
        CHECK(ps.overrun > 0.0);
    }
    return test::report("test_path_insert");
}
