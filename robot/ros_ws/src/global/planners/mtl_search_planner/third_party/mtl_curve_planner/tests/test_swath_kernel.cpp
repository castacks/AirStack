// The calibrated look kernel and the fast objective: the kernel never claims
// more than the real sensor on a straight pass, it respects the physical
// reach, the exploration kernel reaches further, the grids conserve mass, and
// the objective's analytic gradient (both representations) matches finite
// differences.
#include <cmath>
#include <random>

#include "mtl_curve/curve_planning/endpoint_constraints.hpp"
#include "mtl_curve/curve_planning/parametric_curve.hpp"
#include "mtl_curve/mapgen/scenario.hpp"
#include "mtl_curve/optimization/fast_grid.hpp"
#include "mtl_curve/optimization/objective.hpp"
#include "mtl_curve/optimization/swath_kernel.hpp"
#include "mtl_curve/sensing/sweep.hpp"
#include "test_util.hpp"

using namespace mtl::curve;
using namespace mtl::curve::optimization;

int main() {
    PlannerParams P;
    P.cellSize = 10.0;
    P.finalize();

    for (const double tiltDeg : {0.0, 50.0}) {
        const double tau = deg2rad(tiltDeg);
        const SweepParams sw = sensing::computeSweepParams(300.0, P.sweep, P.gimbal, P.sensor, tau);
        CHECK(sw.peakRate <= P.gimbal.gimbalRate * 0.95 + 1e-12);
        CHECK(sw.alphaMax <= P.gimbal.gimbalMax);
        CHECK(std::sqrt(sw.standOff * sw.standOff + sw.crossMax * sw.crossMax + 300.0 * 300.0) <=
              P.sweep.rangeMargin * P.sensor.beta + 1e-6);
        const SwathKernelSet ks = calibrateSwathKernel(300.0, sw, P.sensor, P.fov, P.avgDroneSpeed, P.dt, P.kernel);
        const SwathKernel& K = ks.accurate;
        std::printf("  tilt %2.0f deg: half-width %.0f m, reach %.0f m, straight-line err %.3f\n", tiltDeg,
                    K.halfWidth, K.reach, K.fitMaxErrP);
        CHECK(K.halfWidth > 400.0 && K.halfWidth <= K.reach);
        CHECK(K.T.maxCoeff() <= 0.0);
        // never materially better than the real sensor on a straight pass: the
        // only optimism allowed is the logistic edge's last few metres (a 6 m
        // wide step centred 12 m inside the edge still has a tail past it)
        double worstOptimism = 0.0;
        for (Index i = 0; i < K.d.size(); ++i)
            worstOptimism = std::max(worstOptimism, std::exp(K.lambdaEff(i)) - std::exp(K.lambdaModel(i)));
        std::printf("               worst optimism %.4f (in miss probability)\n", worstOptimism);
        CHECK(worstOptimism < 0.05);
        // and nothing beyond the physical reach (plus the table's smoothing)
        for (Index i = 0; i < K.d.size(); ++i)
            if (K.d(i) > K.reach + 10.0) CHECK(1.0 - std::exp(K.lambdaModel(i)) < 1e-3);
        // a forward-tilted mount looks ahead: little or nothing behind
        if (tiltDeg > 0.0) CHECK(K.aLo > -100.0);
        CHECK(ks.explore.Rk > K.Rk);
        CHECK(ks.explore.T.minCoeff() <= K.T.minCoeff() + 1e-12);
    }

    // --- grids conserve mass --------------------------------------------------
    const BeliefField belief = mapgen::generateBeliefMap(P.mapSize, P.cellSize, BeliefMapParams{}, 21);
    const SweepParams sw = sensing::computeSweepParams(300.0, P.sweep, P.gimbal, P.sensor, P.sensorTiltAngle);
    const SwathKernelSet ks = calibrateSwathKernel(300.0, sw, P.sensor, P.fov, P.avgDroneSpeed, P.dt, P.kernel);
    const FastGrid G = buildFastGrid(belief, 3, ks.accurate.Rk);
    CHECK_NEAR(G.prior.sum(), 1.0, 1e-12);
    CHECK_NEAR(G.hg, 30.0, 1e-9);
    {
        CellSet cells;
        cells.centers.resize(3, 2);
        cells.centers << 100, 100, 2500, 2500, 4900, 300;
        cells.mass = VecX::Constant(3, 2.0);
        const FastGrid R = rasterizeCells(cells, P.mapSize, 25.0, 200.0, 100.0);
        CHECK_NEAR(R.prior.sum(), 1.0, 1e-12);
        CHECK_NEAR(R.prior.block(0, 0, 8, 8).sum(), 1.0 / 3.0, 1e-12);
    }

    // --- the deposit is linear in ds and symmetric in the sweep --------------
    {
        Path2 pts(1, 2), tan(1, 2);
        pts << 2500, 2500;
        tan << 1, 0;
        const MatX L1 = swathKernelDeposit(pts, tan, VecX::Constant(1, 1.0), G, ks.accurate);
        const MatX L2 = swathKernelDeposit(pts, tan, VecX::Constant(1, 2.0), G, ks.accurate);
        CHECK((L2 - 2.0 * L1).cwiseAbs().maxCoeff() < 1e-12);
        CHECK(L1.minCoeff() < 0.0);
    }

    // --- objective gradient against finite differences ----------------------
    std::mt19937_64 rng(11);
    std::normal_distribution<double> nd(0.0, 1.0);
    MatX other = MatX::Zero(G.ny, G.nx);
    {
        Path2 pts(40, 2), tan(40, 2);
        for (Index i = 0; i < 40; ++i) {
            pts(i, 0) = 1000.0 + 50.0 * static_cast<double>(i);
            pts(i, 1) = 1200.0;
            tan(i, 0) = 1.0;
            tan(i, 1) = 0.0;
        }
        other = swathKernelDeposit(pts, tan, VecX::Constant(40, 50.0), G, ks.accurate);
    }
    CurveParams cp = P.curve;
    cp.curvaturePenaltyWeight = 0.0;
    for (const CurveRepresentation type : {CurveRepresentation::Curvature, CurveRepresentation::BSpline}) {
        const CurveRep rep = curve_planning::enforceEndpointConstraints(type, 5000.0, Vec2(2000, 2000),
                                                                        EndpointMode::Open, std::nullopt, cp);
        VecX v(rep.nv);
        if (type == CurveRepresentation::Curvature) {
            for (Index i = 0; i < rep.Nk; ++i) v(i) = 0.004 * std::sin(0.37 * static_cast<double>(i));
            v(rep.Nk) = 0.4;
        } else {
            Index nf = rep.nv / 2;
            for (Index k = 0; k < nf; ++k) {
                const double t = static_cast<double>(k + 1) / static_cast<double>(nf);
                v(k) = 2000.0 + 900.0 * std::sin(kPi * t);
                v(nf + k) = 2000.0 + 1500.0 * t;
            }
        }
        const AgentProblem prob{&G, &ks.accurate, &other};
        VecX g;
        const double J = objectiveResidualBelief(v, rep, prob, &g);
        CHECK(J > 0.0 && J < 1.0);
        double worst = 0.0;
        for (int trial = 0; trial < 4; ++trial) {
            VecX d(rep.nv);
            for (Index i = 0; i < rep.nv; ++i) d(i) = nd(rng);
            if (type == CurveRepresentation::Curvature) d.head(rep.Nk) *= 1e-3;
            else d *= 10.0;
            const double e = 1e-4;
            const double fd = (objectiveResidualBelief(v + e * d, rep, prob) - objectiveResidualBelief(v - e * d, rep, prob)) / (2 * e);
            worst = std::max(worst, std::abs(fd - g.dot(d)) / std::max(1e-6, std::abs(fd)));
        }
        std::printf("  objective gradient (%s): worst relative error %.2e\n", toString(type), worst);
        // bilinear table derivatives are piecewise constant: agreement to ~1e-2
        CHECK(worst < 2e-2);
    }

    return test::report("test_swath_kernel");
}
