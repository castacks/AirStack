// =============================================================================
//  mtl_curve/optimization/swath_kernel.hpp
//
//  The fast objective's look model, CALIBRATED from the real sensor - the port
//  of calibrateSwathKernel.m and swathKernelDeposit.m.
//
//  WHY (deviation from the plan's PHASE 3).  The plan scores a pixel with ONE
//  look, P_det(d_perp).  The evaluator multiplies (1 - P) over EVERY time step
//  the pixel is inside the swept footprint, so a pixel under the track gets
//  hundreds of looks and one near the edge a handful; and a forward-tilted
//  mount looks h tan(tau) AHEAD, which swings round in a turn.  So:
//
//   1. k(a, d) [log-miss per metre flown] at along-track a / cross-track d from
//      the aircraft = the per-look log(1 - P) averaged over the sweep phase,
//      divided by V*dt.  Same footprint, sigmoid, beta and pOut as
//      eval::computeResidualBelief.  A curve deposits
//          Lambda(x) = sum_j ds_j k( R(theta_j)' (x - gamma_j) )
//      so the kernel turns with the aircraft, deposits ADD (self-overlap and
//      other agents are independent looks - the plan's joint objective), and
//      the gradient w.r.t. position AND heading is analytic.
//   2. Hard edges (footprint circle, beta sphere) become logistic steps of
//      width edgeWidth centred edgeInset INSIDE the physical edge: smooth for
//      the optimiser, never claiming coverage the sensor cannot deliver.
//   3. Straight-line correction: a long straight pass is simulated with the
//      exact model and averaged along track in PROBABILITY,
//          Lambda_eff(d) = log(mean_a exp(Lambda(a, d)))   (saturated at -lambdaCap)
//      and each kernel row is scaled by c(d) = min(1, Lambda_eff(d) / int k da),
//      so on a straight line the fast model reproduces Lambda_eff or is more
//      conservative, never better.
//
//  The EXPLORATION kernel is the accurate one plus tailWeight times a
//  Gaussian(tailSigma) blur of it: deliberately optimistic and long-reaching,
//  so belief beyond the real reach still pulls on the curve.  The optimiser
//  runs a first stage on it and refines on the accurate one.
// =============================================================================
#ifndef MTLC_OPTIMIZATION_SWATH_KERNEL_HPP
#define MTLC_OPTIMIZATION_SWATH_KERNEL_HPP

#include "mtl_curve/params.hpp"
#include "mtl_curve/types.hpp"

namespace mtl::curve::optimization {

/// @param h     [m] altitude
/// @param sweep the gimbal sweep (sensing::computeSweepParams)
SwathKernelSet calibrateSwathKernel(double h, const SweepParams& sweep,
                                    const SensorModelParams& sensor, double fov, double V,
                                    double dt, const KernelParams& opts);

/// Lambda(x) = sum_j ds_j k(a_j(x), d_j(x)) on the grid (ny-by-nx, <= 0).
/// pts: aircraft positions, tan: unit tangents, ds: arc weights (all M long).
MatX swathKernelDeposit(const Path2& pts, const Path2& tan, const VecX& ds, const FastGrid& G,
                        const SwathKernel& K);

/// Gradient of J = sum_x W(x) exp(Lambda_total(x)) given W = dJ/dLambda
/// (= prior .* exp(Lambda_total)), w.r.t. each sample's position (gp, M-by-2),
/// heading (gth) and arc weight (gds):
///     dk/dp = -(k_a t + k_d n),   dk/dtheta = k_a d - k_d a.
/// yield (optional) = -ds .* gds: first-order information each sample collects.
void swathKernelGradient(const Path2& pts, const Path2& tan, const VecX& ds, const FastGrid& G,
                         const SwathKernel& K, const MatX& W, Path2& gp, VecX& gth, VecX& gds,
                         VecX* yield = nullptr);

/// Lambda at arbitrary query points (not on a grid): the per-cell coverage.
VecX swathKernelAtPoints(const Path2& pts, const Path2& tan, const VecX& ds, const SwathKernel& K,
                         const Path2& query);

}  // namespace mtl::curve::optimization

#endif  // MTLC_OPTIMIZATION_SWATH_KERNEL_HPP
