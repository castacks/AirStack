// =============================================================================
//  mtl_curve/optimization/optimizer.hpp
//
//  Local optimisation of one agent's curve against the team - the port of
//  optimizeAgentCurve.m (its 'internal' solver; MATLAB's optional fmincon path
//  has no C++ counterpart).
//
//      min_v  J(v)  s.t.  lb <= v <= ub,  cin(v) <= 0,  ceq(v) == 0
//
//  * L-BFGS (two-loop recursion, Armijo backtracking) on the smooth box
//    transform  v = mid + half * sin(z)  (fixed variables held, unbounded ones
//    passed through);
//  * an augmented-Lagrangian outer loop for the nonlinear constraints (only
//    present for B-splines, or the closed endpoint modes of the curvature
//    representation);
//  * GRADUATED OPTIMISATION: with an exploration problem supplied and
//    exploreIter > 0, a first stage on the optimistic long-tailed kernel over
//    the coarse grid, then the accurate one;
//  * a FEASIBILITY PROJECTION: Gauss-Newton minimum-norm steps onto the
//    equality constraints evaluated on the DENSE (flown) discretisation, so the
//    endpoint / length tolerances hold on the trajectory that is flown.
// =============================================================================
#ifndef MTLC_OPTIMIZATION_OPTIMIZER_HPP
#define MTLC_OPTIMIZATION_OPTIMIZER_HPP

#include "mtl_curve/optimization/objective.hpp"
#include "mtl_curve/params.hpp"
#include "mtl_curve/types.hpp"

namespace mtl::curve::optimization {

/// @param explore  the exploration problem (coarse grid + explore kernel), or
///                 nullptr to skip that stage
/// @param exploreIter iterations of the exploration stage (0 = skip)
/// @param denseStep [m] step of the flown discretisation for the projection
VecX optimizeAgentCurve(const VecX& v0, const CurveRep& rep, const AgentProblem& prob,
                        const AgentProblem* explore, const OptimizerParams& opts, int exploreIter,
                        double denseStep, OptimizeInfo* info = nullptr);

}  // namespace mtl::curve::optimization

#endif  // MTLC_OPTIMIZATION_OPTIMIZER_HPP
