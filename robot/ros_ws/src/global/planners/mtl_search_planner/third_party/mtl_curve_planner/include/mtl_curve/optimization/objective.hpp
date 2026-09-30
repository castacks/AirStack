// =============================================================================
//  mtl_curve/optimization/objective.hpp
//
//  The fast team residual of one agent's curve, its analytic gradient, and the
//  nonlinear constraints - the port of objectiveResidualBelief.m and
//  constraintCurvatureLength.m.
//
//    J(v) = sum_g prior_g exp( LambdaOther_g + Lambda_a(g; v) )
//           + lambda_kappa sum_j max(0, |kappa_j| - kappa_max)^2 ds_j   (B-spline only)
//
//  exp(sum of log-miss) is exactly the plan's joint team miss probability
//  prod_a (1 - P_det_a): overlapping swaths are penalised automatically, with
//  no ad-hoc de-confliction.  LambdaOther (every OTHER agent's deposit) is
//  frozen, which makes the team solve Gauss-Seidel.
//
//  Constraints, cin <= 0, ceq == 0, Jacobians nv-by-m (fmincon's layout):
//    BSpline    ceq = L(P)/budget - 1  (Simpson);
//               cin = [kappa/kappa_c - 1; -kappa/kappa_c - 1] every ~5 m
//    Curvature  length and curvature hold by construction (cin empty); in the
//               closed modes ceq = (gamma(L) - pGoal)/L (2 equations).
// =============================================================================
#ifndef MTLC_OPTIMIZATION_OBJECTIVE_HPP
#define MTLC_OPTIMIZATION_OBJECTIVE_HPP

#include "mtl_curve/types.hpp"

namespace mtl::curve::optimization {

/// One agent's fast problem: the grid, its kernel and the other agents' frozen
/// log-miss.  All pointers are non-owning and must outlive the call.
struct AgentProblem {
    const FastGrid*    G = nullptr;
    const SwathKernel* K = nullptr;
    const MatX*        lambdaOther = nullptr;  ///< ny-by-nx, or nullptr = zeros
};

/// @param grad  filled with dJ/dv when non-null
/// @param lambdaOut this agent's deposit, when non-null
double objectiveResidualBelief(const VecX& v, const CurveRep& rep, const AgentProblem& prob,
                               VecX* grad = nullptr, MatX* lambdaOut = nullptr);

/// @param gcin / gceq  nv-by-m Jacobians when non-null
void constraintCurvatureLength(const VecX& v, const CurveRep& rep, VecX& cin, VecX& ceq,
                               MatX* gcin = nullptr, MatX* gceq = nullptr);

/// This agent's deposit on the grid (the Lambda_a of the objective).
MatX agentDeposit(const VecX& v, const CurveRep& rep, const FastGrid& G, const SwathKernel& K);

}  // namespace mtl::curve::optimization

#endif  // MTLC_OPTIMIZATION_OBJECTIVE_HPP
