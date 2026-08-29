// test_formation_cost.cpp
//
// Verifies the new formation-flight cost J_form added to
// SolverLBFGS::evaluateObjectiveAndGradientFused().
//
// Two cases:
//   1. ZeroWeightBitIdentical  — formation_weight = 0 must yield exactly the
//      same objective and gradient as a baseline build with no neighbors,
//      regardless of whether neighbors are configured.
//   2. GradientMatchesFiniteDiff — with formation_weight > 0 and a non-trivial
//      neighbor, the analytic gradient must match a central-difference
//      gradient to relative tolerance < 1e-5 in every coordinate direction.

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <mighty/lbfgs.hpp>
#include <mighty/lbfgs_solver.hpp>

namespace {

vec_Vecf<3> makeWaypoints() {
  return {Vecf<3>{0.0, 0.0, 0.0}, Vecf<3>{0.0, 2.0, 0.0}, Vecf<3>{0.0, 4.0, 0.0},
          Vecf<3>{5.0, 5.0, 0.0}};
}

state makeState(double x, double y, double z) {
  state s;
  s.setPos(x, y, z);
  s.setVel(0.0, 0.0, 0.0);
  s.setAccel(0.0, 0.0, 0.0);
  return s;
}

/// Build an analytic-mode dynTraj that drifts linearly across the world.
/// Uses ExprTk expression strings so we don't depend on the dynTraj Quintic
/// constructor (which has a coefficient-ordering bug w.r.t. poly5_abs).
std::shared_ptr<dynTraj> makeNeighborAnalytic(int id) {
  auto d = std::make_shared<dynTraj>();
  d->mode = dynTraj::Mode::Analytic;
  d->poly_start_time = 0.0;
  d->poly_end_time = 30.0;
  d->traj_x = "2.0 + 0.1*t";
  d->traj_y = "1.5 + 0.2*t";
  d->traj_z = "0.0";
  d->traj_vx = "0.1";
  d->traj_vy = "0.2";
  d->traj_vz = "0.0";
  d->id = id;
  d->is_agent = true;
  EXPECT_TRUE(d->compileAnalytic());
  return d;
}

/// Stationary neighbor at a fixed position (analytic constant expressions).
std::shared_ptr<dynTraj> makeStationaryNeighbor(int id, const Eigen::Vector3d& pos) {
  auto d = std::make_shared<dynTraj>();
  d->mode = dynTraj::Mode::Analytic;
  d->poly_start_time = 0.0;
  d->poly_end_time = 30.0;
  d->traj_x = std::to_string(pos.x());
  d->traj_y = std::to_string(pos.y());
  d->traj_z = std::to_string(pos.z());
  d->traj_vx = "0.0";
  d->traj_vy = "0.0";
  d->traj_vz = "0.0";
  d->id = id;
  d->is_agent = true;
  EXPECT_TRUE(d->compileAnalytic());
  return d;
}

/// Default planner_params used by both tests. Caller may override formation
/// fields before calling buildSolver().
lbfgs::planner_params_t defaultParams() {
  lbfgs::planner_params_t p;
  p.verbose = false;
  p.V_max = 2.0;
  p.A_max = 10.0;
  p.J_max = 100.0;
  p.num_perturbation = 8;
  p.r_max = 1.0;
  p.dyn_weight = 0.0;
  p.time_weight = 1.0;
  p.pos_anchor_weight = 1.0;
  p.jerk_weight = 1.0;
  p.stat_weight = 1.0;
  p.dyn_constr_vel_weight = 1.0;
  p.dyn_constr_acc_weight = 1.0;
  p.dyn_constr_jerk_weight = 1.0;
  p.dyn_constr_bodyrate_weight = 1.0;
  p.dyn_constr_tilt_weight = 1.0;
  p.dyn_constr_thrust_weight = 1.0;
  p.Co = 0.2;
  p.Cw = 1.0;
  p.BIG = 1e8;
  p.dc = 0.01;
  p.integral_resolution = 30;
  p.hinge_mu = 1e-2;
  p.omega_max = 0.5;
  p.tilt_max_rad = 0.174533;
  p.f_min = 0.0;
  p.f_max = 10.0;
  p.mass = 1.0;
  p.g = 9.81;
  return p;
}

/// Variant with all weights set to zero — used to isolate J_form for debugging.
lbfgs::planner_params_t isolatedFormationParams() {
  auto p = defaultParams();
  p.dyn_weight = 0.0;
  p.time_weight = 0.0;
  p.pos_anchor_weight = 0.0;
  p.jerk_weight = 0.0;
  p.stat_weight = 0.0;
  p.dyn_constr_vel_weight = 0.0;
  p.dyn_constr_acc_weight = 0.0;
  p.dyn_constr_jerk_weight = 0.0;
  p.dyn_constr_bodyrate_weight = 0.0;
  p.dyn_constr_tilt_weight = 0.0;
  p.dyn_constr_thrust_weight = 0.0;
  return p;
}

/// Build a fully prepared solver with the given params and the given
/// trajectory list (possibly empty). Returns the solver and writes the first
/// initial guess into z0_out.
std::shared_ptr<lbfgs::SolverLBFGS> buildSolver(
    const lbfgs::planner_params_t& p,
    const std::vector<std::shared_ptr<dynTraj>>& trajs,
    Eigen::VectorXd& z0_out) {
  auto solver = std::make_shared<lbfgs::SolverLBFGS>();
  solver->initializeSolver(p);

  vec_Vecf<3> wps = makeWaypoints();
  std::vector<LinearConstraint3D> empty_corridor;  // no static planes for this test

  state s0 = makeState(0.0, 0.0, 0.0);
  state sf = makeState(5.0, 5.0, 0.0);

  double dummy_t = 0.0;
  solver->prepareSolverForReplan(0.0, wps, empty_corridor, trajs, s0, sf, dummy_t, false);

  const auto& guesses = solver->getInitialGuesses();
  z0_out = guesses.empty() ? Eigen::VectorXd() : guesses.front();
  return solver;
}

}  // namespace

// With formation_weight = 0 the new cost block must short-circuit and produce
// numerically identical objective + gradient to a build that has no neighbors
// configured at all. This is the zero-impact regression invariant.
TEST(FormationCost, ZeroWeightBitIdentical) {
  // (a) Baseline: no formation, no neighbors.
  auto p_base = defaultParams();
  Eigen::VectorXd z0_base;
  auto solver_base = buildSolver(p_base, {}, z0_base);
  ASSERT_GT(z0_base.size(), 0);

  Eigen::VectorXd g_base;
  const double f_base = solver_base->evaluateObjectiveAndGradientFused(z0_base, g_base);

  // (b) Same params but with formation_weight = 0 AND a configured neighbor
  // present in the trajectory list. setFormationNeighbors should still
  // short-circuit because the weight is zero.
  auto p_zero = defaultParams();
  p_zero.formation_weight = 0.0;
  p_zero.formation_neighbor_ids = {42};
  p_zero.formation_offsets = {Eigen::Vector3d(1.5, -0.3, 0.0)};
  std::vector<std::shared_ptr<dynTraj>> trajs{makeNeighborAnalytic(42)};

  Eigen::VectorXd z0_form;
  auto solver_form = buildSolver(p_zero, trajs, z0_form);
  ASSERT_EQ(z0_form.size(), z0_base.size());

  Eigen::VectorXd g_form;
  const double f_form = solver_form->evaluateObjectiveAndGradientFused(z0_form, g_form);

  // The two solvers may produce slightly different initial guesses depending
  // on internal state, so first check the initial guess vector itself is
  // identical (it should be — same waypoints, same problem geometry).
  ASSERT_TRUE(z0_form.isApprox(z0_base, 0.0))
      << "Initial guesses differ; cannot compare bit-identically";

  EXPECT_EQ(f_form, f_base) << "Objective changed when formation_weight = 0";
  EXPECT_EQ(g_form.size(), g_base.size());
  for (int i = 0; i < g_form.size(); ++i) {
    EXPECT_EQ(g_form[i], g_base[i])
        << "Gradient component " << i << " differs at zero formation weight";
  }
}

// Stationary neighbor isolation — eliminates the neighbor-motion time path
// entirely. If this passes but moving-neighbor case fails, the bug is in
// the neighbor-motion path. If both fail, the bug is in CP/τ/dt scaling.
TEST(FormationCost, IsolatedStationaryNeighbor) {
  auto p = isolatedFormationParams();
  p.formation_weight = 1.0;
  p.formation_neighbor_ids = {42};
  p.formation_offsets = {Eigen::Vector3d::Zero()};

  std::vector<std::shared_ptr<dynTraj>> trajs{
      makeStationaryNeighbor(42, Eigen::Vector3d{2.0, 2.0, 0.0})};

  Eigen::VectorXd z0;
  auto solver = buildSolver(p, trajs, z0);
  ASSERT_GT(z0.size(), 0);

  Eigen::VectorXd g;
  const double f = solver->evaluateObjectiveAndGradientFused(z0, g);
  std::cout << "[stationary] f = " << f << ", |g| = " << g.norm() << std::endl;

  const double err = solver->checkGradCoordinates(z0, static_cast<int>(z0.size()),
                                                  /*eps=*/1e-6, /*seed=*/17);
  EXPECT_LT(err, 1e-5) << "Stationary J_form per-coord rel err = " << err;
}

// Isolated J_form gradient check — all other weights zero so the only cost
// term in the fused evaluator is formation_weight_ * J_form. Pinpoints whether
// any gradient mismatch is in the J_form derivation specifically.
TEST(FormationCost, IsolatedGradientMatchesFiniteDiff) {
  auto p = isolatedFormationParams();
  p.formation_weight = 1.0;
  p.formation_neighbor_ids = {42};
  p.formation_offsets = {Eigen::Vector3d(1.5, -0.3, 0.0)};

  std::vector<std::shared_ptr<dynTraj>> trajs{makeNeighborAnalytic(42)};

  Eigen::VectorXd z0;
  auto solver = buildSolver(p, trajs, z0);
  ASSERT_GT(z0.size(), 0);

  Eigen::VectorXd g;
  const double f = solver->evaluateObjectiveAndGradientFused(z0, g);
  std::cout << "[isolated] f = " << f << ", |g| = " << g.norm() << std::endl;

  const double err = solver->checkGradCoordinates(z0,
                                                  /*max_coords=*/static_cast<int>(z0.size()),
                                                  /*eps=*/1e-6, /*seed=*/17);
  EXPECT_LT(err, 1e-5) << "Isolated J_form per-coord rel err = " << err;
}

// With formation_weight > 0 and a moving neighbor, the analytic gradient
// must match central-difference finite differences in every coordinate.
TEST(FormationCost, GradientMatchesFiniteDiff) {
  auto p = defaultParams();
  p.formation_weight = 5.0;
  p.formation_neighbor_ids = {42};
  p.formation_offsets = {Eigen::Vector3d(1.5, -0.3, 0.0)};

  std::vector<std::shared_ptr<dynTraj>> trajs{makeNeighborAnalytic(42)};

  Eigen::VectorXd z0;
  auto solver = buildSolver(p, trajs, z0);
  ASSERT_GT(z0.size(), 0);

  // Sanity: J_form must actually fire — the objective with the neighbor
  // present must be strictly larger than the same problem with formation
  // disabled. Note we have to call the FUSED entry point because that's the
  // one L-BFGS uses (and the only one where J_form is summed in).
  Eigen::VectorXd g_with;
  const double f_with = solver->evaluateObjectiveAndGradientFused(z0, g_with);

  auto p_off = p;
  p_off.formation_weight = 0.0;
  Eigen::VectorXd z0_off;
  auto solver_off = buildSolver(p_off, trajs, z0_off);
  Eigen::VectorXd g_off;
  const double f_off = solver_off->evaluateObjectiveAndGradientFused(z0_off, g_off);
  EXPECT_GT(f_with, f_off) << "Formation cost did not contribute to the objective";

  // Coordinate-by-coordinate FD check (covers J_form's CP-path AND its
  // T-path together with everything else in the fused evaluator).
  const double err = solver->checkGradCoordinates(z0,
                                                  /*max_coords=*/static_cast<int>(z0.size()),
                                                  /*eps=*/1e-6, /*seed=*/17);
  EXPECT_LT(err, 1e-5) << "Worst per-coord rel err = " << err;

  // Random-direction directional check as a second independent FD test.
  const double err_dir = solver->checkGradDirectional(z0,
                                                      /*num_dirs=*/16,
                                                      /*eps=*/1e-6, /*seed=*/23);
  EXPECT_LT(err_dir, 1e-5) << "Worst directional rel err = " << err_dir;
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
