// test_gradient_check.cpp
//
// Verifies that the analytic gradient produced by
// SolverLBFGS::evaluateObjectiveAndGradientFused() matches a finite-difference
// approximation on the same baseline problem used by test_lbfgs_solver.cpp.
//
// We use the solver's own first initial guess (built by prepareSolverForReplan)
// as the test point so the dimension always matches the current K_ layout.

#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <mighty/lbfgs.hpp>
#include <mighty/lbfgs_solver.hpp>

namespace {

/// Toy static-constraint set: 3 segments, each with one plane.
/// Mirrors makeSampleStaticConstraints() in src/test/test_lbfgs_solver.cpp.
std::vector<LinearConstraint3D> makeSampleStaticConstraints() {
  std::vector<LinearConstraint3D> constraints;
  constraints.reserve(3);

  {
    Eigen::Matrix<double, 1, 3> A0;
    A0 << -0.5, 0.0, 0.1;
    Eigen::VectorXd b0(1);
    b0 << -0.5;
    constraints.emplace_back(A0, b0);
  }
  {
    Eigen::Matrix<double, 1, 3> A1;
    A1 << -0.5, 0.3, 0.01;
    Eigen::VectorXd b1(1);
    b1 << -0.1;
    constraints.emplace_back(A1, b1);
  }
  {
    Eigen::Matrix<double, 1, 3> A2;
    A2 << 0.0, 0.5, 0.1;
    Eigen::VectorXd b2(1);
    b2 << 2.0;
    constraints.emplace_back(A2, b2);
  }

  return constraints;
}

/// Two moving dynamic obstacles.
/// Mirrors makeSampleDynamicObstacles() in src/test/test_lbfgs_solver.cpp.
std::vector<std::shared_ptr<dynTraj>> makeSampleDynamicObstacles() {
  std::vector<std::shared_ptr<dynTraj>> obstacles;
  obstacles.reserve(2);

  {
    dynTraj d(Eigen::Vector3d{1.0, 3.0, 1.0}, Eigen::Vector3d{-0.3, 0.3, 0.0},
              Eigen::Vector3d{0.0, 0.0, 0.0}, Eigen::Vector3d{5.0, 5.0, 1.0},
              Eigen::Vector3d{0.0, 0.0, 0.0}, Eigen::Vector3d{0.0, 0.0, 0.0},
              0.0, 8.0);
    d.id = 10;
    d.is_agent = false;
    obstacles.push_back(std::make_shared<dynTraj>(std::move(d)));
  }
  {
    dynTraj d(Eigen::Vector3d{0.0, 0.5, 0.0}, Eigen::Vector3d{0.5, 0.0, 0.0},
              Eigen::Vector3d{0.0, 0.0, 0.0}, Eigen::Vector3d{6.0, 5.0, -0.5},
              Eigen::Vector3d{0.0, 0.0, 0.0}, Eigen::Vector3d{0.0, 0.0, 0.0},
              0.0, 8.0);
    d.id = 11;
    d.is_agent = false;
    obstacles.push_back(std::make_shared<dynTraj>(std::move(d)));
  }

  return obstacles;
}

/// Build a fully configured solver on the baseline problem and return it
/// alongside the first valid initial-guess vector for that problem.
std::shared_ptr<lbfgs::SolverLBFGS> buildBaselineSolver(Eigen::VectorXd& z0_out) {
  using namespace lbfgs;

  planner_params_t p;
  p.verbose = false;
  p.V_max = 2.0;
  p.A_max = 10.0;
  p.J_max = 100.0;
  p.num_perturbation = 8;
  p.r_max = 1.0;
  // Exercise every cost term so the gradient check covers the whole evaluator.
  p.dyn_weight = 1.0;
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

  auto solver = std::make_shared<SolverLBFGS>();
  solver->initializeSolver(p);

  vec_Vecf<3> global_wps = {Vecf<3>{0.0, 0.0, 0.0}, Vecf<3>{0.0, 2.0, 0.0},
                            Vecf<3>{0.0, 4.0, 0.0}, Vecf<3>{5.0, 5.0, 0.0}};
  std::vector<LinearConstraint3D> safe_corridor = makeSampleStaticConstraints();
  std::vector<std::shared_ptr<dynTraj>> obstacles = makeSampleDynamicObstacles();

  state s0, sf;
  s0.setPos(0.0, 0.0, 0.0);
  s0.setVel(0.0, 0.0, 0.0);
  s0.setAccel(0.0, 0.0, 0.0);
  sf.setPos(5.0, 5.0, 0.0);
  sf.setVel(0.0, 0.0, 0.0);
  sf.setAccel(0.0, 0.0, 0.0);

  double dummy_t = 0.0;
  solver->prepareSolverForReplan(0.0, global_wps, safe_corridor, obstacles, s0, sf,
                                 dummy_t, false);

  // Use the first initial guess produced by prepareSolverForReplan; this is
  // guaranteed to have the correct K_ layout.
  const auto& guesses = solver->getInitialGuesses();
  if (guesses.empty()) {
    z0_out = Eigen::VectorXd();
  } else {
    z0_out = guesses.front();
  }
  return solver;
}

}  // namespace

// Random-direction directional derivative test.
TEST(GradientCheck, DirectionalMatchesFiniteDiff) {
  Eigen::VectorXd z0;
  auto solver = buildBaselineSolver(z0);
  ASSERT_GT(z0.size(), 0) << "prepareSolverForReplan produced no initial guesses";

  const double err = solver->checkGradDirectional(z0, /*num_dirs=*/16, /*eps=*/1e-6,
                                                  /*seed=*/42);
  EXPECT_LT(err, 1e-5) << "Worst directional rel err = " << err;
}

// Coordinate-by-coordinate gradient check.
TEST(GradientCheck, CoordinatesMatchFiniteDiff) {
  Eigen::VectorXd z0;
  auto solver = buildBaselineSolver(z0);
  ASSERT_GT(z0.size(), 0) << "prepareSolverForReplan produced no initial guesses";

  const double err = solver->checkGradCoordinates(z0, /*max_coords=*/static_cast<int>(z0.size()),
                                                  /*eps=*/1e-6, /*seed=*/7);
  EXPECT_LT(err, 1e-5) << "Worst per-coord rel err = " << err;
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
