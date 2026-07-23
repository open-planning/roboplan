#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <roboplan/core/scene.hpp>
#include <roboplan_example_models/resources.hpp>

#include <roboplan_aligator/trajectory_optimizer.hpp>
#include <roboplan_aligator/types.hpp>

// This target exercises ONLY the public TrajectoryOptimizer surface, so it does not link aligator
// (the driver's whole point is that aligator stays behind the PIMPL). The finite-difference test
// of the underlying dynamics lives in test_dynamics.cpp.

namespace roboplan {
namespace {

std::shared_ptr<Scene> makeSo101Scene() {
  const auto model_prefix = example_models::get_package_models_dir();
  const std::vector<std::filesystem::path> package_paths = {
      example_models::get_package_share_dir()};
  return std::make_shared<Scene>("test_scene", model_prefix / "so101_robot_model" / "so101.urdf",
                                 model_prefix / "so101_robot_model" / "so101.srdf", package_paths);
}

}  // namespace

// The problem shell carries only the default control-regularization cost, so the well-posed
// optimum from rest is to drive the controls toward zero — the solver must converge and return a
// fully-populated, dimensionally-consistent result.
TEST(TrajectoryOptimizerTest, SolvesControlRegShellAndReturnsPopulatedResult) {
  auto scene = makeSo101Scene();
  TrajOptOptions options;
  options.max_iters = 100;
  options.control_reg = 1e-2;

  TrajectoryOptimizer opt(scene, "arm", /*horizon=*/10, /*dt=*/0.02, options);

  const int num_stages = opt.horizon();
  const int nx = opt.nx();
  const int nq = opt.nq();
  const int nv = opt.nv();
  EXPECT_EQ(nx, nq + nv);

  // Empty seed: aligator default-initializes the warm start from the problem's initial state.
  const auto result = opt.solve(TrajOptSeed{});
  ASSERT_TRUE(result.has_value()) << result.error();

  EXPECT_TRUE(result->converged);
  EXPECT_GE(result->iterations, 0);
  EXPECT_GE(result->max_constraint_violation, 0.0);

  // Raw solver arrays.
  ASSERT_EQ(result->xs.size(), static_cast<std::size_t>(num_stages + 1));
  ASSERT_EQ(result->us.size(), static_cast<std::size_t>(num_stages));
  EXPECT_EQ(result->xs.front().size(), nx);
  EXPECT_EQ(result->us.front().size(), nv);

  // controls == us for actuation B = I.
  ASSERT_EQ(result->controls.size(), result->us.size());
  EXPECT_TRUE(result->controls.front().isApprox(result->us.front()));

  // Semantic trajectory views.
  ASSERT_EQ(result->trajectory.times.size(), static_cast<std::size_t>(num_stages + 1));
  ASSERT_EQ(result->trajectory.positions.size(), static_cast<std::size_t>(num_stages + 1));
  ASSERT_EQ(result->trajectory.velocities.size(), static_cast<std::size_t>(num_stages + 1));
  EXPECT_EQ(result->trajectory.positions.front().size(), nq);
  EXPECT_EQ(result->trajectory.velocities.front().size(), nv);
  EXPECT_DOUBLE_EQ(result->trajectory.times[0], 0.0);
  EXPECT_DOUBLE_EQ(result->trajectory.times[1], 0.02);

  // The minimum-effort optimum from rest is (near) zero torque.
  for (const auto& u : result->controls) {
    EXPECT_LT(u.cwiseAbs().maxCoeff(), 1e-2);
  }

  // Same-seed determinism (testing rule): a fresh optimizer solving the identical problem and seed
  // reproduces the trajectory. A second instance (rather than re-solving on the same one) isolates
  // input->output determinism from the solver's carried-over internal state. Compared to 1e-9,
  // which is ~5 orders below the 1e-4 solver tolerance yet above the ~1e-16 floating-point /
  // OpenMP reduction-order noise that makes bit-exact reproduction unattainable.
  const double determinism_tol = 1e-9;
  TrajectoryOptimizer opt2(makeSo101Scene(), "arm", /*horizon=*/10, /*dt=*/0.02, options);
  const auto result2 = opt2.solve(TrajOptSeed{});
  ASSERT_TRUE(result2.has_value()) << result2.error();
  ASSERT_EQ(result2->xs.size(), result->xs.size());
  ASSERT_EQ(result2->us.size(), result->us.size());
  for (std::size_t k = 0; k < result->xs.size(); ++k) {
    EXPECT_LT((result2->xs[k] - result->xs[k]).cwiseAbs().maxCoeff(), determinism_tol);
  }
  for (std::size_t k = 0; k < result->us.size(); ++k) {
    EXPECT_LT((result2->us[k] - result->us[k]).cwiseAbs().maxCoeff(), determinism_tol);
  }
}

TEST(TrajectoryOptimizerTest, SolveRejectsWrongSeedSize) {
  auto scene = makeSo101Scene();
  TrajectoryOptimizer opt(scene, "arm", /*horizon=*/8, /*dt=*/0.02);

  // us with the wrong number of entries (horizon + 3 instead of horizon).
  TrajOptSeed seed;
  seed.us.assign(static_cast<std::size_t>(opt.horizon() + 3), Eigen::VectorXd::Zero(opt.nv()));
  const auto result = opt.solve(seed);
  EXPECT_FALSE(result.has_value());
}

TEST(TrajectoryOptimizerTest, SetInitialStateValidatesSizes) {
  auto scene = makeSo101Scene();
  TrajectoryOptimizer opt(scene, "arm", /*horizon=*/8, /*dt=*/0.02);

  // Correct nq, default (empty) velocity is accepted.
  EXPECT_NO_THROW(opt.setInitialState(Eigen::VectorXd::Zero(opt.nq())));
  // Wrong nq throws.
  EXPECT_THROW(opt.setInitialState(Eigen::VectorXd::Zero(opt.nq() + 1)), std::invalid_argument);
  // Wrong nv throws.
  EXPECT_THROW(
      opt.setInitialState(Eigen::VectorXd::Zero(opt.nq()), Eigen::VectorXd::Zero(opt.nv() + 1)),
      std::invalid_argument);
}

TEST(TrajectoryOptimizerTest, ConstructorRejectsInvalidGrid) {
  auto scene = makeSo101Scene();
  EXPECT_THROW(TrajectoryOptimizer(scene, "arm", /*horizon=*/0, /*dt=*/0.02),
               std::invalid_argument);
  EXPECT_THROW(TrajectoryOptimizer(scene, "arm", /*horizon=*/5, /*dt=*/0.0), std::invalid_argument);
}

}  // namespace roboplan
