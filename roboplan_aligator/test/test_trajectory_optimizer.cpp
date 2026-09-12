#include <gtest/gtest.h>

#include <cstddef>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <roboplan/core/scene.hpp>

#include <roboplan_aligator/trajectory_optimizer.hpp>
#include <roboplan_aligator/types.hpp>

#include "test_util.hpp"

// This target exercises ONLY the public TrajectoryOptimizer surface (no direct aligator or
// reduced-model calls), so it does not need the finite-difference/geometry machinery of the
// internal tests. The finite-difference dynamics test lives in test_dynamics.cpp.

namespace roboplan {
namespace {

using testing::makeSo101Scene;

}  // namespace

// The problem shell carries only the default control-regularization cost, so from rest the optimal
// control is (near) zero torque; the solver must converge and return a fully-populated,
// dimensionally-consistent result.
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
  opt.build();
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

  // Same-seed determinism (testing rule): a fresh, identically-built solve reproduces the result
  // via a second instance (isolating input->output determinism from carried-over solver state).
  // 1e-9 is ~5 orders below the 1e-4 solver tolerance yet above FP/OpenMP reduction-order noise.
  const double determinism_tol = 1e-9;
  TrajectoryOptimizer opt2(makeSo101Scene(), "arm", /*horizon=*/10, /*dt=*/0.02, options);
  opt2.build();
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
  opt.build();
  const auto result = opt.solve(seed);
  EXPECT_FALSE(result.has_value());
}

TEST(TrajectoryOptimizerTest, SetInitialStateValidatesSizes) {
  auto scene = makeSo101Scene();
  TrajectoryOptimizer opt(scene, "arm", /*horizon=*/8, /*dt=*/0.02);

  // Correct nq is accepted (the initial state is set to [q; 0]).
  EXPECT_NO_THROW(opt.setInitialState(Eigen::VectorXd::Zero(opt.nq())));
  // Wrong nq throws.
  EXPECT_THROW(opt.setInitialState(Eigen::VectorXd::Zero(opt.nq() + 1)), std::invalid_argument);
}

TEST(TrajectoryOptimizerTest, ConstructorRejectsInvalidGrid) {
  auto scene = makeSo101Scene();
  EXPECT_THROW(TrajectoryOptimizer(scene, "arm", /*horizon=*/0, /*dt=*/0.02),
               std::invalid_argument);
  EXPECT_THROW(TrajectoryOptimizer(scene, "arm", /*horizon=*/5, /*dt=*/0.0), std::invalid_argument);
}

// --- Lifecycle: build() gate ----------------------------------------------------------------

TEST(TrajectoryOptimizerTest, SolveBeforeBuildErrors) {
  auto scene = makeSo101Scene();
  TrajectoryOptimizer opt(scene, "arm", /*horizon=*/8, /*dt=*/0.02);
  // solve() does not auto-build: it returns a recoverable error.
  const auto result = opt.solve(TrajOptSeed{});
  ASSERT_FALSE(result.has_value());
  EXPECT_NE(result.error().find("build()"), std::string::npos);
}

TEST(TrajectoryOptimizerTest, BuildIsIdempotentAndResetRequiresRebuild) {
  auto scene = makeSo101Scene();
  TrajectoryOptimizer opt(scene, "arm", /*horizon=*/8, /*dt=*/0.02);
  opt.build();
  EXPECT_NO_THROW(opt.build());  // idempotent: a second build() while built is a no-op
  EXPECT_TRUE(opt.solve(TrajOptSeed{}).has_value());

  opt.resetProblem();
  // After reset the problem is unbuilt again; solve must error until a fresh build().
  EXPECT_FALSE(opt.solve(TrajOptSeed{}).has_value());
  opt.build();
  EXPECT_TRUE(opt.solve(TrajOptSeed{}).has_value());
}

// --- Warm-start seeding ---------------------------------------------------------------------

TEST(TrajectoryOptimizerTest, InterpolatePathBuildsGridSeed) {
  auto scene = makeSo101Scene();
  TrajectoryOptimizer opt(scene, "arm", /*horizon=*/10, /*dt=*/0.05);
  const int nq = opt.nq();
  const int nv = opt.nv();
  const auto num_stages = static_cast<std::size_t>(opt.horizon());

  const Eigen::VectorXd q0 = Eigen::VectorXd::Zero(nq);
  const Eigen::VectorXd q1 = Eigen::VectorXd::Constant(nq, 0.4);
  const TrajOptSeed seed = opt.interpolatePath({q0, q1});

  ASSERT_EQ(seed.xs.size(), num_stages + 1);
  ASSERT_EQ(seed.us.size(), num_stages);
  // Endpoints match the waypoints; velocities and controls are zero.
  EXPECT_TRUE(seed.xs.front().head(nq).isApprox(q0));
  EXPECT_TRUE(seed.xs.back().head(nq).isApprox(q1));
  EXPECT_LT(seed.xs.front().tail(nv).cwiseAbs().maxCoeff(), 1e-12);
  EXPECT_LT(seed.us.front().cwiseAbs().maxCoeff(), 1e-12);
  // Revolute/prismatic joints interpolate linearly, so the grid advances monotonically toward q1.
  for (std::size_t k = 1; k < seed.xs.size(); ++k) {
    EXPECT_GE(seed.xs[k].head(nq).sum() + 1e-12, seed.xs[k - 1].head(nq).sum());
  }

  // A single waypoint yields a constant seed.
  const TrajOptSeed constant_seed = opt.interpolatePath({q1});
  for (const auto& x : constant_seed.xs) {
    EXPECT_TRUE(x.head(nq).isApprox(q1));
  }

  EXPECT_THROW(opt.interpolatePath({}), std::invalid_argument);
  EXPECT_THROW(opt.interpolatePath({Eigen::VectorXd::Zero(nq + 1)}), std::invalid_argument);
}

}  // namespace roboplan
