#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <aligator/modelling/constraints/box-constraint.hpp>

#include <roboplan/core/scene.hpp>

#include <roboplan_aligator/constraints.hpp>
#include <roboplan_aligator/costs.hpp>
#include <roboplan_aligator/trajectory_optimizer.hpp>
#include <roboplan_aligator/types.hpp>

#include "test_fd_util.hpp"
#include "test_util.hpp"
#include <roboplan_aligator/constraint_factory.hpp>
#include <roboplan_aligator/problem_builder.hpp>
#include <roboplan_aligator/reduced_group_model.hpp>

namespace roboplan {
namespace {

using aligator_detail::ConstraintPair;
using aligator_detail::PhaseSpace;
using BoxConstraint = aligator::BoxConstraintTpl<double>;
using testing::deterministicState;
using testing::kTipFrame;
using testing::makeSo101Scene;

// The BoxConstraintTpl behind a pair's constraint set (for bound inspection).
const BoxConstraint& asBox(const ConstraintPair& pair) {
  const auto* box = dynamic_cast<const BoxConstraint*>(&*pair.set);
  EXPECT_NE(box, nullptr) << "constraint set is not a BoxConstraint";
  return *box;
}

// Evaluate a pair's residual value at (x, u).
Eigen::VectorXd residualValue(const ConstraintPair& pair, const Eigen::VectorXd& x,
                              const Eigen::VectorXd& u) {
  auto data = pair.func->createData();
  pair.func->evaluate(x, u, *data);
  return data->value_;
}

// Fixture holding the reduced-model machinery shared by the constraint tests.
struct ConstraintFixture {
  std::shared_ptr<Scene> scene = makeSo101Scene();
  ReducedGroupModel rgm{*scene, "arm"};
  PhaseSpace space = aligator_detail::makePhaseSpace(rgm.reducedModel());

  [[nodiscard]] int nq() const { return rgm.nq(); }
  [[nodiscard]] int nv() const { return rgm.nv(); }
};

// Largest |torque| across a control trajectory.
double peakTorque(const std::vector<Eigen::VectorXd>& us) {
  double peak = 0.0;
  for (const auto& u : us) {
    peak = std::max(peak, u.cwiseAbs().maxCoeff());
  }
  return peak;
}

}  // namespace

// --- Residual + box correctness (each constraint maps to the value we think) ------------------

TEST(ConstraintTest, TorqueLimitResidualIsControlAndBoxIsModelEffort) {
  ConstraintFixture f;
  const auto pair = aligator_detail::buildTorqueLimit(f.space, f.rgm, TorqueLimit{});

  // Value = u exactly (Euclidean control residual).
  const Eigen::VectorXd u = Eigen::VectorXd::LinSpaced(f.nv(), -0.7, 1.3);
  EXPECT_LT((residualValue(pair, deterministicState(f.space), u) - u).cwiseAbs().maxCoeff(), 1e-12);

  // Default box comes from the model's (finite, actuated) effort limits: upper > 0, lower < 0, and
  // FINITE — a finite default must not be confused with the ±inf a missing/zero effort would yield.
  const BoxConstraint& box = asBox(pair);
  ASSERT_EQ(box.upper_limit.size(), f.nv());
  EXPECT_TRUE((box.upper_limit.array() > 0.0).all()) << box.upper_limit.transpose();
  EXPECT_TRUE((box.lower_limit.array() < 0.0).all()) << box.lower_limit.transpose();
  EXPECT_TRUE(box.upper_limit.allFinite()) << box.upper_limit.transpose();
  EXPECT_TRUE(box.lower_limit.allFinite()) << box.lower_limit.transpose();
}

// --- Decision gate: a user bound clamps to (intersects) the model's per-DoF -------------------

TEST(ConstraintTest, UserBoundClampsToModel) {
  ConstraintFixture f;
  const int nv = f.nv();

  // The default box comes from the model's effort limits.
  const auto default_tau = aligator_detail::buildTorqueLimit(f.space, f.rgm, TorqueLimit{});
  const Eigen::VectorXd model_tau_upper = asBox(default_tau).upper_limit;

  // A user tau_max is symmetric [-tau_max, +tau_max], intersected with the model's per-DoF upper.
  TorqueLimit tight;
  tight.tau_max = Eigen::VectorXd::Constant(nv, 0.5);
  const auto clamped_tau = aligator_detail::buildTorqueLimit(f.space, f.rgm, tight);
  EXPECT_LT((asBox(clamped_tau).upper_limit - model_tau_upper.cwiseMin(0.5)).cwiseAbs().maxCoeff(),
            1e-12);
  EXPECT_LT(
      (asBox(clamped_tau).lower_limit - (-model_tau_upper.cwiseMin(0.5))).cwiseAbs().maxCoeff(),
      1e-12);
}

// --- Windowing: a constraint attaches only to in-range stages ---------------------------------

TEST(ConstraintTest, WindowAttachesToInRangeStagesOnly) {
  ConstraintFixture f;
  const int horizon = 6;
  Eigen::VectorXd x0(f.nq() + f.nv());
  x0 << f.rgm.q0(), f.rgm.v0();
  auto problem =
      aligator_detail::buildProblemShell(f.space, x0, horizon, /*dt=*/0.02, TrajOptOptions{});

  // The shell has no stage constraints yet (only cost + dynamics + the problem-level init cond).
  for (int k = 0; k < horizon; ++k) {
    ASSERT_EQ(problem->stages_[static_cast<std::size_t>(k)]->numConstraints(), 0u);
  }

  const auto pair = aligator_detail::buildTorqueLimit(f.space, f.rgm, TorqueLimit{});
  const StageWindow window = StageWindow::range(1, 4);  // stages 1, 2, 3 (half-open)
  for (const int k : window.resolveStages(horizon)) {
    problem->stages_[static_cast<std::size_t>(k)]->addConstraint(pair.func, pair.set);
  }
  for (int k = 0; k < horizon; ++k) {
    const std::size_t expected = (k >= 1 && k < 4) ? 1u : 0u;
    EXPECT_EQ(problem->stages_[static_cast<std::size_t>(k)]->numConstraints(), expected)
        << "stage " << k;
  }
}

// --- A constrained solve respects the torque bounds within max_constraint_violation -----------

TEST(ConstraintTest, TorqueConstrainedSolveRespectsBound) {
  auto scene = makeSo101Scene();

  // A fixed reach target (world<-tip): a translation the arm must work toward, so the endpoint
  // requires nonzero torque. The horizon (200 stages @ 50 ms) is long enough that the reach stays
  // feasible under a reduced torque budget, so the AL solve converges rather than stalling against
  // an infeasible box.
  Eigen::Matrix4d target = Eigen::Matrix4d::Identity();
  target(0, 3) = 0.15;
  target(2, 3) = 0.20;

  const auto make_reach_opt = [&]() {
    TrajOptOptions options;
    options.max_iters = 500;
    options.control_reg = 1e-4;
    TrajectoryOptimizer opt(scene, "arm", /*horizon=*/200, /*dt=*/0.05, options);
    FramePoseCost pose;
    pose.frame = kTipFrame;
    pose.target = target;
    pose.position_cost = Eigen::Vector3d::Constant(200.0);
    pose.orientation_cost = Eigen::Vector3d::Constant(200.0);
    opt.addCost(pose, StageWindow::terminal(), 1.0);
    return opt;
  };

  // The unconstrained reach's peak torque is ~0.10 Nm; a 0.08 Nm cap sits strictly below it (so the
  // bound is genuinely active) yet high enough that the reach stays feasible (AL violation ~6e-5).
  const double tau = 0.08;

  auto opt_unc = make_reach_opt();
  opt_unc.build();
  const auto res_unc = opt_unc.solve(TrajOptSeed{});
  ASSERT_TRUE(res_unc.has_value()) << res_unc.error();
  const double peak_unc = peakTorque(res_unc->us);
  EXPECT_GT(peak_unc, tau) << "unconstrained peak " << peak_unc
                           << " is below the bound, so the constraint would be vacuous";

  auto opt_c = make_reach_opt();
  TorqueLimit limit;
  limit.tau_max = Eigen::VectorXd::Constant(opt_c.nv(), tau);
  opt_c.addConstraint(limit);  // all stages
  opt_c.build();
  const auto res_c = opt_c.solve(TrajOptSeed{});
  ASSERT_TRUE(res_c.has_value()) << res_c.error();

  const double peak_c = peakTorque(res_c->us);
  const double viol = res_c->max_constraint_violation;
  // Respect the box up to the reported violation (1e-6 = Eigen round-off, not a slackened bound).
  EXPECT_LE(peak_c, tau + viol + 1e-6) << "peak " << peak_c << " exceeds bound " << tau;
  // AL residual below 1e-3 Nm: a clearly-feasible box (the solve reaches ~7e-5), not tautological.
  EXPECT_LT(viol, 1e-3) << "max_constraint_violation " << viol;
  // Bound is genuinely active: the optimizer presses the peak torque up to the cap (0.9*tau
  // margin).
  EXPECT_GE(peak_c, 0.9 * tau) << "peak torque " << peak_c << " never approached the bound " << tau;

  // Same-seed determinism (testing rule): a fresh, identically-built solve reproduces the controls.
  auto opt_c2 = make_reach_opt();
  TorqueLimit limit2;
  limit2.tau_max = Eigen::VectorXd::Constant(opt_c2.nv(), tau);
  opt_c2.addConstraint(limit2);
  opt_c2.build();
  const auto res_c2 = opt_c2.solve(TrajOptSeed{});
  ASSERT_TRUE(res_c2.has_value()) << res_c2.error();
  ASSERT_EQ(res_c->us.size(), res_c2->us.size());
  double max_diff = 0.0;
  for (std::size_t k = 0; k < res_c->us.size(); ++k) {
    max_diff = std::max(max_diff, (res_c->us[k] - res_c2->us[k]).cwiseAbs().maxCoeff());
  }
  // 1e-9: determinism up to benign FP reassociation across two fresh solver instances.
  EXPECT_LT(max_diff, 1e-9) << "same-seed solves diverged by " << max_diff;
}

// --- Guards -----------------------------------------------------------------------------------

TEST(ConstraintTest, TorqueLimitRejectsTerminalWindow) {
  auto scene = makeSo101Scene();
  TrajectoryOptimizer opt(scene, "arm", /*horizon=*/8, /*dt=*/0.02);
  TorqueLimit limit;
  // The terminal node has no control, so a control-box constraint there is ill-defined.
  EXPECT_THROW(opt.addConstraint(limit, StageWindow::terminal()), std::invalid_argument);
}

TEST(ConstraintTest, WrongSizeBoundThrows) {
  auto scene = makeSo101Scene();
  TrajectoryOptimizer opt(scene, "arm", /*horizon=*/8, /*dt=*/0.02);
  TorqueLimit limit;
  limit.tau_max = Eigen::VectorXd::Zero(opt.nv() + 1);  // wrong size
  EXPECT_THROW(opt.addConstraint(limit), std::invalid_argument);
}

TEST(ConstraintTest, AddConstraintAfterBuildThrowsThenResetAllows) {
  auto scene = makeSo101Scene();
  TrajectoryOptimizer opt(scene, "arm", /*horizon=*/8, /*dt=*/0.02);

  TorqueLimit limit;
  EXPECT_NO_THROW(opt.addConstraint(limit));
  opt.build();
  EXPECT_THROW(opt.addConstraint(limit), std::logic_error);  // locked after build()
  ASSERT_TRUE(opt.solve(TrajOptSeed{}).has_value());

  opt.resetProblem();
  EXPECT_NO_THROW(opt.addConstraint(limit));  // legal again
}

}  // namespace roboplan
