#include <gtest/gtest.h>

#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <aligator/modelling/costs/sum-of-costs.hpp>

#include <roboplan/core/scene.hpp>

#include <roboplan_aligator/costs.hpp>
#include <roboplan_aligator/trajectory_optimizer.hpp>
#include <roboplan_aligator/types.hpp>

#include "test_fd_util.hpp"
#include "test_util.hpp"
#include <roboplan_aligator/cost_factory.hpp>
#include <roboplan_aligator/problem_builder.hpp>
#include <roboplan_aligator/reduced_group_model.hpp>

namespace roboplan {
namespace {

using aligator_detail::CostStack;
using aligator_detail::PhaseSpace;
using ManifoldPoly = xyz::polymorphic<aligator::ManifoldAbstractTpl<double>>;
using CostAbstract = aligator::CostAbstractTpl<double>;
using testing::deterministicState;
using testing::makeSo101Scene;

// Central-difference check: analytic cost gradient (Lx_, Lu_) vs finite differences of its value.
// eps=1e-6 gives O(eps^2) truncation and ~1e-10 roundoff, well under the 1e-5 tolerance.
void expectCostGradientMatchesFD(CostAbstract& cost, const PhaseSpace& space,
                                 const Eigen::VectorXd& x, const Eigen::VectorXd& u) {
  const int ndx = space.ndx();
  const int nu = static_cast<int>(u.size());
  const double eps = 1e-6;
  const double tol = 1e-5;

  auto data = cost.createData();
  cost.evaluate(x, u, *data);
  cost.computeGradients(x, u, *data);
  const Eigen::VectorXd lx = data->Lx_;
  const Eigen::VectorXd lu = data->Lu_;

  const auto value_at = [&](const Eigen::VectorXd& xx, const Eigen::VectorXd& uu) {
    auto d = cost.createData();
    cost.evaluate(xx, uu, *d);
    return d->value_;
  };

  for (int i = 0; i < ndx; ++i) {
    Eigen::VectorXd dv = Eigen::VectorXd::Zero(ndx);
    dv(i) = eps;
    const double fp = value_at(space.integrate(x, dv), u);
    dv(i) = -eps;
    const double fm = value_at(space.integrate(x, dv), u);
    EXPECT_NEAR((fp - fm) / (2.0 * eps), lx(i), tol) << "Lx[" << i << "]";
  }
  for (int j = 0; j < nu; ++j) {
    Eigen::VectorXd up = u;
    Eigen::VectorXd um = u;
    up(j) += eps;
    um(j) -= eps;
    EXPECT_NEAR((value_at(x, up) - value_at(x, um)) / (2.0 * eps), lu(j), tol) << "Lu[" << j << "]";
  }
}

// Fixture holding the reduced-model machinery shared by the cost tests.
struct CostFixture {
  std::shared_ptr<Scene> scene = makeSo101Scene();
  ReducedGroupModel rgm{*scene, "arm"};
  PhaseSpace space = aligator_detail::makePhaseSpace(rgm.reducedModel());

  [[nodiscard]] int nq() const { return rgm.nq(); }
  [[nodiscard]] int nv() const { return rgm.nv(); }
};

}  // namespace

// --- Finite-difference value+gradient of each cost --------------------------------------------

TEST(CostTest, ConfigurationCostGradientMatchesFD) {
  CostFixture f;
  ConfigurationCost spec;
  spec.q_target = Eigen::VectorXd::Constant(f.nq(), 0.15);
  spec.weights = Eigen::VectorXd::LinSpaced(f.nv(), 1.0, 2.0);

  CostStack stack(ManifoldPoly(f.space), f.nv());
  aligator_detail::attachConfigurationCost(stack, f.space, f.rgm, spec, 1.0);
  auto* cost = stack.getComponent<CostAbstract>(std::size_t{0});
  const Eigen::VectorXd x = deterministicState(f.space);
  expectCostGradientMatchesFD(*cost, f.space, x, Eigen::VectorXd::Constant(f.nv(), 0.2));
}

TEST(CostTest, VelocityCostGradientMatchesFD) {
  CostFixture f;
  VelocityCost spec;
  spec.weights = Eigen::VectorXd::LinSpaced(f.nv(), 1.0, 2.5);

  CostStack stack(ManifoldPoly(f.space), f.nv());
  aligator_detail::attachVelocityCost(stack, f.space, f.rgm, spec, 1.0);
  auto* cost = stack.getComponent<CostAbstract>(std::size_t{0});
  const Eigen::VectorXd x = deterministicState(f.space);
  expectCostGradientMatchesFD(*cost, f.space, x, Eigen::VectorXd::Constant(f.nv(), 0.2));
}

// --- Mutable target (value-polymorphism caveat) ------------------------------------------------

// setTarget must mutate the residual living INSIDE the problem: solve to q_a, retarget to q_b,
// re-solve, and the second result must approach q_b. Vector-target costs (ConfigurationCost) reach
// their in-problem residual through QuadraticStateCost::setTarget.
TEST(CostTest, SetTargetMutatesVectorCostResidual) {
  auto scene = makeSo101Scene();
  const int nq = ReducedGroupModel(*scene, "arm").nq();

  TrajOptOptions options;
  options.max_iters = 200;
  TrajectoryOptimizer opt(scene, "arm", /*horizon=*/40, /*dt=*/0.05, options);

  const Eigen::VectorXd q_a = Eigen::VectorXd::Constant(nq, 0.3);
  const Eigen::VectorXd q_b = Eigen::VectorXd::Constant(nq, -0.3);

  ConfigurationCost config;
  config.q_target = q_a;
  config.weights = Eigen::VectorXd::Constant(nq, 100.0);
  CostHandle handle = opt.addCost(config, StageWindow::terminal(), 1.0);

  opt.build();
  const auto res_a = opt.solve(TrajOptSeed{});
  ASSERT_TRUE(res_a.has_value()) << res_a.error();
  const Eigen::VectorXd q_reached_a = res_a->xs.back().head(nq);

  handle.setTarget(q_b);  // hot-path retarget of the in-problem state-error residual
  const auto res_b = opt.solve(TrajOptSeed{});
  ASSERT_TRUE(res_b.has_value()) << res_b.error();
  const Eigen::VectorXd q_reached_b = res_b->xs.back().head(nq);

  // After retargeting to q_b, the terminal configuration is nearer q_b than the q_a-solve was.
  EXPECT_LT((q_reached_b - q_b).norm(), (q_reached_a - q_b).norm());
  EXPECT_LT((q_reached_b - q_b).norm(), 0.1) << "terminal config did not track the new target";
}

// --- Lifecycle + handle guards ----------------------------------------------------------------

TEST(CostTest, AddCostAfterBuildThrowsThenResetAllows) {
  auto scene = makeSo101Scene();
  TrajectoryOptimizer opt(scene, "arm", /*horizon=*/8, /*dt=*/0.02);

  ConfigurationCost cost;
  cost.q_target = Eigen::VectorXd::Zero(opt.nq());
  cost.weights = Eigen::VectorXd::Ones(opt.nv());
  EXPECT_NO_THROW(opt.addCost(cost));

  opt.build();
  EXPECT_THROW(opt.addCost(cost), std::logic_error);  // locked after build()
  EXPECT_TRUE(opt.solve(TrajOptSeed{}).has_value());  // solving does not un/re-lock

  opt.resetProblem();
  EXPECT_NO_THROW(opt.addCost(cost));  // legal again (a fresh build() is required to solve)
}

TEST(CostTest, CostHandleRejectsWrongTargetSize) {
  auto scene = makeSo101Scene();
  TrajectoryOptimizer opt(scene, "arm", /*horizon=*/8, /*dt=*/0.02);

  const Eigen::VectorXd config_target = Eigen::VectorXd::Zero(opt.nv());
  const Eigen::VectorXd wrong_size = Eigen::VectorXd::Zero(opt.nv() + 1);

  ConfigurationCost config;
  config.q_target = Eigen::VectorXd::Zero(opt.nq());
  config.weights = Eigen::VectorXd::Ones(opt.nv());
  CostHandle vector_handle = opt.addCost(config);
  EXPECT_THROW(vector_handle.setTarget(wrong_size), std::invalid_argument);
  EXPECT_NO_THROW(vector_handle.setTarget(config_target));
}

}  // namespace roboplan
