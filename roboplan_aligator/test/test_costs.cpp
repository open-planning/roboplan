#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <aligator/modelling/costs/sum-of-costs.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>

#include <roboplan/core/scene.hpp>

#include <roboplan_aligator/costs.hpp>
#include <roboplan_aligator/trajectory_optimizer.hpp>
#include <roboplan_aligator/types.hpp>

#include "frame_axis_residual.hpp"
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
using testing::kTipFrame;
using testing::makeSo101Scene;

// Frame pose (4x4) of `fid` at reduced configuration `q`, for reach checks.
Eigen::Matrix4d framePose(const pinocchio::Model& model, const Eigen::VectorXd& q,
                          pinocchio::FrameIndex fid) {
  pinocchio::Data data(model);
  pinocchio::forwardKinematics(model, data, q);
  pinocchio::updateFramePlacement(model, data, fid);
  return data.oMf[fid].toHomogeneousMatrix();
}

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

TEST(CostTest, FramePoseCostGradientMatchesFD) {
  CostFixture f;
  FramePoseCost spec;
  spec.frame = kTipFrame;
  spec.target = Eigen::Matrix4d::Identity();
  spec.target(0, 3) = 0.1;  // offset so the residual is nonzero
  spec.position_cost = Eigen::Vector3d(2.0, 1.0, 0.5);
  spec.orientation_cost = Eigen::Vector3d(0.7, 1.3, 0.9);

  CostStack stack(ManifoldPoly(f.space), f.nv());
  aligator_detail::attachFramePoseCost(stack, f.space, f.rgm, spec, 1.5);
  auto* cost = stack.getComponent<CostAbstract>(std::size_t{0});
  ASSERT_NE(cost, nullptr);
  const Eigen::VectorXd x = deterministicState(f.space);
  expectCostGradientMatchesFD(*cost, f.space, x, Eigen::VectorXd::Constant(f.nv(), 0.2));
}

TEST(CostTest, FrameAxisCostGradientMatchesFD) {
  CostFixture f;
  FrameAxisCost spec;
  spec.frame = kTipFrame;
  spec.axis_local = Eigen::Vector3d::UnitZ();
  spec.axis_world_target = Eigen::Vector3d::UnitX();
  spec.weight = 1.7;

  CostStack stack(ManifoldPoly(f.space), f.nv());
  aligator_detail::attachFrameAxisCost(stack, f.space, f.rgm, spec, 1.0);
  auto* cost = stack.getComponent<CostAbstract>(std::size_t{0});
  ASSERT_NE(cost, nullptr);
  const Eigen::VectorXd x = deterministicState(f.space);
  expectCostGradientMatchesFD(*cost, f.space, x, Eigen::VectorXd::Constant(f.nv(), 0.2));
}

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

TEST(CostTest, ControlCostGradientMatchesFD) {
  CostFixture f;
  ControlCost spec;
  spec.weights = Eigen::VectorXd::LinSpaced(f.nv(), 1.0, 3.0);
  spec.u_target = Eigen::VectorXd::Constant(f.nv(), 0.1);

  CostStack stack(ManifoldPoly(f.space), f.nv());
  aligator_detail::attachControlCost(stack, f.space, f.rgm, spec, 1.0);
  auto* cost = stack.getComponent<CostAbstract>(std::size_t{0});
  const Eigen::VectorXd x = deterministicState(f.space);
  expectCostGradientMatchesFD(*cost, f.space, x, Eigen::VectorXd::Constant(f.nv(), 0.25));
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

// The custom FrameAxis residual carries an analytic Jacobian; check it directly against FD.
TEST(CostTest, FrameAxisResidualJacobianMatchesFD) {
  CostFixture f;
  const auto fid = f.rgm.frameId(kTipFrame);
  ASSERT_TRUE(fid.has_value()) << fid.error();

  aligator_detail::FrameAxisResidual residual(f.space.ndx(), f.nv(), f.rgm.reducedModel(), *fid,
                                              Eigen::Vector3d::UnitZ(), Eigen::Vector3d::UnitX());
  auto data = residual.createData();
  const Eigen::VectorXd x = deterministicState(f.space);
  residual.evaluate(x, *data);
  residual.computeJacobians(x, *data);
  const Eigen::MatrixXd jx = data->Jx_;  // 3 x ndx

  const double eps = 1e-6;
  const int ndx = f.space.ndx();
  Eigen::MatrixXd jx_fd(3, ndx);
  const auto value_at = [&](const Eigen::VectorXd& xx) {
    auto d = residual.createData();
    residual.evaluate(xx, *d);
    return Eigen::Vector3d(d->value_);
  };
  for (int i = 0; i < ndx; ++i) {
    Eigen::VectorXd dv = Eigen::VectorXd::Zero(ndx);
    dv(i) = eps;
    const Eigen::Vector3d rp = value_at(f.space.integrate(x, dv));
    dv(i) = -eps;
    const Eigen::Vector3d rm = value_at(f.space.integrate(x, dv));
    jx_fd.col(i) = (rp - rm) / (2.0 * eps);
  }
  // 1e-5: central-difference accuracy on a smooth (FK) residual.
  EXPECT_LT((jx - jx_fd).cwiseAbs().maxCoeff(), 1e-5);
}

// --- Mutable target (value-polymorphism caveat) ------------------------------------------------

// setTarget must mutate the residual living INSIDE the problem: solve to pose A, retarget to B,
// re-solve, and the second result must approach B.
TEST(CostTest, SetTargetMutatesInProblemResidual) {
  auto scene = makeSo101Scene();
  const ReducedGroupModel rgm(*scene, "arm");
  const auto& model = rgm.reducedModel();
  const int nq = rgm.nq();
  const auto fid = rgm.frameId(kTipFrame).value();

  const Eigen::Matrix4d target_a = framePose(model, Eigen::VectorXd::Constant(nq, 0.25), fid);
  const Eigen::Matrix4d target_b = framePose(model, Eigen::VectorXd::Constant(nq, -0.25), fid);

  TrajOptOptions options;
  options.max_iters = 200;
  TrajectoryOptimizer opt(scene, "arm", /*horizon=*/40, /*dt=*/0.05, options);

  FramePoseCost pose;
  pose.frame = kTipFrame;
  pose.target = target_a;
  pose.position_cost = Eigen::Vector3d::Constant(200.0);
  pose.orientation_cost = Eigen::Vector3d::Constant(200.0);
  CostHandle handle = opt.addCost(pose, StageWindow::terminal(), 1.0);

  opt.build();
  const auto res_a = opt.solve(TrajOptSeed{});
  ASSERT_TRUE(res_a.has_value()) << res_a.error();
  const Eigen::Matrix4d reached_a = framePose(model, res_a->xs.back().head(nq), fid);

  handle.setTarget(target_b);  // hot-path retarget of the in-problem residual
  const auto res_b = opt.solve(TrajOptSeed{});
  ASSERT_TRUE(res_b.has_value()) << res_b.error();
  const Eigen::Matrix4d reached_b = framePose(model, res_b->xs.back().head(nq), fid);

  const auto pos = [](const Eigen::Matrix4d& m) { return Eigen::Vector3d(m.block<3, 1>(0, 3)); };
  // After retargeting to B, the terminal frame is nearer B than the A-solve was.
  const double err_b_after = (pos(reached_b) - pos(target_b)).norm();
  const double err_b_before = (pos(reached_a) - pos(target_b)).norm();
  EXPECT_LT(err_b_after, err_b_before);
  EXPECT_LT(err_b_after, 0.05) << "terminal frame did not track the new target B";
}

// Same build->setTarget->solve contract for a VECTOR-target cost (ConfigurationCost), which reaches
// its in-problem residual through QuadraticStateCost::setTarget rather than the FramePose path.
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

// --- UC1 core: a terminal FramePoseCost reach converges to the target pose --------------------

TEST(CostTest, TerminalFramePoseReachConverges) {
  auto scene = makeSo101Scene();
  const ReducedGroupModel rgm(*scene, "arm");
  const auto& model = rgm.reducedModel();
  const int nq = rgm.nq();
  const auto fid = rgm.frameId(kTipFrame).value();

  // A reachable target: the tip pose at a modest configuration.
  const Eigen::Matrix4d target = framePose(model, Eigen::VectorXd::Constant(nq, 0.3), fid);

  TrajOptOptions options;
  options.max_iters = 300;
  options.control_reg = 1e-3;
  TrajectoryOptimizer opt(scene, "arm", /*horizon=*/60, /*dt=*/0.05, options);

  FramePoseCost pose;
  pose.frame = kTipFrame;
  pose.target = target;
  pose.position_cost = Eigen::Vector3d::Constant(500.0);
  pose.orientation_cost = Eigen::Vector3d::Constant(500.0);
  opt.addCost(pose, StageWindow::terminal(), 1.0);

  opt.build();
  const auto result = opt.solve(TrajOptSeed{});
  ASSERT_TRUE(result.has_value()) << result.error();

  // Convergence means the terminal FRAME reaches the pose — not the solver's tolerance flag: a pure
  // terminal-cost reach with only control regularization need not push the AL residual below `tol`.
  const Eigen::Matrix4d reached = framePose(model, result->xs.back().head(nq), fid);
  const double pos_err = (reached.block<3, 1>(0, 3) - target.block<3, 1>(0, 3)).norm();
  // Rotation error via the trace of R_reached^T R_target: err = arccos((trace - 1) / 2).
  const Eigen::Matrix3d r_rel = reached.block<3, 3>(0, 0).transpose() * target.block<3, 3>(0, 0);
  const double rot_err = std::acos(std::clamp((r_rel.trace() - 1.0) / 2.0, -1.0, 1.0));
  // 3 cm / 0.05 rad: a terminal-only reach (no terminal velocity cost) converges to a neighborhood.
  EXPECT_LT(pos_err, 0.03) << "terminal position error " << pos_err << " m";
  EXPECT_LT(rot_err, 0.05) << "terminal orientation error " << rot_err << " rad";
}

// --- Lifecycle + handle guards ----------------------------------------------------------------

TEST(CostTest, AddCostAfterBuildThrowsThenResetAllows) {
  auto scene = makeSo101Scene();
  TrajectoryOptimizer opt(scene, "arm", /*horizon=*/8, /*dt=*/0.02);

  ControlCost cost;
  cost.weights = Eigen::VectorXd::Ones(opt.nv());
  EXPECT_NO_THROW(opt.addCost(cost));

  opt.build();
  EXPECT_THROW(opt.addCost(cost), std::logic_error);  // locked after build()
  EXPECT_TRUE(opt.solve(TrajOptSeed{}).has_value());  // solving does not un/re-lock

  opt.resetProblem();
  EXPECT_NO_THROW(opt.addCost(cost));  // legal again (a fresh build() is required to solve)
}

TEST(CostTest, CostHandleRejectsWrongTargetKind) {
  auto scene = makeSo101Scene();
  TrajectoryOptimizer opt(scene, "arm", /*horizon=*/8, /*dt=*/0.02);

  // Typed locals: the setTarget overloads (Matrix4d vs VectorXd) are unambiguous only for concrete
  // types, not raw Eigen expressions.
  const Eigen::Matrix4d identity_pose = Eigen::Matrix4d::Identity();
  const Eigen::VectorXd control_target = Eigen::VectorXd::Zero(opt.nv());
  const Eigen::VectorXd wrong_size = Eigen::VectorXd::Zero(opt.nv() + 1);
  const Eigen::VectorXd axis_target = Eigen::VectorXd::Zero(3);

  ControlCost control;
  control.weights = Eigen::VectorXd::Ones(opt.nv());
  CostHandle vector_handle = opt.addCost(control);
  EXPECT_THROW(vector_handle.setTarget(identity_pose), std::logic_error);
  EXPECT_THROW(vector_handle.setTarget(wrong_size), std::invalid_argument);
  EXPECT_NO_THROW(vector_handle.setTarget(control_target));

  FramePoseCost pose;
  pose.frame = kTipFrame;
  CostHandle pose_handle = opt.addCost(pose, StageWindow::terminal());
  EXPECT_THROW(pose_handle.setTarget(axis_target), std::logic_error);
  EXPECT_NO_THROW(pose_handle.setTarget(identity_pose));
}

}  // namespace roboplan
