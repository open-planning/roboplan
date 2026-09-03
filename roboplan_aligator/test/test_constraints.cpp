#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include <Eigen/Dense>

#include <aligator/modelling/constraints/box-constraint.hpp>
#include <pinocchio/algorithm/geometry.hpp>    // updateGeometryPlacements
#include <pinocchio/algorithm/kinematics.hpp>  // forwardKinematics
#include <pinocchio/collision/distance.hpp>
#include <pinocchio/multibody/geometry.hpp>

#include <roboplan/core/scene.hpp>

#include <roboplan_aligator/constraints.hpp>
#include <roboplan_aligator/costs.hpp>
#include <roboplan_aligator/trajectory_optimizer.hpp>
#include <roboplan_aligator/types.hpp>

#include "collision_residual.hpp"
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

// Reusable single-pair signed-distance evaluator on a reduced model (avoids building scratch per
// call — coal mesh distance is expensive). One instance per test.
struct PairDistance {
  const pinocchio::Model& model;
  const pinocchio::GeometryModel& geom;
  pinocchio::Data data{model};
  pinocchio::GeometryData geom_data{geom};

  PairDistance(const pinocchio::Model& m, const pinocchio::GeometryModel& g) : model(m), geom(g) {}

  double at(const Eigen::VectorXd& q, std::size_t pair_id) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateGeometryPlacements(model, data, geom, geom_data);
    return pinocchio::computeDistance(geom, geom_data, pair_id).min_distance;
  }
};

// Index of the collision pair between the two named geometries, or SIZE_MAX if absent.
std::size_t findPair(const pinocchio::GeometryModel& geom, const std::string& a,
                     const std::string& b) {
  for (std::size_t k = 0; k < geom.collisionPairs.size(); ++k) {
    const auto& first = geom.geometryObjects[geom.collisionPairs[k].first].name;
    const auto& second = geom.geometryObjects[geom.collisionPairs[k].second].name;
    if ((first == a && second == b) || (first == b && second == a)) {
      return k;
    }
  }
  return std::numeric_limits<std::size_t>::max();
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

TEST(ConstraintTest, PositionLimitResidualSlicesConfigTangent) {
  ConstraintFixture f;
  const auto pair = aligator_detail::buildPositionLimit(f.space, f.rgm, *f.scene, PositionLimit{});
  const Eigen::VectorXd x = deterministicState(f.space);

  // Residual = the q-tangent rows {0..nv-1} of space.difference(neutral, x).
  const Eigen::VectorXd expected = f.space.difference(f.space.neutral(), x).head(f.nv());
  const Eigen::VectorXd value = residualValue(pair, x, Eigen::VectorXd::Zero(f.nv()));
  ASSERT_EQ(value.size(), f.nv());
  EXPECT_LT((value - expected).cwiseAbs().maxCoeff(), 1e-12);
  // A well-posed box: lower strictly below upper on every DoF.
  const BoxConstraint& box = asBox(pair);
  EXPECT_TRUE((box.lower_limit.array() < box.upper_limit.array()).all());
}

TEST(ConstraintTest, VelocityLimitResidualSlicesVelocity) {
  ConstraintFixture f;
  const auto pair = aligator_detail::buildVelocityLimit(f.space, f.rgm, *f.scene, VelocityLimit{});
  const Eigen::VectorXd x = deterministicState(f.space);

  // Residual = the velocity rows {nv..2nv-1}; the v-block is Euclidean, so this equals the
  // velocity.
  const Eigen::VectorXd expected = f.space.difference(f.space.neutral(), x).tail(f.nv());
  const Eigen::VectorXd value = residualValue(pair, x, Eigen::VectorXd::Zero(f.nv()));
  ASSERT_EQ(value.size(), f.nv());
  EXPECT_LT((value - expected).cwiseAbs().maxCoeff(), 1e-12);
}

TEST(ConstraintTest, FramePoseConstraintBoxOrdersTransThenRot) {
  ConstraintFixture f;
  FramePoseConstraint spec;
  spec.frame = kTipFrame;
  spec.tol_pos = 0.02;  // 2 cm
  spec.tol_rot = 0.10;  // ~5.7 deg
  const auto pair = aligator_detail::buildFramePoseConstraint(f.space, f.rgm, spec);

  const BoxConstraint& box = asBox(pair);
  ASSERT_EQ(box.upper_limit.size(), 6);
  Eigen::VectorXd expected_upper(6);
  expected_upper << 0.02, 0.02, 0.02, 0.10, 0.10, 0.10;  // [translation(3); rotation-log(3)]
  EXPECT_LT((box.upper_limit - expected_upper).cwiseAbs().maxCoeff(), 1e-15);
  EXPECT_LT((box.lower_limit + expected_upper).cwiseAbs().maxCoeff(), 1e-15);
}

// --- Decision gate: a user bound clamps to (intersects) the model's per-DoF -------------------

TEST(ConstraintTest, UserBoundClampsToModel) {
  ConstraintFixture f;
  const int nq = f.nq();
  const int nv = f.nv();

  const auto default_pos = aligator_detail::buildPositionLimit(f.space, f.rgm, *f.scene, {});
  const Eigen::VectorXd model_pos_upper = asBox(default_pos).upper_limit;

  // A looser-than-model user bound is ignored (clamped back to the model's).
  PositionLimit loose;
  loose.q_max = Eigen::VectorXd::Constant(nq, 1000.0);
  const auto loose_pos = aligator_detail::buildPositionLimit(f.space, f.rgm, *f.scene, loose);
  EXPECT_LT((asBox(loose_pos).upper_limit - model_pos_upper).cwiseAbs().maxCoeff(), 1e-12);

  // A tighter-than-model user bound wins per-DoF: upper = min(model, 0.05).
  PositionLimit tight;
  tight.q_max = Eigen::VectorXd::Constant(nq, 0.05);
  const auto tight_pos = aligator_detail::buildPositionLimit(f.space, f.rgm, *f.scene, tight);
  EXPECT_LT((asBox(tight_pos).upper_limit - model_pos_upper.cwiseMin(0.05)).cwiseAbs().maxCoeff(),
            1e-12);

  // Same rule for torque (symmetric user bound intersected with the model's effort limits).
  const auto default_tau = aligator_detail::buildTorqueLimit(f.space, f.rgm, TorqueLimit{});
  const Eigen::VectorXd model_tau_upper = asBox(default_tau).upper_limit;
  TorqueLimit tau_tight;
  tau_tight.tau_max = Eigen::VectorXd::Constant(nv, 0.5);
  const auto clamped_tau = aligator_detail::buildTorqueLimit(f.space, f.rgm, tau_tight);
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

  // Terminal window attaches to the terminal constraint stack, not to any stage.
  const auto pose_pair = aligator_detail::buildFramePoseConstraint(
      f.space, f.rgm,
      FramePoseConstraint{.frame = kTipFrame,
                          .target = Eigen::Matrix4d::Identity(),
                          .tol_pos = 0.01,
                          .tol_rot = 0.05});
  EXPECT_EQ(problem->term_cstrs_.size(), 0u);
  problem->addTerminalConstraint(pose_pair.func, pose_pair.set);
  EXPECT_EQ(problem->term_cstrs_.size(), 1u);
}

// Every window kind drives the production attach path (TrajectoryOptimizer::addConstraint ->
// attachConstraintPair): Range -> per-stage addConstraint, All -> every stage, Terminal ->
// addTerminalConstraint. The per-stage placement is asserted directly in
// WindowAttachesToInRangeStagesOnly; here we confirm each branch executes end-to-end through the
// public API and the assembled problem still solves.
TEST(ConstraintTest, AllWindowKindsAttachThroughPublicApiAndSolve) {
  auto scene = makeSo101Scene();
  TrajectoryOptimizer opt(scene, "arm", /*horizon=*/20, /*dt=*/0.05);

  PositionLimit pos;  // Range window -> the per-stage loop branch.
  EXPECT_NO_THROW(opt.addConstraint(pos, StageWindow::range(2, 8)));
  VelocityLimit vel;  // All window -> every stage.
  EXPECT_NO_THROW(opt.addConstraint(vel, StageWindow::all()));
  FramePoseConstraint reach;  // Terminal window -> addTerminalConstraint branch.
  reach.frame = kTipFrame;
  reach.tol_pos = 0.05;
  reach.tol_rot = 0.20;
  EXPECT_NO_THROW(opt.addConstraint(reach, StageWindow::terminal()));

  opt.build();
  const auto result = opt.solve(TrajOptSeed{});
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_GT(result->iterations, 0);
  EXPECT_TRUE(std::isfinite(result->max_constraint_violation));
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

// --- Collision constraints (custom witness-point normal) --------------------------------------

TEST(ConstraintTest, CollisionPairSelectionAndClassification) {
  ConstraintFixture f;
  const Eigen::VectorXd q0 = f.rgm.q0();

  // For the SO-101 arm group the reduced collision model has 45 link-vs-link (self) and 40
  // link-vs-static (base) pairs.
  SelfCollisionConstraint self;  // n_pairs = 0 => all candidates
  const auto self_pairs = aligator_detail::buildSelfCollisionConstraints(f.space, f.rgm, self, q0);
  EXPECT_EQ(self_pairs.size(), 45u);

  CollisionConstraint env;
  const auto env_pairs = aligator_detail::buildCollisionConstraints(f.space, f.rgm, env, q0);
  EXPECT_EQ(env_pairs.size(), 40u);

  // The two kinds partition the candidates: no overlap, and together all 85.
  EXPECT_EQ(self_pairs.size() + env_pairs.size(), 85u);

  // n_pairs caps the tracked set to the closest few.
  SelfCollisionConstraint limited;
  limited.n_pairs = 6;
  EXPECT_EQ(aligator_detail::buildSelfCollisionConstraints(f.space, f.rgm, limited, q0).size(), 6u);

  // Each pair is a 1-D box [d_min, +inf): signed distance >= d_min.
  SelfCollisionConstraint spec;
  spec.n_pairs = 1;
  spec.d_min = 0.03;
  const auto one = aligator_detail::buildSelfCollisionConstraints(f.space, f.rgm, spec, q0);
  ASSERT_EQ(one.size(), 1u);
  const BoxConstraint& box = asBox(one[0]);
  ASSERT_EQ(box.lower_limit.size(), 1);
  EXPECT_DOUBLE_EQ(box.lower_limit[0], 0.03);
  EXPECT_TRUE(std::isinf(box.upper_limit[0]) && box.upper_limit[0] > 0.0);
}

// CollisionDistanceResidual's witness-point analytic Jacobian must match finite differences, on
// every comfortably-separated self pair (coal's signed distance is smooth away from contact). The
// witness-direction normal makes the gradient exact even on mesh geometry, unlike aligator's stock
// FrameCollisionResidual (which reads coal's frequently-zero normal).
TEST(ConstraintTest, CollisionResidualJacobianMatchesFD) {
  ConstraintFixture f;
  using aligator_detail::CollisionDistanceResidual;
  const int ndx = f.space.ndx();
  const auto& reduced_model = f.rgm.reducedModel();
  const auto& reduced_geom = f.rgm.reducedCollisionModel();
  const auto& full_geom = f.rgm.fullCollisionModel();

  const Eigen::VectorXd x = deterministicState(f.space);

  // Per-pair distances at the test configuration (to pick separated self pairs).
  pinocchio::Data data(reduced_model);
  pinocchio::GeometryData geom_data(reduced_geom);
  pinocchio::computeDistances(reduced_model, data, reduced_geom, geom_data, x.head(f.nq()));

  const auto central_fd = [&](CollisionDistanceResidual& res, double eps) {
    Eigen::RowVectorXd fd(ndx);
    for (int i = 0; i < ndx; ++i) {
      Eigen::VectorXd dv = Eigen::VectorXd::Zero(ndx);
      auto d = res.createData();
      dv(i) = eps;
      res.evaluate(f.space.integrate(x, dv), *d);
      const double fp = d->value_[0];
      dv(i) = -eps;
      res.evaluate(f.space.integrate(x, dv), *d);
      fd(i) = (fp - d->value_[0]) / (2.0 * eps);
    }
    return fd;
  };

  int tested = 0;
  for (std::size_t k = 0; k < reduced_geom.collisionPairs.size(); ++k) {
    const auto& cp = reduced_geom.collisionPairs[k];
    const bool is_self = full_geom.geometryObjects[cp.first].parentJoint > 0 &&
                         full_geom.geometryObjects[cp.second].parentJoint > 0;
    const double dist = geom_data.distanceResults[k].min_distance;
    // Skip pairs below ~1 cm (coal's witness sits on a mesh edge there, so the field is
    // non-smooth).
    if (!is_self || dist < 0.02 || dist > 0.15) {
      continue;
    }
    CollisionDistanceResidual residual(ndx, f.nv(), reduced_model, reduced_geom, k);
    auto d = residual.createData();
    residual.evaluate(x, *d);
    residual.computeJacobians(x, *d);
    const Eigen::RowVectorXd jx = d->Jx_.row(0);

    const Eigen::RowVectorXd fd1 = central_fd(residual, 1e-6);
    const Eigen::RowVectorXd fd2 = central_fd(residual, 2.5e-7);
    // 1e-8: the two step sizes must agree, else this pair sits on a witness transition
    // (non-smooth).
    if ((fd1 - fd2).cwiseAbs().maxCoeff() > 1e-8) {
      continue;
    }
    // 1e-5: central-difference accuracy on the smooth (separated) field (matches to ~1e-11).
    EXPECT_LT((jx - fd1).cwiseAbs().maxCoeff(), 1e-5) << "self pair " << k << " dist " << dist;
    // Distance depends on q only: the velocity block of the state Jacobian is zero.
    EXPECT_LT(jx.tail(f.nv()).cwiseAbs().maxCoeff(), 1e-12) << "self pair " << k;
    ++tested;
  }
  ASSERT_GT(tested, 0) << "no separated self pair available to finite-difference";
}

// Inter-stage clearance caveat: the constraint holds at stage knots, but a straight-line segment
// whose two knots both clear d_min can pass through a CLOSER interior point. The SO-101
// wrist-vs-jaw pair, swept by the wrist joint, reaches an interior closest approach — straddling
// that angle makes the segment's midpoint nearer to contact than either knot.
TEST(ConstraintTest, InterStageClearanceCaveat) {
  ConstraintFixture f;
  const auto& reduced_model = f.rgm.reducedModel();
  const auto& reduced_geom = f.rgm.reducedCollisionModel();
  PairDistance pair_dist(reduced_model, reduced_geom);

  const std::size_t pair = findPair(reduced_geom, "wrist_link_1", "moving_jaw_so101_v1_link_0");
  ASSERT_NE(pair, std::numeric_limits<std::size_t>::max()) << "expected SO-101 wrist/jaw pair";

  // Find the joint whose sweep gives this pair the deepest interior closest-approach (a V-shape:
  // farther at both range ends than at an interior angle). Self-calibrating, so it survives
  // re-indexing; asserts such a joint exists.
  const int samples = 41;
  const Eigen::VectorXd base = f.rgm.q0();
  int best_joint = -1;
  int best_index = -1;
  double best_dip = 0.0;
  double best_theta = 0.0;
  double best_lo = 0.0;
  double best_hi = 0.0;
  for (int j = 0; j < f.nv(); ++j) {
    const double lo = std::max(-3.1, reduced_model.lowerPositionLimit(j));
    const double hi = std::min(3.1, reduced_model.upperPositionLimit(j));
    if (!(hi > lo)) {
      continue;
    }
    std::vector<double> sweep(samples);
    int argmin = 0;
    for (int s = 0; s < samples; ++s) {
      Eigen::VectorXd q = base;
      q(j) = lo + (hi - lo) * s / (samples - 1);
      sweep[s] = pair_dist.at(q, pair);
      if (sweep[s] < sweep[argmin]) {
        argmin = s;
      }
    }
    const double dip = std::min(sweep[0], sweep[samples - 1]) - sweep[argmin];
    if (argmin >= 2 && argmin <= samples - 3 && sweep[argmin] > 0.003 && dip > best_dip) {
      best_dip = dip;
      best_joint = j;
      best_index = argmin;
      best_theta = lo + (hi - lo) * argmin / (samples - 1);
      best_lo = lo;
      best_hi = hi;
    }
  }
  ASSERT_GE(best_joint, 0) << "no interior self-approach found to demonstrate the caveat";

  // Knots straddle the closest-approach angle by a few steps; the midpoint is the closest approach.
  const double step = (best_hi - best_lo) / (samples - 1);
  const double delta = std::max(3, best_index / 2) * step;
  Eigen::VectorXd q_mid = base;
  q_mid(best_joint) = best_theta;
  Eigen::VectorXd q_a = base;
  q_a(best_joint) = best_theta - delta;
  Eigen::VectorXd q_b = base;
  q_b(best_joint) = best_theta + delta;

  const double d_mid = pair_dist.at(q_mid, pair);
  const double d_a = pair_dist.at(q_a, pair);
  const double d_b = pair_dist.at(q_b, pair);

  // Both knots are farther from contact than the segment's interior: a d_min in (d_mid,
  // min(d_a,d_b)] passes at the knots but is violated between them — knot feasibility does not
  // imply trajectory feasibility (hence inter-stage clearance needs fine FK resampling).
  EXPECT_LT(d_mid, d_a) << "d_mid=" << d_mid << " d_a=" << d_a;
  EXPECT_LT(d_mid, d_b) << "d_mid=" << d_mid << " d_b=" << d_b;
}

// Self-collision attaches through the public API and the problem solves to a finite result.
// Deliberately tiny (short horizon, few pairs, few iterations) because each mesh-distance residual
// is evaluated per stage per DDP iteration: this checks wiring, not convergence.
TEST(ConstraintTest, SelfCollisionConstraintAttachesAndSolves) {
  auto scene = makeSo101Scene();
  TrajOptOptions options;
  options.max_iters =
      3;  // wiring smoke test, not a convergence test — keep the mesh-distance cost low
  TrajectoryOptimizer opt(scene, "arm", /*horizon=*/4, /*dt=*/0.1, options);
  SelfCollisionConstraint self;
  self.n_pairs = 2;
  self.d_min = 0.005;                        // small clearance the seed already satisfies
  EXPECT_NO_THROW(opt.addConstraint(self));  // all stages
  // Terminal is legal too (collision residual is a UnaryFunction of x only).
  EXPECT_NO_THROW(opt.addConstraint(self, StageWindow::terminal()));

  opt.build();
  const auto result = opt.solve(TrajOptSeed{});
  ASSERT_TRUE(result.has_value()) << result.error();
  EXPECT_TRUE(std::isfinite(result->max_constraint_violation));
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
  PositionLimit limit;
  limit.q_min = Eigen::VectorXd::Zero(opt.nq() + 1);  // wrong size
  EXPECT_THROW(opt.addConstraint(limit), std::invalid_argument);
}

TEST(ConstraintTest, AddConstraintAfterBuildThrowsThenResetAllows) {
  auto scene = makeSo101Scene();
  TrajectoryOptimizer opt(scene, "arm", /*horizon=*/8, /*dt=*/0.02);

  VelocityLimit limit;
  EXPECT_NO_THROW(opt.addConstraint(limit));
  opt.build();
  EXPECT_THROW(opt.addConstraint(limit), std::logic_error);  // locked after build()
  ASSERT_TRUE(opt.solve(TrajOptSeed{}).has_value());

  opt.resetProblem();
  EXPECT_NO_THROW(opt.addConstraint(limit));  // legal again
}

}  // namespace roboplan
