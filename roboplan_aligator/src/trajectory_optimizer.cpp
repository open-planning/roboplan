#include <roboplan_aligator/trajectory_optimizer.hpp>

#include <cstddef>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <algorithm>  // std::min

#include <aligator/fwd.hpp>                             // VerboseLevel
#include <aligator/solvers/proxddp/solver-proxddp.hpp>  // SolverProxDDPTpl
#include <pinocchio/algorithm/joint-configuration.hpp>  // pinocchio::interpolate

#include <roboplan/core/scene.hpp>

#include "constraint_factory.hpp"
#include "cost_factory.hpp"
#include "problem_builder.hpp"
#include "reduced_group_model.hpp"

namespace roboplan {

namespace {

// Stack a reduced [q; v] into a single state vector x (size nq + nv), matching aligator's
// MultibodyPhaseSpace layout (q first, then v).
Eigen::VectorXd stackState(const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
  Eigen::VectorXd x(q.size() + v.size());
  x << q, v;
  return x;
}

}  // namespace

// --- PIMPL: owns all aligator/pinocchio state --------------------------------------------------

struct TrajectoryOptimizer::Impl {
  std::shared_ptr<Scene> scene;
  std::string group_name;
  int horizon;
  double dt;
  TrajOptOptions options;

  ReducedGroupModel rgm;                              // reduced pinocchio model snapshot (§3.1)
  aligator_detail::PhaseSpace space;                  // x = [q; v] on the reduced model
  Eigen::VectorXd x0;                                 // current fixed initial state [q0; v0]
  std::unique_ptr<aligator_detail::Problem> problem;  // assembled N-stage shell
  aligator::SolverProxDDPTpl<double> solver;

  // True once build() has run solver.setup() on the assembled problem. addCost / addConstraint /
  // resetProblem are illegal while locked (lifecycle §3.4); solve() requires it. setup is deferred
  // to build() (not the ctor) so that costs/constraints added via addCost/addConstraint are part of
  // the problem structure the workspace is allocated for.
  bool locked = false;

  Impl(std::shared_ptr<Scene> s, std::string g, int h, double d, TrajOptOptions o)
      : scene(std::move(s)), group_name(std::move(g)), horizon(h), dt(d), options(o),
        rgm(*scene, group_name), space(aligator_detail::makePhaseSpace(rgm.reducedModel())),
        x0(stackState(rgm.q0(), rgm.v0())),
        problem(aligator_detail::buildProblemShell(space, x0, horizon, dt, options)), solver() {}
};

namespace {

// The in-problem CostStacks a window targets: each in-range stage's cost sum, or the terminal cost.
// Returned as pointers into the assembled problem so attached costs mutate its final home (§3.5).
std::vector<aligator_detail::CostStack*>
resolveTargetStacks(aligator_detail::Problem& problem, const StageWindow& window, int horizon) {
  const auto as_stack = [](aligator::CostAbstractTpl<double>& cost) {
    auto* stack = dynamic_cast<aligator_detail::CostStack*>(&cost);
    if (stack == nullptr) {
      throw std::logic_error("TrajectoryOptimizer::addCost: in-problem cost is not a CostStack.");
    }
    return stack;
  };
  std::vector<aligator_detail::CostStack*> stacks;
  if (window.isTerminal()) {
    stacks.push_back(as_stack(*problem.term_cost_));
  } else {
    for (const int k : window.resolveStages(horizon)) {
      stacks.push_back(as_stack(*problem.stages_[static_cast<std::size_t>(k)]->cost_));
    }
  }
  return stacks;
}

// Lifecycle guard: costs and constraints may only be added before build() (or after
// resetProblem()).
void requireUnlocked(bool locked) {
  if (locked) {
    throw std::logic_error(
        "TrajectoryOptimizer: costs and constraints may only be added before build(); call "
        "resetProblem() to rebuild the shell and add more.");
  }
}

// Attaches a built (residual, box) constraint pair to every in-range stage of `window`, or to the
// terminal node. Both StageModel::addConstraint and TrajOptProblem::addTerminalConstraint deep-copy
// (pushBack), so one built pair is reused across the whole window (API_NOTES.md, Prompt 7).
void attachConstraintPair(aligator_detail::Problem& problem,
                          const aligator_detail::ConstraintPair& pair, const StageWindow& window,
                          int horizon) {
  if (window.isTerminal()) {
    problem.addTerminalConstraint(pair.func, pair.set);
  } else {
    for (const int k : window.resolveStages(horizon)) {
      problem.stages_[static_cast<std::size_t>(k)]->addConstraint(pair.func, pair.set);
    }
  }
}

}  // namespace

// --- Special members -------------------------------------------------------------------------

TrajectoryOptimizer::TrajectoryOptimizer(std::shared_ptr<Scene> scene, std::string group_name,
                                         int horizon, double dt, TrajOptOptions options) {
  if (scene == nullptr) {
    throw std::invalid_argument("TrajectoryOptimizer: scene must not be null.");
  }
  if (horizon <= 0) {
    throw std::invalid_argument(
        "TrajectoryOptimizer: horizon (number of stages) must be > 0, got " +
        std::to_string(horizon) + ".");
  }
  if (dt <= 0.0) {
    throw std::invalid_argument("TrajectoryOptimizer: dt must be > 0, got " + std::to_string(dt) +
                                ".");
  }
  impl_ = std::make_unique<Impl>(std::move(scene), std::move(group_name), horizon, dt, options);
}

TrajectoryOptimizer::~TrajectoryOptimizer() = default;
TrajectoryOptimizer::TrajectoryOptimizer(TrajectoryOptimizer&&) noexcept = default;
TrajectoryOptimizer& TrajectoryOptimizer::operator=(TrajectoryOptimizer&&) noexcept = default;

// --- Introspection ---------------------------------------------------------------------------

int TrajectoryOptimizer::horizon() const { return impl_->horizon; }
double TrajectoryOptimizer::dt() const { return impl_->dt; }
int TrajectoryOptimizer::nq() const { return impl_->rgm.nq(); }
int TrajectoryOptimizer::nv() const { return impl_->rgm.nv(); }
int TrajectoryOptimizer::nx() const { return impl_->rgm.nq() + impl_->rgm.nv(); }

// --- Initial state ---------------------------------------------------------------------------

void TrajectoryOptimizer::setInitialState(const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
  const int nq = impl_->rgm.nq();
  const int nv = impl_->rgm.nv();
  if (q.size() != nq) {
    throw std::invalid_argument("TrajectoryOptimizer::setInitialState: q has size " +
                                std::to_string(q.size()) +
                                ", expected reduced nq = " + std::to_string(nq) + ".");
  }
  Eigen::VectorXd v_used = v;
  if (v_used.size() == 0) {
    v_used = Eigen::VectorXd::Zero(nv);
  } else if (v_used.size() != nv) {
    throw std::invalid_argument("TrajectoryOptimizer::setInitialState: v has size " +
                                std::to_string(v.size()) +
                                ", expected reduced nv = " + std::to_string(nv) + ".");
  }
  impl_->x0 = stackState(q, v_used);
  // Updates the initial-condition constraint target in place (no rebuild, §3.4).
  impl_->problem->setInitState(impl_->x0);
}

// --- Costs (design §4.3) ---------------------------------------------------------------------

CostHandle TrajectoryOptimizer::addCost(const FramePoseCost& cost, const StageWindow& window,
                                        double weight) {
  Impl& im = *impl_;
  requireUnlocked(im.locked);
  auto handle = std::make_unique<CostHandle::Impl>();
  handle->kind = CostHandle::Impl::Kind::Pose;
  for (auto* stack : resolveTargetStacks(*im.problem, window, im.horizon)) {
    handle->pose_setters.push_back(
        aligator_detail::attachFramePoseCost(*stack, im.space, im.rgm, cost, weight));
  }
  return CostHandle(std::move(handle));
}

CostHandle TrajectoryOptimizer::addCost(const FrameAxisCost& cost, const StageWindow& window,
                                        double weight) {
  Impl& im = *impl_;
  requireUnlocked(im.locked);
  auto handle = std::make_unique<CostHandle::Impl>();
  handle->kind = CostHandle::Impl::Kind::Vector;
  handle->expected_size = 3;  // world-axis direction
  for (auto* stack : resolveTargetStacks(*im.problem, window, im.horizon)) {
    handle->vector_setters.push_back(
        aligator_detail::attachFrameAxisCost(*stack, im.space, im.rgm, cost, weight));
  }
  return CostHandle(std::move(handle));
}

CostHandle TrajectoryOptimizer::addCost(const ConfigurationCost& cost, const StageWindow& window,
                                        double weight) {
  Impl& im = *impl_;
  requireUnlocked(im.locked);
  auto handle = std::make_unique<CostHandle::Impl>();
  handle->kind = CostHandle::Impl::Kind::Vector;
  handle->expected_size = im.rgm.nq();  // target is a reduced configuration
  for (auto* stack : resolveTargetStacks(*im.problem, window, im.horizon)) {
    handle->vector_setters.push_back(
        aligator_detail::attachConfigurationCost(*stack, im.space, im.rgm, cost, weight));
  }
  return CostHandle(std::move(handle));
}

CostHandle TrajectoryOptimizer::addCost(const ControlCost& cost, const StageWindow& window,
                                        double weight) {
  Impl& im = *impl_;
  requireUnlocked(im.locked);
  auto handle = std::make_unique<CostHandle::Impl>();
  handle->kind = CostHandle::Impl::Kind::Vector;
  handle->expected_size = im.rgm.nv();  // target is a control (torque) vector
  for (auto* stack : resolveTargetStacks(*im.problem, window, im.horizon)) {
    handle->vector_setters.push_back(
        aligator_detail::attachControlCost(*stack, im.space, im.rgm, cost, weight));
  }
  return CostHandle(std::move(handle));
}

CostHandle TrajectoryOptimizer::addCost(const VelocityCost& cost, const StageWindow& window,
                                        double weight) {
  Impl& im = *impl_;
  requireUnlocked(im.locked);
  auto handle = std::make_unique<CostHandle::Impl>();
  handle->kind = CostHandle::Impl::Kind::Vector;
  handle->expected_size = im.rgm.nv();  // target is a velocity vector
  for (auto* stack : resolveTargetStacks(*im.problem, window, im.horizon)) {
    handle->vector_setters.push_back(
        aligator_detail::attachVelocityCost(*stack, im.space, im.rgm, cost, weight));
  }
  return CostHandle(std::move(handle));
}

// --- Constraints (design §4.4) ---------------------------------------------------------------

void TrajectoryOptimizer::addConstraint(const PositionLimit& constraint,
                                        const StageWindow& window) {
  Impl& im = *impl_;
  requireUnlocked(im.locked);
  const auto pair = aligator_detail::buildPositionLimit(im.space, im.rgm, *im.scene, constraint);
  attachConstraintPair(*im.problem, pair, window, im.horizon);
}

void TrajectoryOptimizer::addConstraint(const VelocityLimit& constraint,
                                        const StageWindow& window) {
  Impl& im = *impl_;
  requireUnlocked(im.locked);
  const auto pair = aligator_detail::buildVelocityLimit(im.space, im.rgm, *im.scene, constraint);
  attachConstraintPair(*im.problem, pair, window, im.horizon);
}

void TrajectoryOptimizer::addConstraint(const TorqueLimit& constraint, const StageWindow& window) {
  Impl& im = *impl_;
  requireUnlocked(im.locked);
  if (window.isTerminal()) {
    throw std::invalid_argument(
        "TrajectoryOptimizer::addConstraint: a TorqueLimit cannot target the Terminal window; the "
        "terminal node has no control.");
  }
  const auto pair = aligator_detail::buildTorqueLimit(im.space, im.rgm, constraint);
  attachConstraintPair(*im.problem, pair, window, im.horizon);
}

void TrajectoryOptimizer::addConstraint(const FramePoseConstraint& constraint,
                                        const StageWindow& window) {
  Impl& im = *impl_;
  requireUnlocked(im.locked);
  const auto pair = aligator_detail::buildFramePoseConstraint(im.space, im.rgm, constraint);
  attachConstraintPair(*im.problem, pair, window, im.horizon);
}

void TrajectoryOptimizer::addConstraint(const SelfCollisionConstraint& constraint,
                                        const StageWindow& window) {
  Impl& im = *impl_;
  requireUnlocked(im.locked);
  // Pairs are pre-selected at the current initial configuration (reuse path — no per-iteration
  // re-selection, §5). One collision residual + box is attached to the window per tracked pair.
  const Eigen::VectorXd q_select = im.x0.head(im.rgm.nq());
  for (const auto& pair :
       aligator_detail::buildSelfCollisionConstraints(im.space, im.rgm, constraint, q_select)) {
    attachConstraintPair(*im.problem, pair, window, im.horizon);
  }
}

void TrajectoryOptimizer::addConstraint(const CollisionConstraint& constraint,
                                        const StageWindow& window) {
  Impl& im = *impl_;
  requireUnlocked(im.locked);
  const Eigen::VectorXd q_select = im.x0.head(im.rgm.nq());
  for (const auto& pair :
       aligator_detail::buildCollisionConstraints(im.space, im.rgm, constraint, q_select)) {
    attachConstraintPair(*im.problem, pair, window, im.horizon);
  }
}

void TrajectoryOptimizer::build() {
  Impl& im = *impl_;
  if (im.locked) {
    return;  // idempotent: already built (setup() run, problem structure frozen).
  }
  // Allocate the solver workspace for the assembled problem and freeze it: no more addCost /
  // addConstraint / resetProblem until resetProblem() unlocks (lifecycle §3.4). Deferred to here
  // (not the ctor) so every cost/constraint added is part of the structure setup() allocates for.
  im.solver.setup(*im.problem);
  im.locked = true;
}

void TrajectoryOptimizer::resetProblem() {
  Impl& im = *impl_;
  // Rebuild the empty shell (default control regularization only). Any outstanding CostHandle now
  // dangles (its residual pointers referenced the discarded problem). build() is required again.
  im.problem = aligator_detail::buildProblemShell(im.space, im.x0, im.horizon, im.dt, im.options);
  im.locked = false;
}

// --- Solve -----------------------------------------------------------------------------------

tl::expected<TrajOptResult, std::string> TrajectoryOptimizer::solve(const TrajOptSeed& seed) {
  Impl& im = *impl_;
  const auto num_stages = static_cast<std::size_t>(im.horizon);
  const int nx = im.rgm.nq() + im.rgm.nv();
  const int nu = im.rgm.nv();

  // The problem must be finalized (build()) before it can be solved (maintainer decision, Prompt 9:
  // solve does not auto-build). A missing build() is a recoverable per-call misuse, not a throw.
  if (!im.locked) {
    return tl::make_unexpected(
        "TrajectoryOptimizer::solve: the problem has not been built; call build() first (add all "
        "costs/constraints, then build(), then solve()).");
  }

  // Validate a provided seed (empty vectors are left for aligator to default-initialize). A
  // dimension mismatch is a recoverable per-call failure (numerics rule), not a throw.
  if (!seed.xs.empty()) {
    if (seed.xs.size() != num_stages + 1) {
      return tl::make_unexpected(
          "TrajectoryOptimizer::solve: seed.xs has " + std::to_string(seed.xs.size()) +
          " entries, expected horizon + 1 = " + std::to_string(num_stages + 1) + ".");
    }
    for (std::size_t k = 0; k < seed.xs.size(); ++k) {
      if (seed.xs[k].size() != nx) {
        return tl::make_unexpected("TrajectoryOptimizer::solve: seed.xs[" + std::to_string(k) +
                                   "] has size " + std::to_string(seed.xs[k].size()) +
                                   ", expected nx = " + std::to_string(nx) + ".");
      }
    }
  }
  if (!seed.us.empty()) {
    if (seed.us.size() != num_stages) {
      return tl::make_unexpected(
          "TrajectoryOptimizer::solve: seed.us has " + std::to_string(seed.us.size()) +
          " entries, expected horizon = " + std::to_string(num_stages) + ".");
    }
    for (std::size_t k = 0; k < seed.us.size(); ++k) {
      if (seed.us[k].size() != nu) {
        return tl::make_unexpected("TrajectoryOptimizer::solve: seed.us[" + std::to_string(k) +
                                   "] has size " + std::to_string(seed.us[k].size()) +
                                   ", expected nu = " + std::to_string(nu) + ".");
      }
    }
  }

  // Apply options by direct public-field assignment before the run (maintainer decision, Prompt 5):
  // honours §3.4's "max_iters/tol editable between solves without a rebuild". mu_init_ is consumed
  // by run() (setAlmPenalty(mu_init_), solver-proxddp.hxx:460), not by setup(), so assigning it
  // here means every solve honours the current options.mu_init.
  im.solver.target_tol_ = im.options.tol;
  im.solver.mu_init_ = im.options.mu_init;
  im.solver.max_iters = static_cast<std::size_t>(im.options.max_iters);
  im.solver.verbose_ =
      im.options.verbose ? aligator::VerboseLevel::VERBOSE : aligator::VerboseLevel::QUIET;

  bool converged = false;
  try {
    converged = im.solver.run(*im.problem, seed.xs, seed.us);
  } catch (const std::exception& e) {
    return tl::make_unexpected(std::string("TrajectoryOptimizer::solve: aligator solver threw: ") +
                               e.what());
  }

  const auto& res = im.solver.results_;

  TrajOptResult out;
  out.converged = converged;
  out.iterations = static_cast<int>(res.num_iters);
  out.cost = res.traj_cost_;
  out.max_constraint_violation = res.prim_infeas;
  out.xs = res.xs;
  out.us = res.us;
  out.controls = res.us;  // actuation B = I, so the applied torque equals the control (§3.2).

  // Semantic views: split each state x = [q; v] and sample times k*dt. Positions use nq, velocities
  // use nv (never assume nq == nv, numerics rule) even though nu == nv here for actuation B = I.
  const int nq = im.rgm.nq();
  const int nv = im.rgm.nv();
  out.trajectory.times.reserve(res.xs.size());
  out.trajectory.positions.reserve(res.xs.size());
  out.trajectory.velocities.reserve(res.xs.size());
  for (std::size_t k = 0; k < res.xs.size(); ++k) {
    out.trajectory.times.push_back(static_cast<double>(k) * im.dt);
    out.trajectory.positions.emplace_back(res.xs[k].head(nq));
    out.trajectory.velocities.emplace_back(res.xs[k].segment(nq, nv));
  }

  return out;
}

tl::expected<TrajOptResult, std::string> TrajectoryOptimizer::solve(const TrajOptResult& previous) {
  // Warm-start from a previous solution: its states/controls are already in the seed layout
  // (xs size N+1, us size N). Dispatch to the seed overload (§3.6).
  return solve(TrajOptSeed{.xs = previous.xs, .us = previous.us});
}

// --- Warm-start helpers (design §3.6) --------------------------------------------------------

TrajOptSeed
TrajectoryOptimizer::interpolatePath(const std::vector<Eigen::VectorXd>& waypoints) const {
  const Impl& im = *impl_;
  const int nq = im.rgm.nq();
  const int nv = im.rgm.nv();
  if (waypoints.empty()) {
    throw std::invalid_argument(
        "TrajectoryOptimizer::interpolatePath: need at least one waypoint.");
  }
  for (std::size_t i = 0; i < waypoints.size(); ++i) {
    if (waypoints[i].size() != nq) {
      throw std::invalid_argument("TrajectoryOptimizer::interpolatePath: waypoint " +
                                  std::to_string(i) + " has size " +
                                  std::to_string(waypoints[i].size()) +
                                  ", expected reduced nq = " + std::to_string(nq) + ".");
    }
  }

  // Straight-line joint interpolation onto the N+1 grid, Lie-group-aware on the REDUCED model
  // (pinocchio::interpolate; Scene::interpolate is bound to the full model, so it cannot target the
  // reduced sub-model — API_NOTES §3.6). Multiple waypoints form a piecewise-linear path evenly
  // parameterized over [0, 1]; velocities and controls are zero (design §3.6).
  const pinocchio::Model& model = im.rgm.reducedModel();
  const int num_segments = static_cast<int>(waypoints.size()) - 1;

  TrajOptSeed seed;
  seed.xs.reserve(static_cast<std::size_t>(im.horizon) + 1);
  for (int k = 0; k <= im.horizon; ++k) {
    Eigen::VectorXd q(nq);
    if (num_segments == 0) {
      q = waypoints.front();  // single waypoint: a constant seed at that configuration.
    } else {
      const double t = static_cast<double>(k) / static_cast<double>(im.horizon);  // global [0, 1]
      const double scaled = t * num_segments;
      int segment = std::min(static_cast<int>(scaled), num_segments - 1);  // clamp the t == 1 end
      const double alpha = scaled - segment;
      q = pinocchio::interpolate(model, waypoints[static_cast<std::size_t>(segment)],
                                 waypoints[static_cast<std::size_t>(segment) + 1], alpha);
    }
    Eigen::VectorXd x(nq + nv);
    x << q, Eigen::VectorXd::Zero(nv);
    seed.xs.push_back(std::move(x));
  }
  seed.us.assign(static_cast<std::size_t>(im.horizon), Eigen::VectorXd::Zero(nv));
  return seed;
}

TrajOptSeed TrajectoryOptimizer::shift(const TrajOptResult& result, int n_steps) const {
  const Impl& im = *impl_;
  const auto num_stages = static_cast<std::size_t>(im.horizon);
  if (n_steps < 0) {
    throw std::invalid_argument("TrajectoryOptimizer::shift: n_steps must be >= 0, got " +
                                std::to_string(n_steps) + ".");
  }
  if (result.xs.size() != num_stages + 1 || result.us.size() != num_stages) {
    throw std::invalid_argument("TrajectoryOptimizer::shift: result has " +
                                std::to_string(result.xs.size()) + " states / " +
                                std::to_string(result.us.size()) + " controls, expected " +
                                std::to_string(num_stages + 1) + " / " +
                                std::to_string(num_stages) + " for this optimizer's horizon.");
  }

  // Advance the horizon by n_steps: drop the first n_steps knots and repeat the last state/control
  // to refill the tail (the receding-horizon "hold" convention; maintainer decision, Prompt 9 —
  // matches aligator's own cycleAppend, which duplicates the final knot). n_steps is clamped per
  // index.
  TrajOptSeed seed;
  seed.xs.reserve(num_stages + 1);
  for (std::size_t k = 0; k <= num_stages; ++k) {
    const std::size_t src = std::min(k + static_cast<std::size_t>(n_steps), num_stages);
    seed.xs.push_back(result.xs[src]);
  }
  seed.us.reserve(num_stages);
  for (std::size_t k = 0; k < num_stages; ++k) {
    const std::size_t src = std::min(k + static_cast<std::size_t>(n_steps), num_stages - 1);
    seed.us.push_back(result.us[src]);
  }
  return seed;
}

}  // namespace roboplan
