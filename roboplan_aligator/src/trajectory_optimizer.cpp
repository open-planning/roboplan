#include <roboplan_aligator/trajectory_optimizer.hpp>

#include <cstddef>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <algorithm>  // std::min
#include <variant>

#include <aligator/fwd.hpp>                             // VerboseLevel
#include <aligator/solvers/proxddp/solver-proxddp.hpp>  // SolverProxDDPTpl
#include <pinocchio/algorithm/joint-configuration.hpp>  // pinocchio::interpolate

#include <roboplan/core/scene.hpp>

#include <roboplan_aligator/constraint_factory.hpp>
#include <roboplan_aligator/cost_factory.hpp>
#include <roboplan_aligator/problem_builder.hpp>
#include <roboplan_aligator/reduced_group_model.hpp>

namespace roboplan {

namespace {

// Stack a reduced [q; v] into a single state vector x (size nq + nv), matching aligator's
// MultibodyPhaseSpace layout (q first, then v).
Eigen::VectorXd stackState(const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
  Eigen::VectorXd x(q.size() + v.size());
  x << q, v;
  return x;
}

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
                                         int horizon, double dt, TrajOptOptions options)
    : scene_([&] {
        if (scene == nullptr) {
          throw std::invalid_argument("TrajectoryOptimizer: scene must not be null.");
        }
        if (horizon <= 0) {
          throw std::invalid_argument(
              "TrajectoryOptimizer: horizon (number of stages) must be > 0, got " +
              std::to_string(horizon) + ".");
        }
        if (dt <= 0.0) {
          throw std::invalid_argument("TrajectoryOptimizer: dt must be > 0, got " +
                                      std::to_string(dt) + ".");
        }
        return std::move(scene);
      }()),
      group_name_(std::move(group_name)), horizon_(horizon), dt_(dt), options_(options),
      rgm_(*scene_, group_name_), space_(aligator_detail::makePhaseSpace(rgm_.reducedModel())),
      x0_(stackState(rgm_.q0(), rgm_.v0())),
      problem_(aligator_detail::buildProblemShell(space_, x0_, horizon_, dt_, options_)) {}

TrajectoryOptimizer::~TrajectoryOptimizer() = default;

TrajectoryOptimizer::TrajectoryOptimizer(TrajectoryOptimizer&& other) noexcept
    : scene_(std::move(other.scene_)), group_name_(std::move(other.group_name_)),
      horizon_(other.horizon_), dt_(other.dt_), options_(other.options_),
      rgm_(*scene_, group_name_), space_(aligator_detail::makePhaseSpace(rgm_.reducedModel())),
      x0_(std::move(other.x0_)), problem_(std::move(other.problem_)),
      solver_(std::move(other.solver_)), locked_(other.locked_) {}

// rgm_ (and the space_ derived from it) hold references into the originating scene and cannot be
// moved or reassigned, so move assignment reconstructs the members in place from the moved-from
// optimizer's scene/group.
TrajectoryOptimizer& TrajectoryOptimizer::operator=(TrajectoryOptimizer&& other) noexcept {
  if (this != &other) {
    this->~TrajectoryOptimizer();
    new (this) TrajectoryOptimizer(std::move(other));
  }
  return *this;
}

// --- Introspection ---------------------------------------------------------------------------

int TrajectoryOptimizer::horizon() const { return horizon_; }
double TrajectoryOptimizer::dt() const { return dt_; }
int TrajectoryOptimizer::nq() const { return rgm_.nq(); }
int TrajectoryOptimizer::nv() const { return rgm_.nv(); }
int TrajectoryOptimizer::nx() const { return rgm_.nq() + rgm_.nv(); }

// --- Initial state ---------------------------------------------------------------------------

void TrajectoryOptimizer::setInitialState(const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
  const int nq = rgm_.nq();
  const int nv = rgm_.nv();
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
  x0_ = stackState(q, v_used);
  // Updates the initial-condition constraint target in place (no rebuild, §3.4).
  problem_->setInitState(x0_);
}

// --- Costs (design §4.3) ---------------------------------------------------------------------

CostHandle TrajectoryOptimizer::addCost(const CostSpec& cost, const StageWindow& window,
                                        double weight) {
  requireUnlocked(locked_);

  return std::visit(
      [&](const auto& spec) -> CostHandle {
        using T = std::decay_t<decltype(spec)>;
        auto handle = std::make_unique<CostHandle::Impl>();

        if constexpr (std::is_same_v<T, FramePoseCost>) {
          handle->kind = CostHandle::Impl::Kind::Pose;
          for (auto* stack : resolveTargetStacks(*problem_, window, horizon_)) {
            handle->pose_setters.push_back(
                aligator_detail::attachFramePoseCost(*stack, space_, rgm_, spec, weight));
          }
        } else if constexpr (std::is_same_v<T, FrameAxisCost>) {
          handle->kind = CostHandle::Impl::Kind::Vector;
          handle->expected_size = 3;
          for (auto* stack : resolveTargetStacks(*problem_, window, horizon_)) {
            handle->vector_setters.push_back(
                aligator_detail::attachFrameAxisCost(*stack, space_, rgm_, spec, weight));
          }
        } else if constexpr (std::is_same_v<T, ConfigurationCost>) {
          handle->kind = CostHandle::Impl::Kind::Vector;
          handle->expected_size = rgm_.nq();
          for (auto* stack : resolveTargetStacks(*problem_, window, horizon_)) {
            handle->vector_setters.push_back(
                aligator_detail::attachConfigurationCost(*stack, space_, rgm_, spec, weight));
          }
        } else if constexpr (std::is_same_v<T, ControlCost>) {
          handle->kind = CostHandle::Impl::Kind::Vector;
          handle->expected_size = rgm_.nv();
          for (auto* stack : resolveTargetStacks(*problem_, window, horizon_)) {
            handle->vector_setters.push_back(
                aligator_detail::attachControlCost(*stack, space_, rgm_, spec, weight));
          }
        } else if constexpr (std::is_same_v<T, VelocityCost>) {
          handle->kind = CostHandle::Impl::Kind::Vector;
          handle->expected_size = rgm_.nv();
          for (auto* stack : resolveTargetStacks(*problem_, window, horizon_)) {
            handle->vector_setters.push_back(
                aligator_detail::attachVelocityCost(*stack, space_, rgm_, spec, weight));
          }
        }

        return CostHandle(std::move(handle));
      },
      cost);
}

CostHandle TrajectoryOptimizer::addCost(xyz::polymorphic<aligator::CostAbstractTpl<double>> cost,
                                        const StageWindow& window, double weight) {
  requireUnlocked(locked_);
  for (auto* stack : resolveTargetStacks(*problem_, window, horizon_)) {
    stack->addCost(std::move(cost), weight);
  }
  // Return a default handle (no target setters for custom costs).
  return CostHandle(std::make_unique<CostHandle::Impl>());
}

// --- Constraints (design §4.4) ---------------------------------------------------------------

void TrajectoryOptimizer::addConstraint(const ConstraintSpec& constraint,
                                        const StageWindow& window) {
  requireUnlocked(locked_);

  std::visit(
      [&](const auto& spec) {
        using T = std::decay_t<decltype(spec)>;

        if constexpr (std::is_same_v<T, PositionLimit>) {
          const auto pair = aligator_detail::buildPositionLimit(space_, rgm_, *scene_, spec);
          attachConstraintPair(*problem_, pair, window, horizon_);
        } else if constexpr (std::is_same_v<T, VelocityLimit>) {
          const auto pair = aligator_detail::buildVelocityLimit(space_, rgm_, *scene_, spec);
          attachConstraintPair(*problem_, pair, window, horizon_);
        } else if constexpr (std::is_same_v<T, TorqueLimit>) {
          if (window.isTerminal()) {
            throw std::invalid_argument(
                "TrajectoryOptimizer::addConstraint: a TorqueLimit cannot target the Terminal "
                "window; the terminal node has no control.");
          }
          const auto pair = aligator_detail::buildTorqueLimit(space_, rgm_, spec);
          attachConstraintPair(*problem_, pair, window, horizon_);
        } else if constexpr (std::is_same_v<T, FramePoseConstraint>) {
          const auto pair = aligator_detail::buildFramePoseConstraint(space_, rgm_, spec);
          attachConstraintPair(*problem_, pair, window, horizon_);
        } else if constexpr (std::is_same_v<T, SelfCollisionConstraint>) {
          const Eigen::VectorXd q_select = x0_.head(rgm_.nq());
          for (const auto& pair :
               aligator_detail::buildSelfCollisionConstraints(space_, rgm_, spec, q_select)) {
            attachConstraintPair(*problem_, pair, window, horizon_);
          }
        } else if constexpr (std::is_same_v<T, CollisionConstraint>) {
          const Eigen::VectorXd q_select = x0_.head(rgm_.nq());
          for (const auto& pair :
               aligator_detail::buildCollisionConstraints(space_, rgm_, spec, q_select)) {
            attachConstraintPair(*problem_, pair, window, horizon_);
          }
        }
      },
      constraint);
}

void TrajectoryOptimizer::addConstraint(
    xyz::polymorphic<aligator::StageFunctionTpl<double>> residual,
    xyz::polymorphic<aligator::ConstraintSetTpl<double>> set, const StageWindow& window) {
  requireUnlocked(locked_);
  const aligator_detail::ConstraintPair pair{std::move(residual), std::move(set)};
  attachConstraintPair(*problem_, pair, window, horizon_);
}

void TrajectoryOptimizer::build() {
  if (locked_) {
    return;  // idempotent: already built (setup() run, problem structure frozen).
  }
  // Allocate the solver workspace for the assembled problem and freeze it: no more addCost /
  // addConstraint / resetProblem until resetProblem() unlocks (lifecycle §3.4). Deferred to here
  // (not the ctor) so every cost/constraint added is part of the structure setup() allocates for.
  solver_.setup(*problem_);
  locked_ = true;
}

void TrajectoryOptimizer::resetProblem() {
  // Rebuild the empty shell (default control regularization only). Any outstanding CostHandle now
  // dangles (its residual pointers referenced the discarded problem). build() is required again.
  problem_ = aligator_detail::buildProblemShell(space_, x0_, horizon_, dt_, options_);
  locked_ = false;
}

// --- Solve -----------------------------------------------------------------------------------

tl::expected<TrajOptResult, std::string> TrajectoryOptimizer::solve(const TrajOptSeed& seed) {
  const auto num_stages = static_cast<std::size_t>(horizon_);
  const int nx = rgm_.nq() + rgm_.nv();
  const int nu = rgm_.nv();

  // The problem must be finalized (build()) before it can be solved (maintainer decision, Prompt 9:
  // solve does not auto-build). A missing build() is a recoverable per-call misuse, not a throw.
  if (!locked_) {
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
  solver_.target_tol_ = options_.tol;
  solver_.mu_init_ = options_.mu_init;
  solver_.max_iters = static_cast<std::size_t>(options_.max_iters);
  solver_.verbose_ =
      options_.verbose ? aligator::VerboseLevel::VERBOSE : aligator::VerboseLevel::QUIET;

  bool converged = false;
  try {
    converged = solver_.run(*problem_, seed.xs, seed.us);
  } catch (const std::exception& e) {
    return tl::make_unexpected(std::string("TrajectoryOptimizer::solve: aligator solver threw: ") +
                               e.what());
  }

  const auto& res = solver_.results_;

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
  const int nq = rgm_.nq();
  const int nv = rgm_.nv();
  out.trajectory.times.reserve(res.xs.size());
  out.trajectory.positions.reserve(res.xs.size());
  out.trajectory.velocities.reserve(res.xs.size());
  for (std::size_t k = 0; k < res.xs.size(); ++k) {
    out.trajectory.times.push_back(static_cast<double>(k) * dt_);
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
  const int nq = rgm_.nq();
  const int nv = rgm_.nv();
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
  const pinocchio::Model& model = rgm_.reducedModel();
  const int num_segments = static_cast<int>(waypoints.size()) - 1;

  TrajOptSeed seed;
  seed.xs.reserve(static_cast<std::size_t>(horizon_) + 1);
  for (int k = 0; k <= horizon_; ++k) {
    Eigen::VectorXd q(nq);
    if (num_segments == 0) {
      q = waypoints.front();  // single waypoint: a constant seed at that configuration.
    } else {
      const double t = static_cast<double>(k) / static_cast<double>(horizon_);  // global [0, 1]
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
  seed.us.assign(static_cast<std::size_t>(horizon_), Eigen::VectorXd::Zero(nv));
  return seed;
}

TrajOptSeed TrajectoryOptimizer::shift(const TrajOptResult& result, int n_steps) const {
  const auto num_stages = static_cast<std::size_t>(horizon_);
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
