#include <roboplan_aligator/trajectory_optimizer.hpp>

#include <cstddef>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <algorithm>  // std::min
#include <variant>

#include <aligator/fwd.hpp>                              // VerboseLevel
#include <aligator/modelling/costs/quad-state-cost.hpp>  // QuadraticControlCostTpl
#include <aligator/modelling/costs/sum-of-costs.hpp>     // CostStackTpl
#include <aligator/solvers/proxddp/solver-proxddp.hpp>   // SolverProxDDPTpl
#include <pinocchio/algorithm/joint-configuration.hpp>   // pinocchio::interpolate

#include <roboplan/core/scene.hpp>

#include <roboplan_aligator/constraint_factory.hpp>
#include <roboplan_aligator/cost_factory.hpp>
#include <roboplan_aligator/problem_builder.hpp>
#include <roboplan_aligator/reduced_group_model.hpp>

namespace roboplan {

namespace {

using aligator_detail::CostStack;
using StageModel = aligator::StageModelTpl<double>;
using ManifoldPoly = xyz::polymorphic<aligator::ManifoldAbstractTpl<double>>;
using CostPoly = xyz::polymorphic<aligator::CostAbstractTpl<double>>;

// Stack a reduced [q; v] into a single state vector x (size nq + nv), matching aligator's
// MultibodyPhaseSpace layout (q first, then v).
Eigen::VectorXd stackState(const Eigen::VectorXd& q, const Eigen::VectorXd& v) {
  Eigen::VectorXd x(q.size() + v.size());
  x << q, v;
  return x;
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

// Dispatches a CostSpec to the matching factory: builds the concrete aligator cost and
// inserts it into `stack` once, at stage-assembly time.
void applyCostSpec(CostStack& stack, const aligator_detail::PhaseSpace& space,
                   const ReducedGroupModel& rgm, const CostSpec& spec, double weight) {
  std::visit(
      [&](const auto& s) {
        using T = std::decay_t<decltype(s)>;
        if constexpr (std::is_same_v<T, ConfigurationCost>) {
          aligator_detail::attachConfigurationCost(stack, space, rgm, s, weight);
        } else if constexpr (std::is_same_v<T, VelocityCost>) {
          aligator_detail::attachVelocityCost(stack, space, rgm, s, weight);
        }
      },
      spec);
}

// Dispatches a ConstraintSpec to the matching factory, building the (residual, set) pair.
// `is_terminal` lets the factory reject targets it does not support (e.g. TorqueLimit at the
// terminal node, which has no control) at the call site, not deferred to build().
aligator_detail::ConstraintPair buildConstraintPair(const aligator_detail::PhaseSpace& space,
                                                    const ReducedGroupModel& rgm,
                                                    const ConstraintSpec& spec, bool is_terminal) {
  return std::visit(
      [&](const auto& s) -> aligator_detail::ConstraintPair {
        using T = std::decay_t<decltype(s)>;
        if constexpr (std::is_same_v<T, TorqueLimit>) {
          if (is_terminal) {
            throw std::invalid_argument(
                "TrajectoryOptimizer::addTerminalConstraint: a TorqueLimit cannot target the "
                "terminal node; the terminal node has no control.");
          }
          return aligator_detail::buildTorqueLimit(space, rgm, s);
        }
        // Unreachable: ConstraintSpec is a closed variant, all alternatives handled above.
        throw std::logic_error("buildConstraintPair: unhandled ConstraintSpec alternative.");
      },
      spec);
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
      problem_(aligator_detail::buildEmptyProblem(space_, x0_, rgm_.nv())) {
  registerHistoryCallbackIfRequested();
}

TrajectoryOptimizer::~TrajectoryOptimizer() = default;

TrajectoryOptimizer::TrajectoryOptimizer(TrajectoryOptimizer&& other) noexcept
    : scene_(std::move(other.scene_)), group_name_(std::move(other.group_name_)),
      horizon_(other.horizon_), dt_(other.dt_), options_(other.options_),
      rgm_(*scene_, group_name_), space_(aligator_detail::makePhaseSpace(rgm_.reducedModel())),
      x0_(std::move(other.x0_)), problem_(std::move(other.problem_)),
      solver_(std::move(other.solver_)), locked_(other.locked_),
      global_costs_(std::move(other.global_costs_)),
      global_direct_costs_(std::move(other.global_direct_costs_)),
      stage_costs_(std::move(other.stage_costs_)),
      stage_direct_costs_(std::move(other.stage_direct_costs_)),
      terminal_costs_(std::move(other.terminal_costs_)),
      terminal_direct_costs_(std::move(other.terminal_direct_costs_)),
      global_constraints_(std::move(other.global_constraints_)),
      global_direct_constraints_(std::move(other.global_direct_constraints_)),
      stage_constraints_(std::move(other.stage_constraints_)),
      stage_direct_constraints_(std::move(other.stage_direct_constraints_)),
      terminal_constraints_(std::move(other.terminal_constraints_)),
      terminal_direct_constraints_(std::move(other.terminal_direct_constraints_)),
      stage_factory_(std::move(other.stage_factory_)) {
  // history_callback_ is intentionally NOT moved from `other`: HistoryCallbackTpl stores a raw
  // pointer to the solver it was constructed against, which would otherwise reference the
  // moved-from `other.solver_`. Reconstruct fresh, bound to this->solver_.
  registerHistoryCallbackIfRequested();
}

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

// --- Diagnostics (Tier 1: history; Tier 2: raw callback registration) -----------------------

void TrajectoryOptimizer::registerHistoryCallbackIfRequested() {
  if (!options_.record_history) {
    return;
  }
  // store_pd_vars=false: we only read values_/prim_infeas/dual_infeas, not the (expensive) xs/us
  // history. store_values_=true: populates `values` (cost) in lockstep with prim_infeas/dual_infeas
  // on every invokeCallbacks() call (aligator core/history-callback.hxx:15-20), so the three
  // vectors are always the same length.
  history_callback_ = std::make_shared<aligator::HistoryCallbackTpl<double>>(
      &solver_, /*store_pd_vars=*/false, /*store_values=*/true);
  solver_.registerCallback("roboplan_history", history_callback_);
}

void TrajectoryOptimizer::registerCallback(
    std::string_view name, std::shared_ptr<aligator::CallbackBaseTpl<double>> callback) {
  solver_.registerCallback(name, std::move(callback));
}

// --- Introspection ---------------------------------------------------------------------------

int TrajectoryOptimizer::horizon() const { return horizon_; }
double TrajectoryOptimizer::dt() const { return dt_; }
IntegratorType TrajectoryOptimizer::integrator() const { return options_.integrator; }
int TrajectoryOptimizer::nq() const { return rgm_.nq(); }
int TrajectoryOptimizer::nv() const { return rgm_.nv(); }
int TrajectoryOptimizer::nx() const { return rgm_.nq() + rgm_.nv(); }

// --- Initial state ---------------------------------------------------------------------------

void TrajectoryOptimizer::setInitialState(const Eigen::VectorXd& q) {
  const int nq = rgm_.nq();
  const int nv = rgm_.nv();
  if (q.size() != nq) {
    throw std::invalid_argument("TrajectoryOptimizer::setInitialState: q has size " +
                                std::to_string(q.size()) +
                                ", expected reduced nq = " + std::to_string(nq) + ".");
  }
  x0_ = stackState(q, Eigen::VectorXd::Zero(nv));
  // Updates the initial-condition constraint target in place (no rebuild) -- a genuinely
  // aligator-native mutation (Problem::setInitState, traj-opt-problem.hpp:175-181), used the same
  // way in aligator's own MPC loop (external/aligator/tests/mpc-cycle.cpp:197).
  problem_->setInitState(x0_);
}

// --- Lifecycle guards --------------------------------------------------------------------------

bool TrajectoryOptimizer::hasStagePlan() const {
  return !global_costs_.empty() || !global_direct_costs_.empty() || !stage_costs_.empty() ||
         !stage_direct_costs_.empty() || !global_constraints_.empty() ||
         !global_direct_constraints_.empty() || !stage_constraints_.empty() ||
         !stage_direct_constraints_.empty();
}

void TrajectoryOptimizer::requireNoStageFactory() const {
  if (stage_factory_) {
    throw std::logic_error(
        "TrajectoryOptimizer: a stage factory was set via setStageFactory(); global/per-stage "
        "addCost/addConstraint calls are mutually exclusive with it on the same instance.");
  }
}

void TrajectoryOptimizer::requireNoStagePlan() const {
  if (hasStagePlan()) {
    throw std::logic_error(
        "TrajectoryOptimizer::setStageFactory: global/per-stage addCost/addConstraint entries "
        "were already added; the two mechanisms are mutually exclusive on the same instance.");
  }
}

void TrajectoryOptimizer::requireValidStageIndex(int stage) const {
  if (stage < 0 || stage >= horizon_) {
    throw std::invalid_argument("TrajectoryOptimizer: stage index " + std::to_string(stage) +
                                " is out of range [0, " + std::to_string(horizon_) + ").");
  }
}

// --- Costs: spec-based -------------------------------------------------------------------------

void TrajectoryOptimizer::addCost(const CostSpec& cost, double weight) {
  requireUnlocked(locked_);
  requireNoStageFactory();
  global_costs_.push_back({cost, weight});
}

void TrajectoryOptimizer::addStageCost(int stage, const CostSpec& cost, double weight) {
  requireUnlocked(locked_);
  requireNoStageFactory();
  requireValidStageIndex(stage);
  stage_costs_[stage].push_back({cost, weight});
}

void TrajectoryOptimizer::addTerminalCost(const CostSpec& cost, double weight) {
  requireUnlocked(locked_);
  terminal_costs_.push_back({cost, weight});
}

// --- Costs: direct aligator ----------------------------------------------------------------

void TrajectoryOptimizer::addCost(xyz::polymorphic<aligator::CostAbstractTpl<double>> cost,
                                  double weight) {
  requireUnlocked(locked_);
  requireNoStageFactory();
  global_direct_costs_.push_back({std::move(cost), weight});
}

void TrajectoryOptimizer::addStageCost(int stage,
                                       xyz::polymorphic<aligator::CostAbstractTpl<double>> cost,
                                       double weight) {
  requireUnlocked(locked_);
  requireNoStageFactory();
  requireValidStageIndex(stage);
  stage_direct_costs_[stage].push_back({std::move(cost), weight});
}

void TrajectoryOptimizer::addTerminalCost(xyz::polymorphic<aligator::CostAbstractTpl<double>> cost,
                                          double weight) {
  requireUnlocked(locked_);
  terminal_direct_costs_.push_back({std::move(cost), weight});
}

// --- Constraints: spec-based -------------------------------------------------------------------

void TrajectoryOptimizer::addConstraint(const ConstraintSpec& constraint) {
  requireUnlocked(locked_);
  requireNoStageFactory();
  // Validate immediately (e.g. a wrong-size bound) rather than deferring to build().
  (void)buildConstraintPair(space_, rgm_, constraint, /*is_terminal=*/false);
  global_constraints_.push_back({constraint});
}

void TrajectoryOptimizer::addStageConstraint(int stage, const ConstraintSpec& constraint) {
  requireUnlocked(locked_);
  requireNoStageFactory();
  requireValidStageIndex(stage);
  (void)buildConstraintPair(space_, rgm_, constraint, /*is_terminal=*/false);
  stage_constraints_[stage].push_back({constraint});
}

void TrajectoryOptimizer::addTerminalConstraint(const ConstraintSpec& constraint) {
  requireUnlocked(locked_);
  // Validate immediately (e.g. TorqueLimit at the terminal node) rather than deferring to build().
  (void)buildConstraintPair(space_, rgm_, constraint, /*is_terminal=*/true);
  terminal_constraints_.push_back({constraint});
}

// --- Constraints: direct aligator --------------------------------------------------------------

void TrajectoryOptimizer::addConstraint(
    xyz::polymorphic<aligator::StageFunctionTpl<double>> residual,
    xyz::polymorphic<aligator::ConstraintSetTpl<double>> set) {
  requireUnlocked(locked_);
  requireNoStageFactory();
  global_direct_constraints_.push_back({std::move(residual), std::move(set)});
}

void TrajectoryOptimizer::addStageConstraint(
    int stage, xyz::polymorphic<aligator::StageFunctionTpl<double>> residual,
    xyz::polymorphic<aligator::ConstraintSetTpl<double>> set) {
  requireUnlocked(locked_);
  requireNoStageFactory();
  requireValidStageIndex(stage);
  stage_direct_constraints_[stage].push_back({std::move(residual), std::move(set)});
}

void TrajectoryOptimizer::addTerminalConstraint(
    xyz::polymorphic<aligator::StageFunctionTpl<double>> residual,
    xyz::polymorphic<aligator::ConstraintSetTpl<double>> set) {
  requireUnlocked(locked_);
  terminal_direct_constraints_.push_back({std::move(residual), std::move(set)});
}

// --- Stage authorship ----------------------------------------------------------------------

void TrajectoryOptimizer::setStageFactory(StageFactory factory) {
  requireUnlocked(locked_);
  requireNoStagePlan();
  stage_factory_ = std::move(factory);
}

// --- Build / reset -------------------------------------------------------------------------

void TrajectoryOptimizer::build() {
  if (locked_) {
    return;  // idempotent: already built (setup() run, problem structure frozen).
  }
  // linear_solver_choice/rollout_type_ are read by setup() itself to construct the solver's
  // internal linear-solver object (and to check their compatibility -- Parallel + Nonlinear
  // throws), so they must be set before setup(), not in solve() like tol/mu_init/max_iters/verbose.
  // setNumThreads() must likewise precede setup() per aligator's own documented warning.
  solver_.setNumThreads(static_cast<std::size_t>(options_.num_threads));
  solver_.linear_solver_choice = options_.linear_solver_choice;
  solver_.rollout_type_ = options_.rollout_type;

  const int nu = rgm_.nv();
  const ManifoldPoly space_poly(space_);
  aligator_detail::DiscreteDynamics dynamics =
      aligator_detail::makeDiscreteDynamics(space_, options_.integrator, dt_);

  if (stage_factory_) {
    for (int k = 0; k < horizon_; ++k) {
      problem_->addStage(stage_factory_(k, dynamics));
    }
  } else {
    const bool add_control_reg = options_.control_reg > 0.0;
    const Eigen::MatrixXd control_weights =
        add_control_reg ? Eigen::MatrixXd(options_.control_reg * Eigen::MatrixXd::Identity(nu, nu))
                        : Eigen::MatrixXd();

    for (int k = 0; k < horizon_; ++k) {
      CostStack stack(space_poly, nu);
      if (add_control_reg) {
        stack.addCost(
            CostPoly(aligator::QuadraticControlCostTpl<double>(space_poly, nu, control_weights)),
            1.0);
      }
      for (const auto& e : global_costs_) {
        applyCostSpec(stack, space_, rgm_, e.spec, e.weight);
      }
      for (const auto& e : global_direct_costs_) {
        stack.addCost(e.cost, e.weight);
      }
      if (auto it = stage_costs_.find(k); it != stage_costs_.end()) {
        for (const auto& e : it->second) {
          applyCostSpec(stack, space_, rgm_, e.spec, e.weight);
        }
      }
      if (auto it = stage_direct_costs_.find(k); it != stage_direct_costs_.end()) {
        for (const auto& e : it->second) {
          stack.addCost(e.cost, e.weight);
        }
      }

      StageModel stage(CostPoly(stack), dynamics);

      for (const auto& e : global_constraints_) {
        const auto pair = buildConstraintPair(space_, rgm_, e.spec, /*is_terminal=*/false);
        stage.addConstraint(pair.func, pair.set);
      }
      for (const auto& e : global_direct_constraints_) {
        stage.addConstraint(e.func, e.set);
      }
      if (auto it = stage_constraints_.find(k); it != stage_constraints_.end()) {
        for (const auto& e : it->second) {
          const auto pair = buildConstraintPair(space_, rgm_, e.spec, /*is_terminal=*/false);
          stage.addConstraint(pair.func, pair.set);
        }
      }
      if (auto it = stage_direct_constraints_.find(k); it != stage_direct_constraints_.end()) {
        for (const auto& e : it->second) {
          stage.addConstraint(e.func, e.set);
        }
      }

      problem_->addStage(stage);
    }
  }

  // Terminal cost/constraint: always from the spec/direct entries (independent of stage_factory_
  // -- aligator itself keeps term_cost_/term_cstrs_ as fields separate from stages_). A plain
  // wholesale field assignment, matching aligator's own idiom for updating the terminal cost
  // (external/aligator/tests/mpc-cycle.cpp: `problem.term_cost_ = makeCost(...)`).
  CostStack term_stack(space_poly, nu);
  for (const auto& e : terminal_costs_) {
    applyCostSpec(term_stack, space_, rgm_, e.spec, e.weight);
  }
  for (const auto& e : terminal_direct_costs_) {
    term_stack.addCost(e.cost, e.weight);
  }
  problem_->term_cost_ = CostPoly(term_stack);

  problem_->removeTerminalConstraints();
  for (const auto& e : terminal_constraints_) {
    const auto pair = buildConstraintPair(space_, rgm_, e.spec, /*is_terminal=*/true);
    problem_->addTerminalConstraint(pair.func, pair.set);
  }
  for (const auto& e : terminal_direct_constraints_) {
    problem_->addTerminalConstraint(e.func, e.set);
  }

  // Allocate the solver workspace for the assembled problem and freeze it: no more addCost /
  // addConstraint / setStageFactory / resetProblem until resetProblem() unlocks.
  solver_.setup(*problem_);
  locked_ = true;
}

void TrajectoryOptimizer::resetProblem() {
  // Rebuild the empty shell (no stages, no terminal cost/constraint content) and discard the plan.
  problem_ = aligator_detail::buildEmptyProblem(space_, x0_, rgm_.nv());
  global_costs_.clear();
  global_direct_costs_.clear();
  stage_costs_.clear();
  stage_direct_costs_.clear();
  terminal_costs_.clear();
  terminal_direct_costs_.clear();
  global_constraints_.clear();
  global_direct_constraints_.clear();
  stage_constraints_.clear();
  stage_direct_constraints_.clear();
  terminal_constraints_.clear();
  terminal_direct_constraints_.clear();
  stage_factory_ = nullptr;
  locked_ = false;
}

// --- Solve -----------------------------------------------------------------------------------

tl::expected<TrajOptResult, std::string> TrajectoryOptimizer::solve(const TrajOptSeed& seed) {
  const auto num_stages = static_cast<std::size_t>(horizon_);
  const int nx = rgm_.nq() + rgm_.nv();
  const int nu = rgm_.nv();

  // The problem must be finalized (build()) before it can be solved -- solve() does not auto-build.
  // A missing build() is a recoverable per-call misuse, not a throw.
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

  // Apply options by direct public-field assignment before the run: max_iters/tol/verbose are
  // editable between solves without a rebuild. mu_init_ is consumed by run()
  // (setAlmPenalty(mu_init_), solver-proxddp.hxx:460), not by setup(), so assigning it here means
  // every solve honours the current options.mu_init.
  solver_.target_tol_ = options_.tol;
  solver_.mu_init_ = options_.mu_init;
  solver_.max_iters = static_cast<std::size_t>(options_.max_iters);
  solver_.verbose_ =
      options_.verbose ? aligator::VerboseLevel::VERBOSE : aligator::VerboseLevel::QUIET;

  // Clear any history from a previous solve() on this same built problem: TrajOptResult::history
  // reflects only the solve about to run, not an accumulation across repeated solves.
  if (history_callback_) {
    history_callback_->values.clear();
    history_callback_->prim_infeas.clear();
    history_callback_->dual_infeas.clear();
  }

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
  out.controls = res.us;  // actuation B = I, so the applied torque equals the control.

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

  if (history_callback_) {
    const auto& cb = *history_callback_;
    out.history.reserve(cb.values.size());
    for (std::size_t k = 0; k < cb.values.size(); ++k) {
      out.history.push_back(
          TrajOptIterate{static_cast<int>(k), cb.values[k], cb.prim_infeas[k], cb.dual_infeas[k]});
    }
  }

  return out;
}

// --- Warm-start --------------------------------------------------------------------------------

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
  // reduced sub-model). Multiple waypoints form a piecewise-linear path evenly parameterized over
  // [0, 1]; velocities and controls are zero.
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

}  // namespace roboplan
