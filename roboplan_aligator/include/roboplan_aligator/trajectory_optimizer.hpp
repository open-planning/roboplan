#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <Eigen/Dense>
#include <tl/expected.hpp>

#include <aligator/core/callback-base.hpp>
#include <aligator/core/constraint-set.hpp>
#include <aligator/core/cost-abstract.hpp>
#include <aligator/core/function-abstract.hpp>
#include <aligator/core/history-callback.hpp>
#include <aligator/core/stage-model.hpp>
#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/modelling/spaces/multibody.hpp>
#include <aligator/solvers/proxddp/solver-proxddp.hpp>
#include <aligator/third-party/polymorphic_cxx14.h>

#include <roboplan_aligator/constraint_spec.hpp>
#include <roboplan_aligator/cost_spec.hpp>
#include <roboplan_aligator/problem_builder.hpp>
#include <roboplan_aligator/reduced_group_model.hpp>
#include <roboplan_aligator/types.hpp>

namespace roboplan {

class Scene;

/// @brief Trajectory optimizer wrapping aligator's proximal-DDP solver over a group-reduced,
/// free-space multibody model.
///
/// Usage: construct for a scene and joint group, add costs/constraints, `build()`, then `solve()`
/// from a warm-start seed. Each stage is assembled fully, once, inside `build()` -- mirroring
/// aligator's own `addStage` loop -- so there is no post-build mutation: retargeting a cost means
/// `resetProblem()`, re-adding it with the new value, and
/// `build()` again.
///
/// Three ways to describe what goes on a stage, all consumed once at `build()`:
/// - Spec-based (`addCost`/`addConstraint` + their `*Stage`/`*Terminal` variants, Python + C++):
///   plain structs (`ConfigurationCost`, `VelocityCost`, `TorqueLimit`) translated internally into
///   aligator costs/residuals. The only path Python has (aligator's `xyz::polymorphic` types are
///   not nanobind-bindable).
/// - Direct-aligator (same method names, overloaded on a raw `xyz::polymorphic` aligator cost or
///   (residual, set) pair, C++ only): inserted into a stage roboplan still assembles.
/// - Stage authorship (`setStageFactory`, C++ only): the caller builds the entire `StageModel` for
///   each index themselves, with aligator's own API, no roboplan translation involved. Mutually
///   exclusive with the two tiers above (global/per-stage add* calls) for the lifetime of one
///   built problem, since a caller-authored stage and a roboplan-assembled stage cannot compose.
class TrajectoryOptimizer {
public:
  /// @brief A caller-provided per-index stage builder for the stage-authorship path.
  /// @details Called once per index (0 <= index < horizon()) inside `build()`, mirroring
  /// aligator's own `for (i = 0; i < nsteps; ++i) problem.addStage(makeStage(i))` idiom
  /// (external/aligator/tests/mpc-cycle.cpp). `dynamics` is the optimizer's own discretized
  /// dynamics (built once, shared across stages) so the caller does not need to re-derive it from
  /// `phaseSpace()`/`integrator()`/`dt()` to match the configured default -- though nothing
  /// prevents supplying different dynamics per stage if the factory ignores this argument.
  using StageFactory = std::function<xyz::polymorphic<aligator::StageModelTpl<double>>(
      int index, const aligator_detail::DiscreteDynamics& dynamics)>;

  /// @brief Constructs the optimizer: builds the reduced model for `group_name` and the empty
  /// problem shell (no stages, no user costs/constraints yet).
  /// @param scene The scene to optimize in. Must not be null; retained for the optimizer's
  /// lifetime.
  /// @param group_name The joint group to plan for (fixed-base only).
  /// @param horizon Number of stages N.
  /// @param dt Time step in seconds.
  /// @param options Discretization, regularization, and solver options.
  /// @throws std::invalid_argument if `scene` is null, `horizon <= 0`, `dt <= 0`, or `group_name`
  /// is not a valid fixed-base joint group.
  TrajectoryOptimizer(std::shared_ptr<Scene> scene, std::string group_name, int horizon, double dt,
                      TrajOptOptions options = {});
  ~TrajectoryOptimizer();

  /// @brief Move-constructs from `other`, leaving it in a moved-from state.
  /// @details The reduced model and phase space are rebuilt from the moved `scene`/`group_name`
  /// rather than moved directly, since they hold references into the originating scene.
  TrajectoryOptimizer(TrajectoryOptimizer&&) noexcept;
  TrajectoryOptimizer& operator=(TrajectoryOptimizer&&) noexcept;
  TrajectoryOptimizer(const TrajectoryOptimizer&) = delete;
  TrajectoryOptimizer& operator=(const TrajectoryOptimizer&) = delete;

  /// @brief Number of stages N.
  int horizon() const;
  /// @brief Time step, in seconds.
  double dt() const;
  /// @brief The configured dynamics integrator (see `TrajOptOptions::integrator`).
  IntegratorType integrator() const;
  /// @brief Reduced-model configuration size nq.
  int nq() const;
  /// @brief Reduced-model tangent size nv.
  int nv() const;
  /// @brief State dimension nx = nq + nv.
  int nx() const;

  /// @brief Sets the fixed initial state x0 = [q; 0] for subsequent solves.
  /// @details Updates the in-problem initial-condition constraint target in place; no rebuild.
  /// @param q Reduced-group configuration (size nq).
  /// @throws std::invalid_argument if `q` does not have size nq.
  void setInitialState(const Eigen::VectorXd& q);

  // --- Costs: spec-based (Python + C++) --------------------------------------------------------

  /// @brief Attaches a cost from a concrete spec type to every stage.
  /// @throws std::logic_error if called after `build()`, or if `setStageFactory` was called.
  void addCost(const CostSpec& cost, double weight = 1.0);
  /// @brief Attaches a cost from a concrete spec type to exactly stage `stage`.
  /// @throws std::invalid_argument if `stage` is out of `[0, horizon())`.
  /// @throws std::logic_error if called after `build()`, or if `setStageFactory` was called.
  void addStageCost(int stage, const CostSpec& cost, double weight = 1.0);
  /// @brief Attaches a cost from a concrete spec type to the terminal node.
  /// @throws std::logic_error if called after `build()`.
  void addTerminalCost(const CostSpec& cost, double weight = 1.0);

  // --- Costs: direct aligator (advanced, C++ only) ---------------------------------------------

  /// @brief Attaches a user-supplied aligator cost to every stage, bypassing the spec/factory path.
  /// @details Advanced, C++-only: not exposed to Python bindings.
  void addCost(xyz::polymorphic<aligator::CostAbstractTpl<double>> cost, double weight = 1.0);
  /// @brief Attaches a user-supplied aligator cost to exactly stage `stage`.
  void addStageCost(int stage, xyz::polymorphic<aligator::CostAbstractTpl<double>> cost,
                    double weight = 1.0);
  /// @brief Attaches a user-supplied aligator cost to the terminal node.
  void addTerminalCost(xyz::polymorphic<aligator::CostAbstractTpl<double>> cost,
                       double weight = 1.0);

  // --- Constraints: spec-based (Python + C++) --------------------------------------------------

  /// @brief Attaches a constraint from a concrete spec type to every stage.
  /// @throws std::invalid_argument if the constraint type does not support this target (e.g. a
  /// `TorqueLimit` on the terminal node, which has no control).
  /// @throws std::logic_error if called after `build()`, or if `setStageFactory` was called.
  void addConstraint(const ConstraintSpec& constraint);
  /// @brief Attaches a constraint from a concrete spec type to exactly stage `stage`.
  void addStageConstraint(int stage, const ConstraintSpec& constraint);
  /// @brief Attaches a constraint from a concrete spec type to the terminal node.
  void addTerminalConstraint(const ConstraintSpec& constraint);

  // --- Constraints: direct aligator (advanced, C++ only) ----------------------------------------

  /// @brief Attaches a user-supplied aligator (residual, constraint set) pair to every stage.
  /// @details Advanced, C++-only: not exposed to Python bindings.
  void addConstraint(xyz::polymorphic<aligator::StageFunctionTpl<double>> residual,
                     xyz::polymorphic<aligator::ConstraintSetTpl<double>> set);
  /// @brief Attaches a user-supplied (residual, constraint set) pair to exactly stage `stage`.
  void addStageConstraint(int stage, xyz::polymorphic<aligator::StageFunctionTpl<double>> residual,
                          xyz::polymorphic<aligator::ConstraintSetTpl<double>> set);
  /// @brief Attaches a user-supplied (residual, constraint set) pair to the terminal node.
  void addTerminalConstraint(xyz::polymorphic<aligator::StageFunctionTpl<double>> residual,
                             xyz::polymorphic<aligator::ConstraintSetTpl<double>> set);

  // --- Stage authorship (advanced, C++ only) ----------------------------------------------------

  /// @brief Supplies a per-index stage factory: `build()` calls it once per index instead of
  /// assembling stages from the spec/direct entries above, mirroring aligator's own `addStage`
  /// loop with no roboplan translation in the way.
  /// @details Advanced, C++-only: not exposed to Python bindings. Terminal cost/constraint entries
  /// (`addTerminalCost`/`addTerminalConstraint`, either tier) still apply on top of a stage
  /// factory -- aligator itself keeps the terminal cost as a field separate from `stages_`, so
  /// stage authorship and terminal-node authorship are independent.
  /// @throws std::logic_error if any global/per-stage `addCost`/`addConstraint` entry was already
  /// added (the two mechanisms are mutually exclusive), or if called after `build()`.
  void setStageFactory(StageFactory factory);

  /// @brief Finalizes the problem: allocates the solver workspace and freezes the structure.
  /// @details Idempotent — a second call while already built is a no-op. Required before `solve()`.
  /// No costs/constraints may be added until `resetProblem()` unlocks the problem again.
  void build();

  /// @brief Rebuilds the empty problem shell, re-enabling `addCost`/`addConstraint`.
  /// @details Discards all previously-added cost/constraint entries and any stage factory. A fresh
  /// `build()` is required before the next `solve()`.
  void resetProblem();

  /// @brief Builds a straight-line warm-start seed through reduced-group waypoints.
  /// @details Piecewise-linear, Lie-group-aware interpolation (`pinocchio::interpolate`) onto the
  /// N + 1 horizon grid, evenly parameterized over [0, 1]. Velocities and controls are zero.
  /// @param waypoints Reduced-group configurations to interpolate through (at least one).
  /// @throws std::invalid_argument if `waypoints` is empty or any waypoint has the wrong size.
  /// @return The interpolated seed.
  TrajOptSeed interpolatePath(const std::vector<Eigen::VectorXd>& waypoints) const;

  /// @brief Runs the ProxDDP solver from a warm-start seed.
  /// @param seed Warm-start state/control guesses. Empty `xs`/`us` let aligator default-initialize
  /// them from the problem's initial state.
  /// @return The solve result on success, or an error message if the problem has not been built
  /// (`build()` was not called) or the seed has the wrong shape.
  tl::expected<TrajOptResult, std::string> solve(const TrajOptSeed& seed);

  /// @brief Registers a raw aligator callback directly on the solver (e.g. a custom logger).
  /// @details Advanced, C++-only passthrough to `aligator::SolverProxDDPTpl::registerCallback`; not
  /// exposed to Python. See `TrajOptOptions::record_history` for the Python-visible diagnostics
  /// path.
  /// @param name Identifier the callback is registered under.
  /// @param callback The callback to register.
  void registerCallback(std::string_view name,
                        std::shared_ptr<aligator::CallbackBaseTpl<double>> callback);

  /// @brief The reduced model this optimizer plans over.
  const ReducedGroupModel& reducedGroupModel() const { return rgm_; }
  /// @brief The reduced-model phase space (state manifold x = [q; v]).
  const aligator_detail::PhaseSpace& phaseSpace() const { return space_; }
  /// @brief Mutable access to the assembled aligator problem, for advanced direct manipulation.
  aligator_detail::Problem& problem() { return *problem_; }
  /// @brief The assembled aligator problem.
  const aligator_detail::Problem& problem() const { return *problem_; }

private:
  struct CostEntry {
    CostSpec spec;
    double weight;
  };
  struct DirectCostEntry {
    xyz::polymorphic<aligator::CostAbstractTpl<double>> cost;
    double weight;
  };
  struct ConstraintEntry {
    ConstraintSpec spec;
  };
  struct DirectConstraintEntry {
    xyz::polymorphic<aligator::StageFunctionTpl<double>> func;
    xyz::polymorphic<aligator::ConstraintSetTpl<double>> set;
  };

  std::shared_ptr<Scene> scene_;
  std::string group_name_;
  int horizon_;
  double dt_;
  TrajOptOptions options_;
  ReducedGroupModel rgm_;
  aligator_detail::PhaseSpace space_;
  Eigen::VectorXd x0_;
  std::unique_ptr<aligator_detail::Problem> problem_;
  aligator::SolverProxDDPTpl<double> solver_;
  bool locked_ = false;

  // Plan state: consumed once, inside build(). Cleared by resetProblem().
  std::vector<CostEntry> global_costs_;
  std::vector<DirectCostEntry> global_direct_costs_;
  std::map<int, std::vector<CostEntry>> stage_costs_;
  std::map<int, std::vector<DirectCostEntry>> stage_direct_costs_;
  std::vector<CostEntry> terminal_costs_;
  std::vector<DirectCostEntry> terminal_direct_costs_;
  std::vector<ConstraintEntry> global_constraints_;
  std::vector<DirectConstraintEntry> global_direct_constraints_;
  std::map<int, std::vector<ConstraintEntry>> stage_constraints_;
  std::map<int, std::vector<DirectConstraintEntry>> stage_direct_constraints_;
  std::vector<ConstraintEntry> terminal_constraints_;
  std::vector<DirectConstraintEntry> terminal_direct_constraints_;
  StageFactory stage_factory_;

  bool hasStagePlan() const;
  void requireNoStageFactory() const;
  void requireNoStagePlan() const;
  void requireValidStageIndex(int stage) const;

  // Tier 1 diagnostics: wraps aligator's own HistoryCallbackTpl (bound to `solver_`), registered
  // only when options_.record_history is set. Never moved across TrajectoryOptimizer instances
  // (it holds a raw pointer to `solver_` internally) -- reconstructed fresh after every move.
  std::shared_ptr<aligator::HistoryCallbackTpl<double>> history_callback_;
  void registerHistoryCallbackIfRequested();
};

}  // namespace roboplan
