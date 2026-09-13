#pragma once

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
#include <aligator/core/traj-opt-problem.hpp>
#include <aligator/modelling/spaces/multibody.hpp>
#include <aligator/solvers/proxddp/solver-proxddp.hpp>
#include <aligator/third-party/polymorphic_cxx14.h>

#include <roboplan_aligator/constraint_spec.hpp>
#include <roboplan_aligator/cost_handle.hpp>
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
/// from a warm-start seed. Costs may be retargeted between solves via the `CostHandle` returned by
/// `addCost`; constraints are fixed once added and require `resetProblem()` to change.
class TrajectoryOptimizer {
public:
  /// @brief Constructs the optimizer: builds the reduced model for `group_name` and the empty
  /// problem shell (dynamics + default control regularization, no user costs/constraints yet).
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

  /// @brief Attaches a cost from a concrete spec type (`ConfigurationCost`, `VelocityCost`, ...).
  /// @param cost The cost specification.
  /// @param window The stages the cost applies to. Defaults to every stage.
  /// @param weight Scalar weight multiplying the cost.
  /// @return A handle whose `setTarget` retargets this cost between solves.
  /// @throws std::logic_error if called after `build()` (before a subsequent `resetProblem()`).
  CostHandle addCost(const CostSpec& cost, const StageWindow& window = StageWindow::all(),
                     double weight = 1.0);

  /// @brief Attaches a user-supplied aligator cost directly, bypassing the spec/factory path.
  /// @details Advanced, C++-only: not exposed to Python bindings. Returns a default (no-op)
  /// `CostHandle`, since a custom cost has no roboplan-known target to retarget.
  /// @param cost The aligator cost to attach.
  /// @param window The stages the cost applies to. Defaults to every stage.
  /// @param weight Scalar weight multiplying the cost.
  /// @throws std::logic_error if called after `build()` (before a subsequent `resetProblem()`).
  CostHandle addCost(xyz::polymorphic<aligator::CostAbstractTpl<double>> cost,
                     const StageWindow& window = StageWindow::all(), double weight = 1.0);

  /// @brief Attaches a constraint from a concrete spec type (`TorqueLimit`, ...).
  /// @param constraint The constraint specification.
  /// @param window The stages the constraint applies to. Defaults to every stage.
  /// @throws std::invalid_argument if the constraint type does not support `window` (e.g. a
  /// `TorqueLimit` on the terminal node, which has no control).
  /// @throws std::logic_error if called after `build()` (before a subsequent `resetProblem()`).
  void addConstraint(const ConstraintSpec& constraint,
                     const StageWindow& window = StageWindow::all());

  /// @brief Attaches a user-supplied aligator (residual, constraint set) pair directly, bypassing
  /// the spec/factory path.
  /// @details Advanced, C++-only: not exposed to Python bindings.
  /// @param residual The stage residual function.
  /// @param set The constraint set the residual is checked against.
  /// @param window The stages the constraint applies to. Defaults to every stage.
  /// @throws std::logic_error if called after `build()` (before a subsequent `resetProblem()`).
  void addConstraint(xyz::polymorphic<aligator::StageFunctionTpl<double>> residual,
                     xyz::polymorphic<aligator::ConstraintSetTpl<double>> set,
                     const StageWindow& window = StageWindow::all());

  /// @brief Finalizes the problem: allocates the solver workspace and freezes the structure.
  /// @details Idempotent — a second call while already built is a no-op. Required before `solve()`.
  /// No costs/constraints may be added until `resetProblem()` unlocks the problem again.
  void build();

  /// @brief Rebuilds the empty problem shell, re-enabling `addCost`/`addConstraint`.
  /// @details Any outstanding `CostHandle` dangles afterward (its residual pointers referenced the
  /// discarded problem). A fresh `build()` is required before the next `solve()`.
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

  // Tier 1 diagnostics: wraps aligator's own HistoryCallbackTpl (bound to `solver_`), registered
  // only when options_.record_history is set. Never moved across TrajectoryOptimizer instances
  // (it holds a raw pointer to `solver_` internally) -- reconstructed fresh after every move.
  std::shared_ptr<aligator::HistoryCallbackTpl<double>> history_callback_;
  void registerHistoryCallbackIfRequested();
};

}  // namespace roboplan
