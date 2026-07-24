#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <tl/expected.hpp>

#include <roboplan_aligator/constraints.hpp>
#include <roboplan_aligator/costs.hpp>
#include <roboplan_aligator/types.hpp>

namespace roboplan {

// Forward-declared: the constructor takes the Scene by shared_ptr, whose full definition is only
// needed in the .cpp. aligator/pinocchio types are deliberately absent from this public header
// (aligator is linked PRIVATE; see roboplan_aligator/CLAUDE.md) — they are hidden behind the
// PIMPL below.
class Scene;

/// @brief Trajectory optimizer wrapping aligator's proximal-DDP solver over reduced-model
/// free-space multibody dynamics (design §4.2).
/// @details The constructor snapshots the planning group into a reduced model (§3.1) and assembles
/// an N-stage problem (identical stages carrying the default control regularization, an
/// initial-condition constraint, and an empty terminal-cost placeholder).
///
/// Lifecycle (§3.4): `addCost` (§4.3) and `addConstraint` (§4.4) attach soft costs and hard
/// constraints over stage windows and are legal only before `build()` (or after `resetProblem()`);
/// `build()` allocates the solver workspace and freezes the problem structure; `solve()` then runs
/// SolverProxDDP on a warm-start seed and returns a TrajOptResult, and errors if called before
/// `build()`. Between solves, `setInitialState`, `CostHandle::setTarget`, and the option fields
/// (`max_iters`, `tol`) are hot-path (no rebuild). Warm-start helpers: `interpolatePath` (a
/// straight-line seed through waypoints) and `shift` (receding-horizon advance); `solve` also
/// accepts a previous TrajOptResult directly. Receding-horizon MPC is a per-tick `setInitialState`
/// + `CostHandle::setTarget` + `solve(shift(prev))` loop over the fixed-horizon problem (§3.6).
///
/// All aligator and pinocchio state lives behind a PIMPL, so this public header stays free of
/// third-party types. The class is move-only (it owns a solver + assembled problem).
class TrajectoryOptimizer {
public:
  /// @brief Builds the optimizer for `group_name` on `scene` over an `horizon`-stage grid at `dt`.
  /// @param scene Shared scene the trajectory is optimized against. The reduced model is a snapshot
  ///   taken at construction (§3.1); if the scene model later changes, build a new optimizer.
  /// @param group_name The planning group to optimize. Its joints become the reduced model; every
  ///   other movable joint is physically locked at the scene's current configuration.
  /// @param horizon Number of stages N (must be > 0).
  /// @param dt Time step in seconds (must be > 0).
  /// @param options Solver/discretization options (design §4.1).
  /// @throws std::invalid_argument if `scene` is null, `horizon <= 0`, or `dt <= 0`, and
  ///   (propagated from ReducedGroupModel) if the group is unknown, empty, or floating-base.
  TrajectoryOptimizer(std::shared_ptr<Scene> scene, std::string group_name, int horizon, double dt,
                      TrajOptOptions options = {});

  /// @brief Destructor (defaulted in the .cpp, where the PIMPL is a complete type).
  ~TrajectoryOptimizer();

  TrajectoryOptimizer(TrajectoryOptimizer&&) noexcept;
  TrajectoryOptimizer& operator=(TrajectoryOptimizer&&) noexcept;
  TrajectoryOptimizer(const TrajectoryOptimizer&) = delete;
  TrajectoryOptimizer& operator=(const TrajectoryOptimizer&) = delete;

  /// @brief Number of stages N.
  int horizon() const;

  /// @brief Time step dt, in seconds.
  double dt() const;

  /// @brief Reduced-model configuration size nq (use for positions; never assume nq == nv).
  int nq() const;

  /// @brief Reduced-model tangent size nv (use for velocities / torques / controls).
  int nv() const;

  /// @brief State dimension nx = nq + nv (the size of each `xs` entry).
  int nx() const;

  /// @brief Sets the fixed initial state x0 = [q; v] the trajectory starts from (design §4.2).
  /// @details Hot-path-legal between solves — updates the problem's initial-condition constraint in
  /// place, no rebuild (§3.4).
  /// @param q Reduced-group configuration (size nq()).
  /// @param v Reduced-group velocity (size nv()); if left empty (the default), zero velocity is
  ///   used.
  /// @throws std::invalid_argument if `q` or a non-empty `v` has the wrong size.
  void setInitialState(const Eigen::VectorXd& q, const Eigen::VectorXd& v = Eigen::VectorXd());

  /// @brief Attaches a soft cost to the problem over a window of stages (design §4.3, §3.3).
  /// @details Builds the aligator residual (resolving frames against the internal reduced model)
  /// and adds it to each in-range stage's cost sum — Terminal maps to the terminal cost. Windowing
  /// attaches the term only to in-range stages, never via weight masks (§3.3). Legal only before
  /// the first `solve()` (or after `resetProblem()`); see §3.4.
  /// @param cost The cost specification (see `costs.hpp`).
  /// @param window The stages the cost attaches to (default: all stages).
  /// @param weight Overall scalar weight multiplying the cost (default 1).
  /// @return A CostHandle whose `setTarget` mutates the in-problem residual on the hot path (§3.5).
  /// @throws std::logic_error if called after the first `solve()` (message names `resetProblem()`).
  /// @throws std::invalid_argument if a frame is unknown, a spec field has the wrong size, or the
  ///   window is out of range.
  CostHandle addCost(const FramePoseCost& cost, const StageWindow& window = StageWindow::all(),
                     double weight = 1.0);
  /// @copydoc addCost(const FramePoseCost&, const StageWindow&, double)
  CostHandle addCost(const FrameAxisCost& cost, const StageWindow& window = StageWindow::all(),
                     double weight = 1.0);
  /// @copydoc addCost(const FramePoseCost&, const StageWindow&, double)
  CostHandle addCost(const ConfigurationCost& cost, const StageWindow& window = StageWindow::all(),
                     double weight = 1.0);
  /// @copydoc addCost(const FramePoseCost&, const StageWindow&, double)
  CostHandle addCost(const ControlCost& cost, const StageWindow& window = StageWindow::all(),
                     double weight = 1.0);
  /// @copydoc addCost(const FramePoseCost&, const StageWindow&, double)
  CostHandle addCost(const VelocityCost& cost, const StageWindow& window = StageWindow::all(),
                     double weight = 1.0);

  /// @brief Attaches a hard constraint to the problem over a window of stages (design §4.4, §3.3).
  /// @details Builds the aligator residual + box constraint set (resolving frames and reading limit
  /// defaults from the internal reduced model / core Scene, remapped into reduced layout) and adds
  /// it to each in-range stage — Terminal maps to the terminal constraint. Windowing attaches the
  /// constraint only to in-range stages, never via weight masks (§3.3). Legal only before the first
  /// `solve()` (or after `resetProblem()`); see §3.4. Constraints are fixed at build time (no
  /// mutable-target handle — rebuild via `resetProblem()` to change them).
  /// @param constraint The constraint specification (see `constraints.hpp`).
  /// @param window The stages the constraint attaches to (default: all stages).
  /// @throws std::logic_error if called after the first `solve()` (message names `resetProblem()`).
  /// @throws std::invalid_argument if a frame is unknown, a bound has the wrong size, the window is
  ///   out of range, or a TorqueLimit targets the Terminal window (the terminal node has no
  ///   control).
  void addConstraint(const PositionLimit& constraint,
                     const StageWindow& window = StageWindow::all());
  /// @copydoc addConstraint(const PositionLimit&, const StageWindow&)
  void addConstraint(const VelocityLimit& constraint,
                     const StageWindow& window = StageWindow::all());
  /// @copydoc addConstraint(const PositionLimit&, const StageWindow&)
  void addConstraint(const TorqueLimit& constraint, const StageWindow& window = StageWindow::all());
  /// @copydoc addConstraint(const PositionLimit&, const StageWindow&)
  void addConstraint(const FramePoseConstraint& constraint,
                     const StageWindow& window = StageWindow::all());
  /// @copydoc addConstraint(const PositionLimit&, const StageWindow&)
  /// @details Expands to one collision residual + box (distance ≥ d_min) per tracked pair; the
  /// pairs are the closest self-collision pairs at the initial configuration (§5 reuse path). The
  /// pair set is fixed at this call from the initial state as it is *now*, so set the initial state
  /// (`setInitialState`) before adding collision constraints — a later change re-selects nothing.
  void addConstraint(const SelfCollisionConstraint& constraint,
                     const StageWindow& window = StageWindow::all());
  /// @copydoc addConstraint(const SelfCollisionConstraint&, const StageWindow&)
  void addConstraint(const CollisionConstraint& constraint,
                     const StageWindow& window = StageWindow::all());

  /// @brief Finalizes the problem: allocates the solver workspace and freezes the structure so it
  /// can be solved (design §3.4, §4.2).
  /// @details Must be called after all `addCost`/`addConstraint` calls and before the first
  /// `solve()` (solve does not auto-build; maintainer decision, Prompt 9). After this,
  /// `addCost`/`addConstraint` throw until `resetProblem()`. Idempotent: calling `build()` again
  /// while already built is a no-op.
  void build();

  /// @brief Rebuilds the problem to its empty shell (default control regularization only),
  /// re-enabling `addCost`/`addConstraint` and requiring a fresh `build()` (design §3.4).
  /// @details Discards all user costs and constraints added so far. Any CostHandle returned by a
  /// prior `addCost` dangles after this call and must not be used.
  void resetProblem();

  /// @brief Straight-line warm-start seed through reduced-group waypoints onto the horizon grid
  /// (§3.6).
  /// @details Interpolates the waypoints (Lie-group-aware, on the reduced model) into `horizon()+1`
  /// states with zero velocities, plus `horizon()` zero controls. A single waypoint yields a
  /// constant seed; multiple waypoints form a piecewise-linear path evenly parameterized over the
  /// horizon.
  /// @param waypoints One or more reduced-group configurations (each size nq()).
  /// @throws std::invalid_argument if `waypoints` is empty or a waypoint has the wrong size.
  TrajOptSeed interpolatePath(const std::vector<Eigen::VectorXd>& waypoints) const;

  /// @brief Receding-horizon shift of a solved result into a warm-start seed for the next tick
  /// (§3.6).
  /// @details Drops the first `n_steps` knots and repeats the final state/control to refill the
  /// tail (the "hold" convention; maintainer decision, Prompt 9).
  /// @param result A result previously returned by this optimizer (xs size horizon()+1, us
  /// horizon()).
  /// @param n_steps How many steps to advance the horizon (default 1; must be >= 0).
  /// @throws std::invalid_argument if `n_steps < 0` or `result`'s dimensions do not match the
  /// horizon.
  TrajOptSeed shift(const TrajOptResult& result, int n_steps = 1) const;

  /// @brief Runs the ProxDDP solver from a warm-start seed and returns the result (design §4.2).
  /// @details Requires a prior `build()` (returns an error otherwise). The `TrajOptOptions` passed
  /// at construction (`tol`, `max_iters`, `verbose`, `mu_init`) are re-applied to the solver on
  /// each call, but they are fixed at construction — there is no public accessor to mutate them, so
  /// they cannot be changed between solves.
  /// @param seed Warm-start states/controls in reduced-group layout (§3.6). Empty vectors let
  ///   aligator default-initialize; a non-empty `xs` must have horizon()+1 entries of size nx(),
  ///   and a non-empty `us` must have horizon() entries of size nv().
  /// @return The populated TrajOptResult on success, or an error string on a recoverable failure
  ///   (problem not built, a seed whose dimensions do not match the problem, or a solver
  ///   exception).
  tl::expected<TrajOptResult, std::string> solve(const TrajOptSeed& seed);

  /// @brief Solves warm-started from a previous result (design §3.6, `solve(seed | previous
  /// result)`).
  /// @details Dispatches to `solve(const TrajOptSeed&)` using `previous`'s states/controls as the
  /// seed (they already share the seed layout). Requires a prior `build()`.
  /// @param previous A result previously returned by this optimizer.
  tl::expected<TrajOptResult, std::string> solve(const TrajOptResult& previous);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace roboplan
