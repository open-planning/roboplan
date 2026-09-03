#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <tl/expected.hpp>

#include <roboplan_aligator/constraint_spec.hpp>
#include <roboplan_aligator/constraints.hpp>
#include <roboplan_aligator/cost_spec.hpp>
#include <roboplan_aligator/costs.hpp>
#include <roboplan_aligator/types.hpp>

namespace roboplan {

// Forward-declared: the full Scene definition is only needed in the .cpp.
class Scene;

/// @brief Trajectory optimizer wrapping aligator's proximal-DDP solver over reduced-model
/// free-space multibody dynamics.
/// @details The constructor snapshots the planning group into a reduced model and assembles an
/// N-stage problem.
///
/// Lifecycle: `addCost` / `addConstraint` attach soft costs and hard constraints over stage windows
/// and are legal only before `build()` (or after `resetProblem()`); `build()` freezes the problem
/// structure; `solve()` runs the solver on a warm-start seed and returns a `TrajOptResult`. Between
/// solves, `setInitialState` and `CostHandle::setTarget` update the problem in place (no rebuild).
/// Warm-start helpers: `interpolatePath` (a straight-line seed through waypoints) and `shift`
/// (receding-horizon advance); `solve` also accepts a previous `TrajOptResult` directly. Receding-
/// horizon MPC is a per-tick `setInitialState` + `CostHandle::setTarget` + `solve(shift(prev))`
/// loop over the fixed-horizon problem. The class is move-only (it owns a solver and the assembled
/// problem).
class TrajectoryOptimizer {
public:
  /// @brief Builds the optimizer for `group_name` on `scene` over an `horizon`-stage grid at `dt`.
  /// @param scene Shared scene the trajectory is optimized against. The reduced model is a snapshot
  ///   taken at construction; if the scene model later changes, build a new optimizer.
  /// @param group_name The planning group to optimize. Its joints become the reduced model; every
  ///   other movable joint is locked at the scene's current configuration.
  /// @param horizon Number of stages N (must be > 0).
  /// @param dt Time step in seconds (must be > 0).
  /// @param options Solver/discretization options.
  /// @throws std::invalid_argument if `scene` is null, `horizon <= 0`, or `dt <= 0`, or if the
  ///   group is unknown, empty, or floating-base.
  TrajectoryOptimizer(std::shared_ptr<Scene> scene, std::string group_name, int horizon, double dt,
                      TrajOptOptions options = {});

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

  /// @brief Sets the fixed initial state x0 = [q; v] the trajectory starts from.
  /// @details Hot-path-legal between solves — updates the initial-condition constraint in place, no
  /// rebuild.
  /// @param q Reduced-group configuration (size nq()).
  /// @param v Reduced-group velocity (size nv()); empty (the default) means zero velocity.
  /// @throws std::invalid_argument if `q` or a non-empty `v` has the wrong size.
  void setInitialState(const Eigen::VectorXd& q, const Eigen::VectorXd& v = Eigen::VectorXd());

  /// @brief Attaches a soft cost to the problem over a window of stages.
  /// @details Adds the cost to each stage in `window`; a Terminal window maps to the terminal cost.
  /// Legal only before the first `solve()` (or after `resetProblem()`). Accepts any concrete cost
  /// type via CostSpec (FramePoseCost, FrameAxisCost, ConfigurationCost, ControlCost,
  /// VelocityCost).
  /// @param cost The cost specification (any concrete cost type; see `costs.hpp`).
  /// @param window The stages the cost attaches to (default: all stages).
  /// @param weight Overall scalar weight multiplying the cost (default 1).
  /// @return A `CostHandle` whose `setTarget` updates the target between solves.
  /// @throws std::logic_error if called after the first `solve()` (message names `resetProblem()`).
  /// @throws std::invalid_argument if a frame is unknown, a spec field has the wrong size, or the
  ///   window is out of range.
  CostHandle addCost(const CostSpec& cost, const StageWindow& window = StageWindow::all(),
                     double weight = 1.0);

  /// @brief Attaches a hard constraint to the problem over a window of stages.
  /// @details Adds the constraint to each stage in `window`; a Terminal window maps to the terminal
  /// constraint. Legal only before the first `solve()` (or after `resetProblem()`). Constraints are
  /// fixed at build time — rebuild via `resetProblem()` to change them. Accepts any concrete
  /// constraint type via ConstraintSpec (PositionLimit, VelocityLimit, TorqueLimit,
  /// FramePoseConstraint, SelfCollisionConstraint, CollisionConstraint).
  /// @param constraint The constraint specification (any concrete constraint type;
  ///   see `constraints.hpp`).
  /// @param window The stages the constraint attaches to (default: all stages).
  /// @throws std::logic_error if called after the first `solve()` (message names `resetProblem()`).
  /// @throws std::invalid_argument if a frame is unknown, a bound has the wrong size, the window is
  ///   out of range, or a TorqueLimit targets the Terminal window (the terminal node has no
  ///   control).
  void addConstraint(const ConstraintSpec& constraint,
                     const StageWindow& window = StageWindow::all());

  /// @brief Finalizes the problem: allocates the solver workspace and freezes the structure so it
  /// can be solved.
  /// @details Must be called after all `addCost`/`addConstraint` calls and before the first
  /// `solve()` (solve does not auto-build). After this, `addCost`/`addConstraint` throw until
  /// `resetProblem()`. Idempotent: calling `build()` again while already built is a no-op.
  void build();

  /// @brief Discards all user costs and constraints and re-enables `addCost`/`addConstraint`;
  /// requires a fresh `build()`.
  /// @details Any `CostHandle` returned by a prior `addCost` dangles after this call and must not
  /// be used.
  void resetProblem();

  /// @brief Straight-line warm-start seed through reduced-group waypoints onto the horizon grid.
  /// @details Interpolates the waypoints into `horizon()+1` states with zero velocities, plus
  /// `horizon()` zero controls. A single waypoint yields a constant seed; multiple waypoints form
  /// a piecewise-linear path evenly parameterized over the horizon.
  /// @param waypoints One or more reduced-group configurations (each size nq()).
  /// @throws std::invalid_argument if `waypoints` is empty or a waypoint has the wrong size.
  TrajOptSeed interpolatePath(const std::vector<Eigen::VectorXd>& waypoints) const;

  /// @brief Receding-horizon shift of a solved result into a warm-start seed for the next tick.
  /// @details Drops the first `n_steps` knots and repeats the final state/control to refill the
  /// tail.
  /// @param result A result previously returned by this optimizer (xs size horizon()+1, us
  /// horizon()).
  /// @param n_steps How many steps to advance the horizon (default 1; must be >= 0).
  /// @throws std::invalid_argument if `n_steps < 0` or `result`'s dimensions do not match the
  /// horizon.
  TrajOptSeed shift(const TrajOptResult& result, int n_steps = 1) const;

  /// @brief Runs the ProxDDP solver from a warm-start seed and returns the result.
  /// @details Requires a prior `build()` (returns an error otherwise). The options passed at
  /// construction (`tol`, `max_iters`, `verbose`, `mu_init`) are fixed — there is no accessor to
  /// change them between solves.
  /// @param seed Warm-start states/controls in reduced-group layout. Empty vectors let aligator
  ///   default-initialize; a non-empty `xs` must have horizon()+1 entries of size nx(), and a
  ///   non-empty `us` must have horizon() entries of size nv().
  /// @return The populated TrajOptResult on success, or an error string on a recoverable failure
  ///   (problem not built, a seed whose dimensions do not match the problem, or a solver
  ///   exception).
  tl::expected<TrajOptResult, std::string> solve(const TrajOptSeed& seed);

  /// @brief Solves warm-started from a previous result.
  /// @details Uses `previous`'s states/controls as the seed. Requires a prior `build()`.
  /// @param previous A result previously returned by this optimizer.
  tl::expected<TrajOptResult, std::string> solve(const TrajOptResult& previous);

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace roboplan
