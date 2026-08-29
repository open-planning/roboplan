#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

#include <roboplan/core/types.hpp>

namespace roboplan {

// Forward-declared: toRoboplan() borrows the Scene by const reference only.
class Scene;

/// @brief Which integrator discretizes the continuous multibody dynamics.
enum class IntegratorType {
  SemiImplicitEuler,  ///< Semi-implicit Euler (default).
  RK2,                ///< Runge-Kutta 2.
};

/// @brief Options controlling the ProxDDP trajectory optimizer.
/// @details Plain value struct, mirroring `TOPPRAOptions` (`roboplan_toppra`).
struct TrajOptOptions {
  /// @brief Maximum ProxDDP outer iterations.
  int max_iters = 100;

  /// @brief Convergence tolerance.
  double tol = 1e-4;

  /// @brief Augmented-Lagrangian penalty initialization.
  double mu_init = 1e-2;

  /// @brief Dynamics integrator (semi-implicit Euler by default).
  IntegratorType integrator = IntegratorType::SemiImplicitEuler;

  /// @brief Whether the solver prints per-iteration progress.
  bool verbose = false;

  /// @brief Weight of the default quadratic control (torque) regularization cost. 0 disables it.
  double control_reg = 1e-3;
};

/// @brief Warm-start for a solve: state and control guesses on the horizon grid.
/// @details Reduced-group layout. `xs` are stacked states x = [q; v] (each size nq + nv); `us` are
/// controls/torques (each size nv). Produced by `TrajectoryOptimizer::interpolatePath` / `::shift`,
/// or hand-built. For an N-step horizon, `xs` has N + 1 entries and `us` has N.
struct TrajOptSeed {
  /// @brief Per-knot state guesses x = [q; v], reduced-group layout (size N + 1).
  std::vector<Eigen::VectorXd> xs;

  /// @brief Per-stage control (torque) guesses, reduced-group layout (size N).
  std::vector<Eigen::VectorXd> us;
};

/// @brief The optimized state trajectory sampled at `dt`.
/// @details Reduced-group layout. Torque is exposed via `TrajOptResult::controls`; acceleration is
/// not computed.
struct TrajOptTrajectory {
  /// @brief Sample times, k * dt for k = 0..N (size N + 1).
  std::vector<double> times;

  /// @brief Reduced-group joint positions q at each time (size N + 1, each of size nq).
  std::vector<Eigen::VectorXd> positions;

  /// @brief Reduced-group joint velocities v at each time (size N + 1, each of size nv).
  std::vector<Eigen::VectorXd> velocities;
};

/// @brief Result of a trajectory optimization solve.
/// @details Plain value struct. `xs`/`us` are the raw solver arrays; `trajectory`/`controls` are
/// their semantic views. `controls` is the N x nv torque profile (equal to `us` for a fully-
/// actuated group with actuation B = I).
struct TrajOptResult {
  /// @brief Whether the solver reached its convergence tolerance.
  bool converged = false;

  /// @brief Number of ProxDDP outer iterations taken.
  int iterations = 0;

  /// @brief Final total cost.
  double cost = 0.0;

  /// @brief Largest constraint violation at the returned solution (0 if unconstrained/feasible).
  double max_constraint_violation = 0.0;

  /// @brief Raw solver state trajectory, reduced-group layout (size N + 1, each nq + nv).
  std::vector<Eigen::VectorXd> xs;

  /// @brief Raw solver control trajectory, reduced-group layout (size N, each nv).
  std::vector<Eigen::VectorXd> us;

  /// @brief Joint-torque profile, reduced-group layout (size N, each nv). Equals `us` for B = I.
  std::vector<Eigen::VectorXd> controls;

  /// @brief Optimized state trajectory sampled at `dt`, reduced-group layout.
  TrajOptTrajectory trajectory;

  /// @brief Converts the optimized trajectory to a full-model roboplan::JointTrajectory.
  /// @details Maps reduced-group positions/velocities to full-model layout (non-group DoF zero).
  /// Accelerations are left empty and torques are dropped — use `controls` for those.
  /// @param scene The scene the trajectory was optimized against.
  /// @param group_name The planning group whose reduced positions/velocities are being expanded.
  /// @return A full-model JointTrajectory (positions + velocities + times; accelerations empty).
  JointTrajectory toRoboplan(const Scene& scene, const std::string& group_name) const;
};

/// @brief A half-open range of stages a cost/constraint attaches to.
/// @details Half-open convention: `Range(a, b)` covers stages a, ..., b-1 (b excluded), with
/// 0 <= a < b <= N, where N is the horizon (number of stages). `All` covers every stage [0, N);
/// `Terminal` attaches to the terminal node only (mapped to the terminal cost / terminal
/// constraint).
class StageWindow {
public:
  /// @brief The kind of window.
  enum class Kind {
    All,       ///< Every stage [0, N).
    Range,     ///< A half-open subrange [begin, end).
    Terminal,  ///< The terminal node only.
  };

  /// @brief All stages [0, N).
  static StageWindow all();

  /// @brief The half-open stage range [begin, end).
  /// @throws std::invalid_argument if begin < 0 or end <= begin (an empty or negative range).
  ///         The upper bound end <= N is checked later by resolveStages(N), when N is known.
  static StageWindow range(int begin, int end);

  /// @brief The terminal node only.
  static StageWindow terminal();

  /// @brief The kind of window.
  Kind kind() const { return kind_; }

  /// @brief Whether this window targets the terminal node.
  bool isTerminal() const { return kind_ == Kind::Terminal; }

  /// @brief The concrete stage indices this window covers for a horizon of `horizon` stages.
  /// @details All -> {0, ..., horizon - 1}; Range -> {begin, ..., end - 1}; Terminal -> {} (it
  /// attaches to the terminal node, not to a stage; use `isTerminal()` to detect it).
  /// @param horizon The number of stages N (must be strictly positive).
  /// @throws std::invalid_argument if horizon <= 0, or if this is a Range with end > horizon.
  /// @return The in-range stage indices, ascending.
  std::vector<int> resolveStages(int horizon) const;

private:
  StageWindow(Kind kind, int begin, int end) : kind_(kind), begin_(begin), end_(end) {}

  Kind kind_;
  int begin_ = 0;  ///< Meaningful only for Kind::Range.
  int end_ = 0;    ///< Meaningful only for Kind::Range (half-open upper bound).
};

}  // namespace roboplan
