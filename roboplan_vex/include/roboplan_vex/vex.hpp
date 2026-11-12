#pragma once

#include <cstddef>
#include <limits>
#include <memory>
#include <string>

#include <tl/expected.hpp>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>

namespace roboplan {

/// @brief Termination status of a vex solve.
enum class VexStatus {
  /// @brief Converged with all nonconvex constraints satisfied within the tolerance.
  Converged,
  /// @brief Converged, but nonconvex constraints are still violated after the maximum number of
  /// penalty increases.
  ConvergedInfeasible,
  /// @brief Stopped after `max_iterations` QP iterations.
  MaxIterations,
  /// @brief A convex subproblem could not be solved.
  QpFailure,
  /// @brief Stopped after `max_time` seconds.
  Timeout,
};

/// @brief Options of the vex trajectory optimizer.
/// @details Joint position, velocity and acceleration limits are read from the Scene, not from
/// here. The ℓ1 penalty / trust box / ratio test defaults follow Tesseract `trajopt_sqp`.
struct VexOptions {
  /// @brief The name of the joint group to optimize.
  std::string group_name;

  /// @brief The number of trajectory nodes N (>= 3), uniformly spaced in time.
  size_t num_nodes = 50;

  /// @brief Collision samples per segment, in addition to the nodes (0 = nodes only).
  size_t samples_per_segment = 3;

  /// @brief Weight of the path-energy cost, the exact integral of ||v(t)||^2 over the trajectory.
  double velocity_weight = 1.0;

  /// @brief Weight of the smoothing cost, the exact integral of ||a(t)||^2 over the trajectory.
  double acceleration_weight = 1e-2;

  /// @brief Minimum signed distance d_safe (m) between every active collision pair.
  double collision_margin = 0.01;

  /// @brief Pairs closer than this distance d_check (m) enter the QP; must exceed d_safe.
  double collision_check_distance = 0.05;

  /// @brief Initial ℓ1 penalty weight on nonconvex constraint violations.
  double initial_penalty = 10.0;

  /// @brief Factor by which the penalty grows when converged but still infeasible.
  double penalty_increase = 10.0;

  /// @brief Maximum number of penalty increases.
  size_t max_penalty_increases = 5;

  /// @brief Initial half-width of the trust box, in position units (see Vex::solve).
  double initial_trust_box = 0.1;

  /// @brief Factor applied to the trust box after a rejected step.
  double trust_shrink = 0.1;

  /// @brief Factor applied to the trust box after an accepted step.
  double trust_expand = 1.5;

  /// @brief Minimum ratio of actual to predicted merit improvement to accept a step.
  double improve_ratio = 0.25;

  /// @brief Maximum number of SCP iterations (QP solves after the initial projection).
  size_t max_iterations = 50;

  /// @brief Maximum wall time of the SCP loop, in seconds.
  double max_time = std::numeric_limits<double>::infinity();

  /// @brief Tolerance on constraint violation, predicted merit improvement and trust box size.
  double tolerance = 1e-4;
};

/// @brief Result of a vex solve.
struct VexResult {
  /// @brief The optimized trajectory over the group joints. Not retimed.
  JointTrajectory trajectory;

  /// @brief The termination status.
  VexStatus status = VexStatus::QpFailure;

  /// @brief The number of SCP iterations (QP solves after the initial projection).
  size_t iterations = 0;

  /// @brief The final convex (smoothness) cost.
  double cost = 0.0;

  /// @brief The final maximum violation of the nonconvex constraints.
  double max_violation = 0.0;

  /// @brief Whether a dense collision post-check over all active pairs passed.
  /// @details Samples the trajectory's piecewise-quadratic positions so that no group joint moves
  /// more than 0.01 rad (or m) between consecutive checks. Independent of the status.
  bool continuous_collision_free = false;
};

/// @brief Offline, kinematic-only trajectory optimizer based on sequential convex programming.
/// @details Transcription with a double-integrator model: per node k = 0..N-1 the position q_k and
/// velocity v_k, per segment k = 0..N-2 a constant acceleration a_k, with a fixed step
/// dt = duration / (N - 1) and exact dynamics
///   q_{k+1} = q_k + v_k dt + a_k dt^2 / 2,   v_{k+1} = v_k + a_k dt.
/// Only vector-space joints (nq == nv) are supported. Each solve() uses its own SceneContext and
/// never modifies the Scene. Use one Vex per thread.
class Vex {
public:
  /// @brief Constructor.
  /// @param scene The scene; must not be null.
  /// @param options The optimizer options. Validated on every solve().
  Vex(std::shared_ptr<Scene> scene, VexOptions options);

  /// @brief Optimizes a rest-to-rest trajectory from start to goal.
  /// @details The trajectory satisfies q_0 = start, q_{N-1} = goal, v_0 = v_{N-1} = 0, the
  /// dynamics, and the Scene's joint position / velocity / acceleration limits at the nodes as
  /// hard constraints. Every active collision pair of the Scene keeps a signed distance of at
  /// least `collision_margin` at the nodes and the collision samples (ℓ1-penalized). The seed is
  /// resampled to N nodes and projected onto those constraints before
  /// the SCP loop. Joints outside the group stay at the Scene's configuration.
  /// The trust box bounds the step in position units: |dq| <= r, |dv| dt <= r, |da| dt^2 <= r.
  /// @param start The start configuration of the group joints.
  /// @param goal The goal configuration of the group joints.
  /// @param duration The trajectory duration T > 0 (s).
  /// @param seed The seed path over the group joints (at least 2 waypoints).
  /// @return The result, or an error for invalid input, a start or goal in collision, or an
  /// infeasible problem.
  tl::expected<VexResult, std::string> solve(const JointConfiguration& start,
                                             const JointConfiguration& goal, double duration,
                                             const JointPath& seed);

private:
  std::shared_ptr<Scene> scene_;
  VexOptions options_;
};

}  // namespace roboplan
