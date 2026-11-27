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
  /// @brief Converged, collision violation within the tolerance.
  Converged,
  /// @brief Converged, but collision violations remain after `max_penalty_increases` increases.
  ConvergedInfeasible,
  /// @brief `max_iterations` QP solves reached.
  MaxIterations,
  /// @brief A QP could not be solved.
  QpFailure,
  /// @brief `max_time` exceeded.
  Timeout,
};

/// @brief Options of the vex trajectory optimizer. Joint limits are read from the Scene.
struct VexOptions {
  /// @brief The name of the joint group to optimize.
  std::string group_name;

  /// @brief Number of trajectory nodes N (>= 3), uniformly spaced in time.
  size_t num_nodes = 50;

  /// @brief Collision samples inside each segment, besides the nodes (0 = nodes only).
  size_t samples_per_segment = 3;

  /// @brief Weight of the integral of ||v(t)||^2.
  double velocity_weight = 1.0;

  /// @brief Weight of the integral of ||a(t)||^2.
  double acceleration_weight = 1e-2;

  /// @brief Minimum signed distance d_safe (m) on every active collision pair.
  /// @details Start and goal must already have this clearance, otherwise solve() fails.
  double collision_margin = 0.01;

  /// @brief Pairs closer than d_check (m) enter the QP; must exceed `collision_margin`.
  double collision_check_distance = 0.05;

  /// @brief Initial ℓ1 penalty weight on collision violations.
  double initial_penalty = 10.0;

  /// @brief Factor by which the penalty grows when converged with violations left.
  double penalty_increase = 10.0;

  /// @brief Maximum number of penalty increases.
  size_t max_penalty_increases = 5;

  /// @brief Initial trust box radius, in rad or m.
  double initial_trust_box = 0.1;

  /// @brief Factor on the trust box after a rejected step.
  double trust_shrink = 0.1;

  /// @brief Factor on the trust box after an accepted step.
  double trust_expand = 1.5;

  /// @brief Minimum ratio of actual to predicted merit decrease to accept a step.
  double improve_ratio = 0.25;

  /// @brief Maximum number of QP solves in the SCP loop.
  size_t max_iterations = 50;

  /// @brief Wall-time limit of the SCP loop, in seconds.
  double max_time = std::numeric_limits<double>::infinity();

  /// @brief Tolerance on collision violation, predicted decrease and trust box size.
  double tolerance = 1e-4;
};

/// @brief Result of a vex solve.
struct VexResult {
  /// @brief The trajectory over the group joints, not retimed.
  JointTrajectory trajectory;

  /// @brief The termination status.
  VexStatus status = VexStatus::QpFailure;

  /// @brief Number of QP solves in the SCP loop.
  size_t iterations = 0;

  /// @brief The smoothness cost 1/2 z^T P z of the result.
  double cost = 0.0;

  /// @brief The largest collision violation at the samples.
  double max_violation = 0.0;

  /// @brief Whether the dense post-check found no contact between the samples.
  /// @details Independent of the status. It does not check `collision_margin`.
  bool continuous_collision_free = false;
};

/// @brief Offline, kinematic trajectory optimizer based on sequential convex programming.
/// @details Details in docs/source/concepts/trajectory_optimization/vex.rst. Only vector-space
/// joints (nq == nv) are supported. solve() never modifies the Scene; use one Vex per thread.
class Vex {
public:
  /// @param scene The scene; must not be null.
  /// @param options The optimizer options, validated on every solve().
  Vex(std::shared_ptr<Scene> scene, VexOptions options);

  /// @brief Optimizes a rest-to-rest trajectory from start to goal in `duration` seconds.
  /// @details Joint limits hold at the nodes. Collision margins hold at the nodes and samples.
  /// Joints outside the group stay at the Scene's configuration.
  /// @param start The start configuration of the group joints.
  /// @param goal The goal configuration of the group joints.
  /// @param duration The duration T > 0 (s).
  /// @param seed The seed path over the group joints (at least 2 waypoints).
  /// @return The result, or an error for invalid input, an endpoint with a clearance below
  /// `collision_margin` (including in collision), or no feasible trajectory of that duration.
  tl::expected<VexResult, std::string> solve(const JointConfiguration& start,
                                             const JointConfiguration& goal, double duration,
                                             const JointPath& seed);

private:
  std::shared_ptr<Scene> scene_;
  VexOptions options_;
};

}  // namespace roboplan
