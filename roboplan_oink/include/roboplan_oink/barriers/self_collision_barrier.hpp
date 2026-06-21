#pragma once

#include <Eigen/Dense>
#include <optional>
#include <vector>

#include <pinocchio/multibody/geometry.hpp>

#include <roboplan/core/collision_context.hpp>
#include <roboplan_oink/optimal_ik.hpp>

namespace roboplan {

/// @brief Parameters for SelfCollisionBarrier configuration.
struct SelfCollisionBarrierOptions {
  /// @brief Maximum number of closest collision pairs to constrain. Must be > 0. Only the closest
  /// `n_collision_pairs` pairs at the current configuration are constrained. Values larger than the
  /// number of collision pairs in the scene are clipped to that count.
  int n_collision_pairs = 1;

  /// @brief Barrier gain (gamma), controls convergence to safe set (default: 1.0).
  double gain = 1.0;

  /// @brief Gain for safe displacement regularization (default: 1.0).
  double safe_displacement_gain = 1.0;

  /// @brief Minimum allowed distance between any pair of bodies. Must be non-negative
  /// (default: 0.02).
  double d_min = 0.02;

  /// @brief Conservative margin for hard constraint guarantee (default: 0.0).
  double safety_margin = 0.0;

  /// @brief Maximum distance (meters) at which a collision pair is tracked.
  /// @details Pairs whose bounding boxes are farther apart than this skip exact narrow-phase
  /// distance computation (the dominant per-solve cost on dense / mesh-heavy models) and
  /// therefore exert no influence on the barrier.
  ///
  /// This is a visibility / performance bound, NOT a separation limit: it does not constrain how
  /// far apart bodies may be. When set comfortably larger than the distances at which the barrier
  /// actively pushes (a few times d_min), it does not change the solution at all — only a too-small
  /// value silently drops mid-range pairs. Default 0.5. Set to std::nullopt to disable culling.
  /// Paired with d_min, it defines the band [d_min, d_max] of separations in the barrier.
  std::optional<double> d_max = 0.5;
};

/// @brief Self-collision avoidance barrier based on collision pair distances.
///
/// Constrains the closest `n_collision_pairs` collision pairs in the scene to remain at least
/// `d_min` apart:
///     h_i(q) = d(p_a, p_b)_i - d_min  for i in 0 .. n_collision_pairs - 1
/// where the index `i` refers to the i-th closest pair at the current configuration.
///
/// The barrier Jacobian row is built from the witness points returned by collision checking
/// and the joint Jacobians of the parent joints of each colliding body:
///     J_i = n^T  J^1_p + (r_1 × n)^T  J^1_w
///         - n^T  J^2_p - (r_2 × n)^T  J^2_w
/// where n is the unit vector from witness point 1 to witness point 2 (in world coordinates),
/// r_k is the vector from joint k origin to its witness point, and J^k_p / J^k_w are the
/// linear / angular parts of the LOCAL_WORLD_ALIGNED joint Jacobian.
///
/// Uses the same saturating class-K function α(h) = γ·h/(1+|h|) as other barriers.
///
/// @note Inspired by Pink's pink.barriers.SelfCollisionBarrier (Apache-2.0).
/// https://github.com/stephane-caron/pink/blob/main/pink/barriers/self_collision_barrier.py
struct SelfCollisionBarrier : public Barrier {
  /// @brief Constructs a self-collision barrier.
  /// @param oink The Oink solver this barrier will be used with.
  /// @param scene The scene whose collision model defines the collision pairs to monitor.
  /// @param dt Timestep matching the control loop period. Must be positive.
  /// @param options Optional tuning parameters (default: all options set to defaults).
  /// @throws std::invalid_argument if d_min is negative or n_collision_pairs is non-positive.
  SelfCollisionBarrier(const Oink& oink, const Scene& scene, double dt,
                       const SelfCollisionBarrierOptions& options = {});

  /// @brief Get the number of active barrier constraints.
  /// @return The number of collision pairs being constrained.
  int getNumBarriers(const Scene& scene) const override;

  /// @brief Compute barrier function values h(q) for the closest n_collision_pairs pairs.
  ///
  /// Triggers distance computation on the scene's collision data, then fills
  /// `barrier_values` with the distances of the closest pairs, each shifted by `-d_min`.
  /// The selected pair indices are cached in `closest_pair_indices` for use by
  /// computeJacobian().
  ///
  /// @param scene The scene containing the robot collision model and current state.
  /// @return Expected void on success, error message on failure.
  tl::expected<void, std::string> computeBarrier(const Scene& scene) override;

  /// @brief Compute the Jacobian rows for the closest n_collision_pairs collision pairs.
  ///
  /// Assumes computeBarrier() has been called first (it caches the selected pair indices).
  /// Each row uses the witness points and the parent-joint Jacobians of the two bodies in
  /// the collision pair. If the witness points coincide, the row is zeroed (Jacobian is
  /// undefined there). Any NaN values are clamped to zero.
  ///
  /// @param scene The scene containing the robot collision model and current state.
  /// @return Expected void on success, error message on failure.
  tl::expected<void, std::string> computeJacobian(const Scene& scene) override;

  /// @brief Evaluate the minimum barrier value at a candidate configuration.
  ///
  /// Refreshes geometry placements on this barrier's own CollisionContext scratch and runs
  /// narrow-phase distance only on the pairs cached by the most recent computeBarrier() call
  /// (`closest_pair_indices`). For small displacements between the configuration used in
  /// computeBarrier() and `q`, those are the active constraints, and skipping narrow phase
  /// on the remaining pairs is the dominant per-solve speedup. computeBarrier() must have
  /// run before this method.
  ///
  /// @param model Unused; kept for the Barrier interface. Distances are evaluated on this
  ///        barrier's CollisionContext, which owns the model shared with the scene.
  /// @param data Unused; kept for the Barrier interface.
  /// @param q Candidate joint configuration to evaluate.
  /// @return Expected containing the minimum barrier value (negative if any pair is in
  ///         collision), or an error message on failure.
  tl::expected<double, std::string>
  evaluateAtConfiguration(const pinocchio::Model& model, pinocchio::Data& data,
                          const Eigen::VectorXd& q) const override;

  /// @brief Number of closest collision pairs constrained.
  int n_collision_pairs;

  /// @brief Minimum allowed distance between any pair of bodies.
  const double d_min;

  /// @brief Maximum distance (meters) at which a collision pair is tracked; pairs whose bounding
  ///        boxes are farther apart than this skip exact narrow-phase distance. Visibility /
  ///        performance bound, not a separation limit. std::nullopt disables culling.
  ///        See SelfCollisionBarrierOptions::d_max.
  const std::optional<double> d_max;

  /// @brief Velocity indices of the joint group (for Jacobian column selection).
  Eigen::VectorXi v_indices;

  /// @brief Indices into the scene's collision model collision pairs that were selected
  ///        in the most recent computeBarrier() call (size n_collision_pairs).
  std::vector<std::size_t> closest_pair_indices;

  /// @brief Workspace buffer holding `min_distance` for every collision pair in the model,
  ///        repopulated on each computeBarrier() call from the CollisionContext's GeometryData.
  Eigen::VectorXd all_distances;

  /// @brief Pre-allocated workspace for one parent-joint Jacobian (6 x model.nv).
  mutable Eigen::MatrixXd joint_jacobian1;

  /// @brief Pre-allocated workspace for the other parent-joint Jacobian (6 x model.nv).
  mutable Eigen::MatrixXd joint_jacobian2;

  /// @brief Pre-allocated workspace for a single barrier-Jacobian row across all model.nv
  ///        velocity DOFs (before selecting the joint-group columns).
  mutable Eigen::RowVectorXd full_row;

  /// @brief Non-owning pointer to the Oink solver's shared collision scratch (Data + GeometryData),
  /// captured from `oink.getCollisionContext()` at construction. All distance / joint-Jacobian
  /// queries run on this context instead of the scene's shared collision data, so the barrier never
  /// mutates scene state. The referenced Oink must outlive this barrier.
  const CollisionContext* collision_context;
};

}  // namespace roboplan
