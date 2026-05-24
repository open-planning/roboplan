#pragma once

#include <Eigen/Dense>
#include <vector>

#include <pinocchio/multibody/geometry.hpp>

#include <roboplan_oink/optimal_ik.hpp>

namespace roboplan {

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
  /// @param n_collision_pairs Number of closest collision pairs to consider. Must be > 0 and
  ///        no greater than the number of collision pairs in the scene's collision model.
  ///        If fewer pairs than the total are requested, only the closest ones are constrained.
  /// @param dt Timestep matching the control loop period (must match actual control loop).
  /// @param gain Barrier gain (gamma), controls convergence to safe set. Default 1.0.
  /// @param safe_displacement_gain Gain for safe displacement regularization. Default 1.0.
  /// @param d_min Minimum allowed distance between any pair of bodies. Must be non-negative.
  ///        Default 0.02.
  /// @param safety_margin Conservative margin for hard constraint guarantee. Default 0.0.
  /// @throws std::invalid_argument if d_min is negative, n_collision_pairs is non-positive,
  ///         or n_collision_pairs exceeds the number of collision pairs in the scene.
  SelfCollisionBarrier(const Oink& oink, const Scene& scene, int n_collision_pairs, double dt,
                       double gain = 1.0, double safe_displacement_gain = 1.0, double d_min = 0.02,
                       double safety_margin = 0.0);

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
  /// Refreshes geometry placements on a private GeometryData copy and runs narrow-phase
  /// distance only on the pairs cached by the most recent computeBarrier() call
  /// (`closest_pair_indices`). For small displacements between the configuration used in
  /// computeBarrier() and `q`, those are the active constraints, and skipping narrow phase
  /// on the remaining pairs is the dominant per-solve speedup. computeBarrier() must have
  /// run before this method.
  ///
  /// @param model Pinocchio model (must be the same as the scene's model).
  /// @param data Pinocchio data (will be modified by FK / distance computation).
  /// @param q Candidate joint configuration to evaluate.
  /// @return Expected containing the minimum barrier value (negative if any pair is in
  ///         collision), or an error message on failure.
  tl::expected<double, std::string>
  evaluateAtConfiguration(const pinocchio::Model& model, pinocchio::Data& data,
                          const Eigen::VectorXd& q) const override;

  /// @brief Number of closest collision pairs to constrain.
  const int n_collision_pairs;

  /// @brief Minimum allowed distance between any pair of bodies.
  const double d_min;

  /// @brief Velocity indices of the joint group (for Jacobian column selection).
  Eigen::VectorXi v_indices;

  /// @brief Indices into the scene's collision model collision pairs that were selected
  ///        in the most recent computeBarrier() call (size n_collision_pairs).
  std::vector<std::size_t> closest_pair_indices;

  /// @brief Workspace buffer holding `min_distance` for every collision pair in the model,
  ///        repopulated on each computeBarrier() call from the scene's GeometryData.
  Eigen::VectorXd all_distances;

  /// @brief Pre-allocated workspace for one parent-joint Jacobian (6 x model.nv).
  mutable Eigen::MatrixXd joint_jacobian1;

  /// @brief Pre-allocated workspace for the other parent-joint Jacobian (6 x model.nv).
  mutable Eigen::MatrixXd joint_jacobian2;

  /// @brief Pre-allocated workspace for a single barrier-Jacobian row across all model.nv
  ///        velocity DOFs (before selecting the joint-group columns).
  mutable Eigen::RowVectorXd full_row;

  /// @brief Pointer to the scene's collision model (non-owning). Captured at construction.
  const pinocchio::GeometryModel* collision_model;

  /// @brief Private GeometryData used by evaluateAtConfiguration to avoid mutating scene state.
  mutable pinocchio::GeometryData eval_geom_data;
};

}  // namespace roboplan
