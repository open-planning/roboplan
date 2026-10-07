#pragma once

#include <string>

#include <Eigen/Dense>
#include <roboplan_oink/optimal_ik.hpp>

namespace roboplan {

/// @brief Hard constraint that keeps the pose of one frame relative to another near a target.
///
/// With T_ab the pose of `frame_b` in `frame_a` and T_target its desired value, the error is
/// expressed in the target frame:
///     e_pos = R_target^T (p_ab - p_target)
///     e_rot = log_3(R_target^T R_ab)
///
/// Linearizing e(q + δq) ≈ e + J_e δq, each of the 6 rows is bounded by its tolerance:
///     -tol - e <= J_e δq <= tol - e
///
/// so the error stays inside the tolerance box, and is pulled back into it if it starts outside.
/// Use an infinite tolerance to leave an axis free.
struct RelativePoseConstraint : public Constraints {
  /// @brief Constructor.
  /// @param oink The Oink solver this constraint will be used with (provides v_indices).
  /// @param scene The scene used to resolve the frame IDs and allocate storage.
  /// @param frame_a Name of the reference frame.
  /// @param frame_b Name of the constrained frame.
  /// @param target_pose Target pose of frame_b in frame_a (4x4 homogeneous transform).
  /// @param position_tolerance Per-axis position tolerance (meters), in the target frame.
  /// @param orientation_tolerance Per-axis rotation tolerance (radians), in the target frame.
  /// @throws std::runtime_error if either frame is not found in the scene.
  RelativePoseConstraint(const Oink& oink, const Scene& scene, const std::string& frame_a,
                         const std::string& frame_b, const Eigen::Matrix4d& target_pose,
                         const Eigen::Vector3d& position_tolerance = Eigen::Vector3d::Zero(),
                         const Eigen::Vector3d& orientation_tolerance = Eigen::Vector3d::Zero());

  /// @brief Returns 6 (3 position + 3 orientation rows).
  int getNumConstraints(const SceneContext& context) const override;

  tl::expected<void, std::string>
  computeQpConstraints(const SceneContext& context, Eigen::Ref<Eigen::MatrixXd> constraint_matrix,
                       Eigen::Ref<Eigen::VectorXd> lower_bounds,
                       Eigen::Ref<Eigen::VectorXd> upper_bounds) const override;

  /// @brief The 6D error [e_pos, e_rot] of frame_b relative to frame_a, less the tolerance box:
  /// zero on the axes inside their tolerance, the signed excess on those outside.
  /// @param context The context whose frame placements to read.
  Eigen::VectorXd computeViolation(const SceneContext& context) const override;

  std::string frame_a;
  std::string frame_b;
  pinocchio::FrameIndex frame_a_id;
  pinocchio::FrameIndex frame_b_id;
  Eigen::Matrix4d target_pose;
  Eigen::Vector3d position_tolerance;
  Eigen::Vector3d orientation_tolerance;
  Eigen::VectorXi v_indices;

  /// @brief Pre-allocated full-robot relative Jacobian (6 x model.nv).
  mutable Eigen::MatrixXd full_jacobian;
};

}  // namespace roboplan
