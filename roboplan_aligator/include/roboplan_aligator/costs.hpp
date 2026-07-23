#pragma once

#include <memory>
#include <string>

#include <Eigen/Dense>

namespace roboplan {

// Cost specifications (design §4.3). These are plain value structs describing a soft objective in
// terms the USER understands (frame names, targets, per-DoF weights). They carry no aligator or
// pinocchio types and no reference to the optimizer's internal reduced model: the frame is resolved
// and the aligator residual is built inside TrajectoryOptimizer::addCost, which owns the reduced
// model (§4.3 "the factories receive rgm, not a public type users construct").
//
// Each is attached with `TrajectoryOptimizer::addCost(spec, window, weight)`, which returns a
// CostHandle whose `setTarget` mutates the in-problem residual on the hot path (§3.5).

/// @brief Penalize the SE3 placement error of a frame from a target pose (design §4.3).
/// @details Maps to aligator FramePlacementResidual -> QuadraticResidualCost. The 6-D placement
/// error (3 translation, 3 rotation-log) is weighted per-axis by `position_cost` / `orientation_cost`.
struct FramePoseCost {
  /// @brief Name of the frame whose pose is penalized (resolved against the reduced model).
  std::string frame;

  /// @brief Target pose as a 4x4 homogeneous transform (world <- frame).
  Eigen::Matrix4d target = Eigen::Matrix4d::Identity();

  /// @brief Per-axis translation weights (x, y, z). Nonnegative.
  Eigen::Vector3d position_cost = Eigen::Vector3d::Ones();

  /// @brief Per-axis rotation-log weights (rx, ry, rz). Nonnegative.
  Eigen::Vector3d orientation_cost = Eigen::Vector3d::Ones();
};

/// @brief Align a body-fixed axis with a world-target direction (design §4.3).
/// @details Maps to a custom frame-vector residual r(q) = R_wf(q)*axis_local - axis_world_target
/// -> QuadraticResidualCost. For unit axes the cost equals (1 - cos(angle)) up to scale (the
/// vector-difference scalarization; maintainer decision, Prompt 6).
struct FrameAxisCost {
  /// @brief Name of the frame carrying the body-fixed axis (resolved against the reduced model).
  std::string frame;

  /// @brief The body-fixed axis, expressed in the frame (typically a unit vector).
  Eigen::Vector3d axis_local = Eigen::Vector3d::UnitZ();

  /// @brief The desired world-space direction for `axis_local` (typically a unit vector).
  Eigen::Vector3d axis_world_target = Eigen::Vector3d::UnitZ();

  /// @brief Scalar weight on the (isotropic) 3-vector residual. Nonnegative.
  double weight = 1.0;
};

/// @brief Penalize deviation of the reduced-group configuration from a target (design §4.3).
/// @details Maps to aligator QuadraticStateCost (StateError -> quadratic) with the velocity block
/// masked to zero.
struct ConfigurationCost {
  /// @brief Target reduced-group configuration (size nq).
  Eigen::VectorXd q_target;

  /// @brief Per-DoF tangent weights (size nv). Nonnegative.
  Eigen::VectorXd weights;
};

/// @brief Penalize control (joint torque) deviation from a target (design §4.3).
/// @details Maps to aligator QuadraticControlCost (ControlError -> quadratic).
struct ControlCost {
  /// @brief Per-DoF control weights (size nv). Nonnegative.
  Eigen::VectorXd weights;

  /// @brief Target control (size nv); if left empty, the zero vector is used.
  Eigen::VectorXd u_target;
};

/// @brief Penalize reduced-group velocity deviation from a target (design §4.3).
/// @details Maps to aligator QuadraticStateCost with the configuration block masked to zero.
struct VelocityCost {
  /// @brief Per-DoF velocity weights (size nv). Nonnegative.
  Eigen::VectorXd weights;

  /// @brief Target velocity (size nv); if left empty, the zero vector is used.
  Eigen::VectorXd v_target;
};

/// @brief Mutable handle to an attached cost, for hot-path target updates (design §3.5).
/// @details Returned by `TrajectoryOptimizer::addCost`. aligator deep-copies the cost you pass into
/// each in-range stage of the assembled problem, so this handle retains references to the residuals
/// living INSIDE the problem; `setTarget` rewrites those in place (legal between solves, no rebuild).
/// Move-only; becomes dangling if the owning optimizer is destroyed or `resetProblem()` is called.
class CostHandle {
public:
  /// @brief Constructs an empty handle (references nothing). Provided for default-construction only.
  CostHandle();
  ~CostHandle();

  CostHandle(CostHandle&&) noexcept;
  CostHandle& operator=(CostHandle&&) noexcept;
  CostHandle(const CostHandle&) = delete;
  CostHandle& operator=(const CostHandle&) = delete;

  /// @brief Sets a new target pose for a FramePoseCost handle (world <- frame, 4x4 homogeneous).
  /// @throws std::logic_error if this handle was not returned for a FramePoseCost.
  void setTarget(const Eigen::Matrix4d& target_pose);

  /// @brief Sets a new target vector for a ConfigurationCost (q), ControlCost (u), VelocityCost (v),
  /// or FrameAxisCost (world axis, size 3) handle.
  /// @throws std::logic_error if this handle is a FramePoseCost handle (use the Matrix4d overload).
  /// @throws std::invalid_argument if `target` has the wrong size for the cost.
  void setTarget(const Eigen::VectorXd& target);

private:
  friend class TrajectoryOptimizer;
  struct Impl;
  explicit CostHandle(std::unique_ptr<Impl> impl);

  std::unique_ptr<Impl> impl_;
};

}  // namespace roboplan
