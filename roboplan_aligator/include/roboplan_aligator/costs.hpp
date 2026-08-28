#pragma once

#include <memory>
#include <string>

#include <Eigen/Dense>

namespace roboplan {

// A cost is a soft objective described in terms the user understands (frame names, targets,
// per-DoF weights). Each is attached with `TrajectoryOptimizer::addCost(spec, window, weight)`,
// which returns a CostHandle whose `setTarget` updates the target between solves.

/// @brief Penalize the frame's SE3 placement error from a target pose.
/// @details The 6-D error (3 translation, 3 rotation-log) is weighted per-axis by
/// `position_cost` / `orientation_cost`.
struct FramePoseCost {
  /// @brief Name of the frame whose pose is penalized.
  std::string frame;

  /// @brief Target pose as a 4x4 homogeneous transform (world <- frame).
  Eigen::Matrix4d target = Eigen::Matrix4d::Identity();

  /// @brief Per-axis translation weights (x, y, z). Nonnegative.
  Eigen::Vector3d position_cost = Eigen::Vector3d::Ones();

  /// @brief Per-axis rotation-log weights (rx, ry, rz). Nonnegative.
  Eigen::Vector3d orientation_cost = Eigen::Vector3d::Ones();
};

/// @brief Align a body-fixed axis with a world-target direction.
struct FrameAxisCost {
  /// @brief Name of the frame carrying the body-fixed axis.
  std::string frame;

  /// @brief The body-fixed axis, expressed in the frame (typically a unit vector).
  Eigen::Vector3d axis_local = Eigen::Vector3d::UnitZ();

  /// @brief The desired world-space direction for `axis_local` (typically a unit vector).
  Eigen::Vector3d axis_world_target = Eigen::Vector3d::UnitZ();

  /// @brief Scalar weight on the residual. Nonnegative.
  double weight = 1.0;
};

/// @brief Penalize deviation of the reduced-group configuration from a target.
struct ConfigurationCost {
  /// @brief Target reduced-group configuration (size nq).
  Eigen::VectorXd q_target;

  /// @brief Per-DoF weights (size nv). Nonnegative.
  Eigen::VectorXd weights;
};

/// @brief Penalize control (joint torque) deviation from a target.
struct ControlCost {
  /// @brief Per-DoF control weights (size nv). Nonnegative.
  Eigen::VectorXd weights;

  /// @brief Target control (size nv); empty means zero.
  Eigen::VectorXd u_target;
};

/// @brief Penalize reduced-group velocity deviation from a target.
struct VelocityCost {
  /// @brief Per-DoF velocity weights (size nv). Nonnegative.
  Eigen::VectorXd weights;

  /// @brief Target velocity (size nv); empty means zero.
  Eigen::VectorXd v_target;
};

/// @brief Mutable handle to an attached cost, for target updates between solves.
/// @details Returned by `TrajectoryOptimizer::addCost`. `setTarget` rewrites the target in place
/// (no rebuild, legal between solves). Move-only; dangles if the owning optimizer is destroyed or
/// `resetProblem()` is called.
class CostHandle {
public:
  /// @brief Constructs an empty handle (references nothing). Provided for default-construction
  /// only.
  CostHandle();
  ~CostHandle();

  CostHandle(CostHandle&&) noexcept;
  CostHandle& operator=(CostHandle&&) noexcept;
  CostHandle(const CostHandle&) = delete;
  CostHandle& operator=(const CostHandle&) = delete;

  /// @brief Sets a new target pose for a FramePoseCost handle (world <- frame, 4x4 homogeneous).
  /// @throws std::logic_error if this handle was not returned for a FramePoseCost.
  void setTarget(const Eigen::Matrix4d& target_pose);

  /// @brief Sets a new target vector for a ConfigurationCost (q), ControlCost (u), VelocityCost
  /// (v), or FrameAxisCost (world axis, size 3) handle.
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
