#pragma once

#include <Eigen/Dense>

namespace roboplan {

enum class TeleopFrame {
  EndEffector = 0,
  World = 1,
};

struct WorkspaceBounds {
  Eigen::Vector3d min_xyz = Eigen::Vector3d(-1.2, -1.2, -0.05);
  Eigen::Vector3d max_xyz = Eigen::Vector3d(1.2, 1.2, 1.6);
};

/// @brief Maintains and updates a teleop target pose from twist commands.
class TeleopTarget {
public:
  TeleopTarget(const Eigen::Matrix4d& initial_pose, const WorkspaceBounds& bounds = {},
               TeleopFrame frame = TeleopFrame::EndEffector);

  /// @brief Integrate twist into the target pose using the default frame mode.
  /// @param twist [vx, vy, vz, wx, wy, wz] in physical units [m/s, rad/s].
  /// @param dt Timestep in seconds.
  void update(const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& twist, double dt);

  /// @brief Integrate twist into the target pose with an explicit frame mode.
  /// @param twist [vx, vy, vz, wx, wy, wz] in physical units [m/s, rad/s].
  /// @param dt Timestep in seconds.
  /// @param frame Frame mode for this update.
  void update(const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& twist, double dt,
              TeleopFrame frame);

  /// @brief Set the target pose directly (with workspace clamp).
  void resetTo(const Eigen::Matrix4d& pose);

  /// @brief Returns the current target pose.
  Eigen::Matrix4d getTarget() const;

  /// @brief Set default update frame.
  void setFrame(TeleopFrame frame);

  /// @brief Returns default update frame.
  TeleopFrame getFrame() const;

  /// @brief Update workspace bounds and clamp target to the new bounds.
  void setBounds(const WorkspaceBounds& bounds);

  /// @brief Returns workspace bounds.
  const WorkspaceBounds& getBounds() const;

private:
  void clampWorkspace();

  Eigen::Matrix4d target_;
  WorkspaceBounds bounds_;
  TeleopFrame frame_;
};

}  // namespace roboplan
