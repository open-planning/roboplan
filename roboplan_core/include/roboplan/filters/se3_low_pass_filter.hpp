#pragma once

#include <Eigen/Geometry>
#include <pinocchio/spatial/se3.hpp>

namespace roboplan {

/// @brief First-order low-pass filter for SE3 poses.
class SE3LowPassFilter {
public:
  /// @brief Creates an SE3 low-pass filter.
  /// @param tau Time constant in seconds. Larger values produce slower, smoother tracking.
  SE3LowPassFilter(double tau = 0.1);

  /// @brief Resets the filter state to a specific pose.
  /// @param pose Pose used as the new filtered state.
  void reset(const pinocchio::SE3& pose);

  /// @brief Updates the filtered state toward a target pose.
  /// @param target_pose Target pose to filter toward.
  /// @param dt Time step in seconds.
  /// @return The updated filtered pose.
  pinocchio::SE3 update(const pinocchio::SE3& target_pose, double dt);

  /// @brief Returns the filter time constant in seconds.
  /// @return Filter time constant.
  double tau() const;

  /// @brief Sets the filter time constant.
  /// @param tau Time constant in seconds. Larger values produce slower, smoother tracking.
  void setTau(double tau);

  /// @brief Checks whether the filter has an active filtered state.
  /// @return True if the filter has been initialized by reset() or update().
  bool isInitialized() const;

private:
  /// @brief Filter time constant in seconds.
  double tau_;

  /// @brief Whether the filter currently has a valid filtered pose.
  bool initialized_;

  /// @brief Current filtered translation component.
  Eigen::Vector3d filtered_position_;

  /// @brief Current filtered rotation component.
  Eigen::Quaterniond filtered_quaternion_;
};

}  // namespace roboplan
