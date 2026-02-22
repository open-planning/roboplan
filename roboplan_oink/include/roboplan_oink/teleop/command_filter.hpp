#pragma once

#include <Eigen/Dense>

namespace roboplan {

/// @brief Configuration for teleop command conditioning.
struct CommandFilterConfig {
  double linear_deadzone = 0.05;    ///< Unitless threshold in [0, 1]
  double angular_deadzone = 0.05;   ///< Unitless threshold in [0, 1]
  double linear_sensitivity = 0.12; ///< m/s at unit input
  double angular_sensitivity = 0.60; ///< rad/s at unit input
  double max_linear_speed = 0.25;   ///< m/s speed clamp
  double max_angular_speed = 1.50;  ///< rad/s speed clamp
  double smoothing_alpha = 0.35;    ///< EMA coefficient in [0, 1]
};

/// @brief Applies deadzone, scaling, clamping, and smoothing to 6D teleop commands.
class CommandFilter {
public:
  explicit CommandFilter(const CommandFilterConfig& config = {});

  /// @brief Filter a raw normalized twist command.
  /// @param raw_twist [vx, vy, vz, wx, wy, wz], each axis expected in [-1, 1].
  /// @return Filtered twist in physical units [m/s, m/s, m/s, rad/s, rad/s, rad/s].
  Eigen::Matrix<double, 6, 1>
  filter(const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& raw_twist);

  /// @brief Reset internal smoothing state.
  void reset();

  /// @brief Replace the active filter configuration.
  void setConfig(const CommandFilterConfig& config);

  /// @brief Returns the active filter configuration.
  const CommandFilterConfig& getConfig() const;

private:
  static void applyDeadzone(Eigen::Ref<Eigen::Vector3d> values, double threshold);

  CommandFilterConfig config_;
  Eigen::Vector3d prev_linear_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d prev_angular_ = Eigen::Vector3d::Zero();
};

}  // namespace roboplan
