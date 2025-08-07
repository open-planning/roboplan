#pragma once

#include <string>
#include <vector>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>
#include <tl/expected.hpp>
#include <toppra/algorithm/toppra.hpp>

namespace roboplan {

/// @brief Time-parameterizes a joint-space path using TOPP-RA.
/// @details This directly uses https://github.com/hungpham2511/toppra.
class PathParameterizerTOPPRA {
public:
  /// @brief Constructor.
  /// @param scene A pointer to the scene to use for path parameterization.
  PathParameterizerTOPPRA(const std::shared_ptr<Scene> scene);

  /// @brief Time-parameterizes a joint-space path using TOPP-RA.
  /// @param path The path to time parameterize.
  /// @param dt The sample time of the output trajectory, in seconds.
  /// @param velocity_scale A scaling factor (between 0 and 1) for velocity limits.
  /// @param acceleration_scale A scaling factor (between 0 and 1) for acceleration limits.
  /// @return A time-parameterized joint trajectory.
  tl::expected<JointTrajectory, std::string> generate(const JointPath& path, const double dt,
                                                      const double velocity_scale = 1.0,
                                                      const double acceleration_scale = 1.0);

private:
  /// @brief The stored joint names to use for this path parameterizer.
  std::vector<std::string> joint_names_;

  /// @brief The stored velocity lower limits.
  toppra::Vector vel_lower_limits_;

  /// @brief The stored velocity upper limits.
  toppra::Vector vel_upper_limits_;

  /// @brief The stored acceleration lower limits.
  toppra::Vector acc_lower_limits_;

  /// @brief The stored acceleration upper limits.
  toppra::Vector acc_upper_limits_;
};

}  // namespace roboplan
