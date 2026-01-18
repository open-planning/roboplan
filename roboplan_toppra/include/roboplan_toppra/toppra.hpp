#pragma once

#include <string>
#include <vector>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>
#include <tl/expected.hpp>
#include <toppra/algorithm/toppra.hpp>

namespace roboplan {

/// @brief Trajectory time parameterizer using the TOPP-RA algorithm.
/// @details This directly uses https://github.com/hungpham2511/toppra.
class PathParameterizerTOPPRA {
public:
  /// @brief Constructor.
  /// @param scene A pointer to the scene to use for path parameterization.
  /// @param group_name The name of the joint group to use.
  PathParameterizerTOPPRA(const std::shared_ptr<Scene> scene, const std::string& group_name = "");

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
  /// @brief A pointer to the scene.
  std::shared_ptr<Scene> scene_;

  /// @brief The name of the joint group.
  std::string group_name_;

  /// @brief The joint group info for the path parameterizer.
  JointGroupInfo joint_group_info_;

  /// @brief The stored velocity lower limits.
  toppra::Vector vel_lower_limits_;

  /// @brief The stored velocity upper limits.
  toppra::Vector vel_upper_limits_;

  /// @brief The stored acceleration lower limits.
  toppra::Vector acc_lower_limits_;

  /// @brief The stored acceleration upper limits.
  toppra::Vector acc_upper_limits_;

  /// @brief A list of indices of joints with continuous degrees of freedom.
  /// @details This is used to figure out which joints need to be wrapped.
  std::vector<size_t> continuous_joint_indices_;
};

}  // namespace roboplan
