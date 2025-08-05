#pragma once

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
  /// @return A time-parameterized joint trajectory.
  tl::expected<JointTrajectory, std::string> generate(const JointPath& path, const double dt);

private:
  /// @brief The constraints (velocity and acceleration limits) extracted from the scene.
  toppra::LinearConstraintPtrs constraints_;
};

}  // namespace roboplan
