#pragma once

#include <string>
#include <vector>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>
#include <tl/expected.hpp>
#include <toppra/algorithm/toppra.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>

namespace roboplan {

/// @brief Enumeration for TOPP-RA spline fitting mode.
/// @details Refer to the PathParameterizerTOPPRA options for more information.
enum class SplineFittingMode {
  Hermite,
  Cubic,
};

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
  /// @param mode The mode to use for spline fitting the path. Options include:
  ///
  ///   - `SplineFittingMode::Hermite`: Fits a cubic Hermite spline with zero velocity at all
  ///   waypoints. This can cause slow execution, but guarantees perfect adherence to the path.
  ///   - `SplineFittingMode::Cubic`: Fits a cubic spline with zero velocity only at the endpoints.
  ///     This is smoother, but can cause deviations from the desired path that could lead to
  ///     collision.
  /// @param velocity_scale A scaling factor (between 0 and 1) for velocity limits.
  /// @param acceleration_scale A scaling factor (between 0 and 1) for acceleration limits.
  /// @return A time-parameterized joint trajectory.
  tl::expected<JointTrajectory, std::string>
  generate(const JointPath& path, const double dt,
           const SplineFittingMode mode = SplineFittingMode::Hermite,
           const double velocity_scale = 1.0, const double acceleration_scale = 1.0);

private:
  /// @brief Helper function to convert the raw joint path to TOPP-RA compatible position vectors.
  /// @param path The joint path to convert.
  /// @return The TOPP-RA compatible vectors of collapsed joint paths if successful, else a string
  /// describing the error.
  tl::expected<toppra::Vectors, std::string> getPathPositionVectors(const JointPath& path);

  /// @brief Helper function to extract a natural cubic spline from a joint path.
  /// @details This defines zero velocity and acceleration at the endpoints only, meaning that
  /// intermediate waypoints are passed through smoothly, but could deviate from the original path
  /// and therefore lead to collisions.
  /// @param path The joint path.
  /// @return The resulting cubic spline if successful, else a string describing the error.
  tl::expected<std::shared_ptr<toppra::PiecewisePolyPath>, std::string>
  generateCubicSpline(const JointPath& path);

  /// @brief Helper function to extract a cubic Hermite spline from a joint path.
  /// @details This enforces zero velocity and acceleration at all waypoints, meaning the desired
  /// path is exactly adhered to. If the path was checked for collisions, this spline is also safe.
  /// @param path The joint path.
  /// @return The resulting cubic Hermite spline if successful, else a string describing the error.
  tl::expected<std::shared_ptr<toppra::PiecewisePolyPath>, std::string>
  generateCubicHermiteSpline(const JointPath& path);

  /// @brief A pointer to the scene.
  std::shared_ptr<Scene> scene_;

  /// @brief The name of the joint group.
  std::string group_name_;

  /// @brief The names of the joints in the group.
  std::vector<std::string> joint_names_;

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
