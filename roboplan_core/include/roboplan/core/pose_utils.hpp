#pragma once

#include <utility>

#include <Eigen/Dense>

namespace roboplan {

/// @brief Computes the position (meters) and orientation (radians) error between two
/// SE(3) transforms expressed in the same frame.
/// @param a The first transform.
/// @param b The second transform.
/// @return A pair of {position error, orientation error}.
std::pair<double, double> poseError(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b);

/// @brief Interpolates between two SE(3) transforms: linear in position, SLERP in orientation.
/// @param start The transform at fraction 0.
/// @param end The transform at fraction 1.
/// @param fraction The interpolation coefficient, between 0 and 1.
/// @return The interpolated transform.
Eigen::Matrix4d interpolatePose(const Eigen::Matrix4d& start, const Eigen::Matrix4d& end,
                                double fraction);

}  // namespace roboplan
