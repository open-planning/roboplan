#pragma once

#include <vector>

#include <Eigen/Dense>
#include <roboplan/core/scene.hpp>

namespace roboplan {

/// @brief Computes the Cartesian path of a specified frame.
/// @param scene The scene to use for interpolating positions.
/// @param q_start The starting joint positions.
/// @param q_end The ending joint positions.
/// @param frame_name The name of the frame in which to compute the Cartesian path.
/// @param max_step_size The maximum configuration distance step size for interpolation.
/// @return A list of 4x4 matrices corresponding to the poses of the frame along the path.
std::vector<Eigen::Matrix4d> computeFramePath(const Scene& scene, const Eigen::VectorXd& q_start,
                                              const Eigen::VectorXd& q_end,
                                              const std::string& frame_name,
                                              const double max_step_size);

/// @brief Checks collisions along a specified configuration space path.
/// @param scene The scene to use for interpolating positions and checking collisions.
/// @param q_start The starting joint positions.
/// @param q_end The ending joint positions.
/// @param max_step_size The maximum configuration distance step size for interpolation.
/// @return True if there are collisions, else false.
bool hasCollisionsAlongPath(const Scene& scene, const Eigen::VectorXd& q_start,
                            const Eigen::VectorXd& q_end, const double max_step_size);

}  // namespace roboplan
