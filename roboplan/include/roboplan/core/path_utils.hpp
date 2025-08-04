#pragma once

#include <vector>

#include <Eigen/Dense>
#include <roboplan/core/scene.hpp>
#include <tl/expected.hpp>

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

/// @brief Attempts to shortcut the path with random sampling and checking connections.
/// @details This implementation is based on section 3.5.3 of:
/// https://motion.cs.illinois.edu/RoboticSystems/MotionPlanningHigherDimensions.html
/// @param scene The scene for checking connectability between joint poses.
/// @param path The JointPath to try to shorten.
/// @param max_step_size Maximum step size to use in collision checking.
/// @param max_iters Maximum number of iterations of random sampling (default 100).
/// @param seed Seed for the random generator, if < 0 then use a random seed (default -1).
/// @return A shortcutted JointPath, if available.
JointPath shortcutPath(const Scene& scene, const JointPath& path, double max_step_size,
                       unsigned int max_iters = 100, int seed = 0);

/// @brief Computes configuration distances from the start to each pose in a path.
/// @param scene The scene for checking distances between joint poses.
/// @param path The JointPath to evaluate.
/// @return A vector of incremental path distances, if there is sufficient data. Otherwise an error.
tl::expected<Eigen::VectorXd, std::string> getPathLengths(const Scene& scene,
                                                          const JointPath& path);

/// @brief Helper function to compute length-normalized scaling values along a JointPath.
/// @param scene The scene to compute configuration distances.
/// @param path The path to length-normalize.
/// @return A vector of scaling values between 0.0 and 1.0 at each point in the path if available,
/// otherwise an error.
tl::expected<Eigen::VectorXd, std::string> getNormalizedPathScaling(const Scene& scene,
                                                                    const JointPath& path);

/// @brief Helper function to get joint configurations from a path with normalized joint scalings.
/// @param scene The scene to use for joint interpolation.
/// @param path A JointPath of joint poses.
/// @param path_scalings The corresponding path scalings (between 0 and 1) to the provided path.
/// @param value A value between 0.0 and 1.0 pointing to the intermediate point along the path.
/// @return a pair containing the joint configuration at the scaled value along the path,
///         as well as the index corresponding to the next point along the path.
std::pair<Eigen::VectorXd, size_t>
getConfigurationFromNormalizedPathScaling(const Scene& scene, const JointPath& path,
                                          const Eigen::VectorXd& path_scalings, double value);

}  // namespace roboplan
