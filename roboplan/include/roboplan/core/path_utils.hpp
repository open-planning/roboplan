#pragma once

#include <vector>

#include <Eigen/Dense>
#include <roboplan/core/scene.hpp>
#include <tl/expected.hpp>

namespace roboplan {

class CollisionContext;

/// @brief Computes the Cartesian path of a specified frame by interpolating sparse positions.
/// @param scene The scene to use.
/// @param q_start The starting joint positions.
/// @param q_end The ending joint positions.
/// @param frame_name The name of the frame in which to compute the Cartesian path.
/// @param max_step_size The maximum configuration distance step size for interpolation.
/// @return A list of 4x4 matrices corresponding to the poses of the frame along the path.
std::vector<Eigen::Matrix4d> computeFramePath(const Scene& scene, const Eigen::VectorXd& q_start,
                                              const Eigen::VectorXd& q_end,
                                              const std::string& frame_name,
                                              const double max_step_size);

/// @brief Computes the Cartesian path of a specified frame using a vector of provided points.
/// @param scene The scene to use.
/// @param q_vec A vector of joint positions.
/// @param frame_name The name of the frame in which to compute the Cartesian path.
/// @return A list of 4x4 matrices corresponding to the poses of the frame along the path.
std::vector<Eigen::Matrix4d> computeFramePath(const Scene& scene,
                                              const std::vector<Eigen::VectorXd>& q_vec,
                                              const std::string& frame_name);

/// @brief Checks collisions along a specified configuration space path.
/// @details All collision checks are answered by the caller-owned `context`, so the traversal does
///   not contend on the Scene's shared collision scratch. Interpolation and distance use `scene`,
///   which only reads the immutable model and is therefore safe to share.
/// @param scene The scene to use for interpolating positions and computing distances.
/// @param collision_context The collision context whose scratch is used for all collision checks.
/// @param q_start The starting joint positions.
/// @param q_end The ending joint positions.
/// @param max_step_size The maximum configuration distance step size for interpolation.
/// @param bisection If True, visits the interior grid points in a coarse-to-fine bisection order
///   instead of a linear scan. This checks exactly the same minimal number of points as the linear
///   scan, but can find collisions faster in collision-dense environments since points near the
///   middle of the path are checked first.
/// @param check_endpoints If True, checks the start and end endpoints for collisions.
///   Callers that already know both endpoints are collision-free (e.g. they are existing nodes in a
///   search tree) can set this to False to skip redundant, expensive collision checks.
/// @return True if there are collisions, else false.
bool hasCollisionsAlongPath(const Scene& scene, const CollisionContext& collision_context,
                            const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_end,
                            const double max_step_size, const bool bisection = false,
                            const bool check_endpoints = true);

/// @brief Checks collisions along a specified configuration space path using the Scene's own
/// scratch.
/// @details This convenience overload answers every collision check via `scene.hasCollisions`,
/// which
///   uses the Scene's internal (shared) collision scratch. It avoids constructing a per-call
///   CollisionContext, but carries the same caveat as every other Scene collision query: it is not
///   safe to call concurrently with other queries on the same Scene. Callers that need to
///   parallelize should own a CollisionContext and use the overload above.
/// @param scene The scene to use for interpolation, distances, and collision checks.
/// @param q_start The starting joint positions.
/// @param q_end The ending joint positions.
/// @param max_step_size The maximum configuration distance step size for interpolation.
/// @param bisection If True, visits the interior grid points in a coarse-to-fine bisection order
///   instead of a linear scan. This checks exactly the same minimal number of points as the linear
///   scan, but can find collisions faster in collision-dense environments since points near the
///   middle of the path are checked first.
/// @param check_endpoints If True, checks the start and end endpoints for collisions.
///   Callers that already know both endpoints are collision-free (e.g. they are existing nodes in a
///   search tree) can set this to False to skip redundant, expensive collision checks.
/// @return True if there are collisions, else false.
bool hasCollisionsAlongPath(const Scene& scene, const Eigen::VectorXd& q_start,
                            const Eigen::VectorXd& q_end, const double max_step_size,
                            const bool bisection = false, const bool check_endpoints = true);

/// @brief Shortcuts joint paths with random sampling and checking connections.
/// @details This implementation is based on section 3.5.3 of:
/// https://motion.cs.illinois.edu/RoboticSystems/MotionPlanningHigherDimensions.html
class PathShortcutter {
public:
  /// @brief Construct a new path shortcutter instance.
  /// @param scene The scene for checking connectability between joint positions.
  /// @param group_name The name of the group to use for path shortcutting.
  PathShortcutter(const std::shared_ptr<Scene> scene, const std::string& group_name);

  /// @brief Attempts to shortcut a specified path.
  /// @param path The JointPath to try to shorten.
  /// @param max_step_size Maximum step size to use in collision checking, and the minimum
  /// separable distance between points in a shortcut.
  /// @param max_iters Maximum number of iterations of random sampling (default 100).
  /// @param seed Seed for the random generator, if < 0 then use a random seed (default -1).
  /// @return A shortcutted JointPath, if available.
  JointPath shortcut(const JointPath& path, double max_step_size, unsigned int max_iters = 100,
                     int seed = 0);

  /// @brief Computes configuration distances from the start to each pose in a path.
  /// @param path The JointPath to evaluate.
  /// @return A vector of incremental path distances, if there is sufficient data. Otherwise an
  /// error.
  tl::expected<Eigen::VectorXd, std::string> getPathLengths(const JointPath& path);

  /// @brief Computes length-normalized scaling values along a JointPath.
  /// @param path The path to length-normalize.
  /// @return A vector of scaling values between 0.0 and 1.0 at each point in the path if available,
  /// otherwise an error.
  tl::expected<Eigen::VectorXd, std::string> getNormalizedPathScaling(const JointPath& path);

  /// @brief Gets joint configurations from a path with normalized joint scalings.
  /// @param path A JointPath of joint poses.
  /// @param path_scalings The corresponding path scalings (between 0 and 1) to the provided path.
  /// @param value A value between 0.0 and 1.0 pointing to the intermediate point along the path.
  /// @return a pair containing the joint configuration at the scaled value along the path,
  ///         as well as the index corresponding to the next point along the path.
  std::pair<Eigen::VectorXd, size_t>
  getConfigurationFromNormalizedPathScaling(const JointPath& path,
                                            const Eigen::VectorXd& path_scalings, double value);

private:
  /// @brief A pointer to the scene.
  std::shared_ptr<Scene> scene_;

  /// @brief The joint group info for the path shortcutter.
  JointGroupInfo joint_group_info_;

  /// @brief The full joint position vector for the scene (to prevent multiple allocations).
  Eigen::VectorXd q_full_;
};

}  // namespace roboplan
