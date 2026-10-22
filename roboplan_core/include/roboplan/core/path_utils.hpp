#pragma once

#include <memory>
#include <optional>
#include <vector>

#include <Eigen/Dense>
#include <roboplan/core/constraints.hpp>
#include <roboplan/core/scene.hpp>
#include <tl/expected.hpp>

namespace roboplan {

class SceneContext;

/// @brief Computes the Cartesian path of a specified frame by interpolating sparse positions.
/// @note Runs forward kinematics on the Scene's shared scratch, so it is not safe to call
///   concurrently. Threaded callers should walk the configurations themselves through their own
///   SceneContext::forwardKinematics.
/// @param scene The scene to use.
/// @param q_start The starting joint positions.
/// @param q_end The ending joint positions.
/// @param frame_name The name of the frame whose path is computed.
/// @param max_step_size The maximum configuration distance step size for interpolation.
/// @return A list of 4x4 matrices corresponding to the poses of the frame along the path.
std::vector<Eigen::Matrix4d> computeFramePath(const Scene& scene, const Eigen::VectorXd& q_start,
                                              const Eigen::VectorXd& q_end,
                                              const std::string& frame_name,
                                              const double max_step_size);

/// @brief Computes the Cartesian path of a specified frame using a vector of provided points.
/// @param scene The scene to use.
/// @param q_vec A vector of joint positions.
/// @param frame_name The name of the frame whose path is computed.
/// @return A list of 4x4 matrices corresponding to the poses of the frame along the path.
std::vector<Eigen::Matrix4d> computeFramePath(const Scene& scene,
                                              const std::vector<Eigen::VectorXd>& q_vec,
                                              const std::string& frame_name);

/// @brief Resamples a dense sequence of group joint positions to `count` waypoints spaced
/// uniformly in configuration-space arc length (endpoints preserved).
/// @details Arc length and interpolation use Scene::configurationDistance and Scene::interpolate,
/// so continuous / free-rotating joints are measured and blended on their true manifold rather
/// than as flat Euclidean coordinates (which reads the wrap from +pi to -pi as a large jump and
/// does not subtract the SO(2) cos/sin pair of a continuous joint linearly).
///
/// This is useful when downstream consumers need evenly spaced knots: e.g. TOPP-RA parameterizes
/// its spline by waypoint index, so unevenly spaced waypoints (clustered where a tracker throttled
/// at corners) leave large gaps that the spline overshoots, deviating from the path.
/// @param positions Dense group joint positions, each of size `q_indices.size()`.
/// @param count Target number of (uniformly spaced) waypoints, capped at `positions.size()`.
/// @param scene Scene providing the manifold-aware distance/interpolation over the full model.
/// @param q_indices The full-configuration indices occupied by the group's coordinates.
/// @return The resampled group joint positions.
std::vector<Eigen::VectorXd> resampleUniform(const std::vector<Eigen::VectorXd>& positions,
                                             size_t count, const Scene& scene,
                                             const Eigen::VectorXi& q_indices);

/// @brief Checks collisions along a specified configuration space path.
/// @details All collision checks are answered by the caller-owned `scene_context`, so the traversal
///   does not contend on the Scene's shared collision scratch. Interpolation and distance use
///   `scene`, which only reads the immutable model and is therefore safe to share.
/// @param scene The scene to use for interpolating positions and computing distances.
/// @param scene_context The collision context whose scratch is used for all collision checks.
/// @param q_start The starting joint positions.
/// @param q_end The ending joint positions.
/// @param max_step_size The maximum configuration distance step size for interpolation.
/// @param bisection If true, visits the interior grid points coarse-to-fine by bisection instead
///   of a linear scan. The same points are checked, but collisions near the middle of the path are
///   found sooner, which helps in collision-dense environments.
/// @param check_endpoints If true, also checks the start and end points. Set to false when both
///   are already known to be collision-free (e.g. existing search-tree nodes) to skip those checks.
/// @return True if there are collisions, else false.
bool hasCollisionsAlongPath(const Scene& scene, const SceneContext& scene_context,
                            const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_end,
                            const double max_step_size, const bool bisection = false,
                            const bool check_endpoints = true);

/// @brief Checks collisions along a specified configuration space path using the Scene's own
/// scratch.
/// @details Every collision check goes through `scene.hasCollisions`, which avoids building a
///   SceneContext per call but is not safe to call concurrently with other queries on the same
///   Scene. Callers that need to parallelize should own a SceneContext and use the overload above.
/// @param scene The scene to use for interpolation, distances, and collision checks.
/// @param q_start The starting joint positions.
/// @param q_end The ending joint positions.
/// @param max_step_size The maximum configuration distance step size for interpolation.
/// @param bisection If true, visits the interior grid points coarse-to-fine by bisection instead
///   of a linear scan. The same points are checked, but collisions near the middle of the path are
///   found sooner, which helps in collision-dense environments.
/// @param check_endpoints If true, also checks the start and end points. Set to false when both
///   are already known to be collision-free (e.g. existing search-tree nodes) to skip those checks.
/// @return True if there are collisions, else false.
bool hasCollisionsAlongPath(const Scene& scene, const Eigen::VectorXd& q_start,
                            const Eigen::VectorXd& q_end, const double max_step_size,
                            const bool bisection = false, const bool check_endpoints = true);

/// @brief Computes the total configuration-space length of a joint path.
/// @details Sums the Scene's configuration distance between consecutive waypoints. The path's
///   positions are group positions, so each is expanded to full joint positions before measuring.
/// @param scene The scene used to measure configuration distances.
/// @param group_name The joint group the path was planned for.
/// @param path The joint path to measure. Must contain at least two waypoints.
/// @return The total path length, or an error if the path has fewer than two points.
tl::expected<double, std::string>
computePathLength(const Scene& scene, const std::string& group_name, const JointPath& path);

/// @brief Options struct for path shortcutting.
struct PathShortcuttingOptions {
  /// @brief The joint group name to be used for path shortcutting.
  std::string group_name = "";

  /// @brief Maximum step size used in collision checking, and the minimum separable distance
  /// between points in a shortcut.
  double max_step_size = 0.05;

  /// @brief Maximum number of iterations of random sampling.
  unsigned int max_iters = 100;

  /// @brief Seed for the random generator. If < 0, a random seed is used.
  int seed = 0;

  /// @brief Stop early once this many consecutive iterations fail to apply a shortcut
  /// (i.e., the path has converged), instead of always running the full `max_iters`.
  /// A value of 0 disables early stopping.
  unsigned int max_convergence_iters = 20;

  /// @brief Cadence (in iterations) of the redundant-vertex removal pass that cleans up the
  /// micro-segments introduced by shortcutting. Must be greater than 0.
  unsigned int redundant_removal_iters = 20;

  /// @brief Options for the projection used when `shortcut` is given constraints.
  /// @details Its `path_step_size` is the distance between projections along a shortcut that
  /// cannot be connected by a straight line.
  ConstraintProjectorOptions constraint_projection = ConstraintProjectorOptions();

  /// @brief With constraints, the ratio of a section's length to the straight distance between
  /// its ends below which no projected shortcut is attempted, since one is unlikely to be shorter.
  double constrained_min_detour_ratio = 1.1;
};

/// @brief Shortcuts joint paths with random sampling and checking connections.
/// @details This implementation is based on section 3.5.3 of:
/// https://motion.cs.illinois.edu/RoboticSystems/MotionPlanningHigherDimensions.html
class PathShortcutter {
public:
  /// @brief Construct a new path shortcutter instance.
  /// @param scene The scene for checking connectability between joint positions.
  /// @param options A struct containing path shortcutting options.
  PathShortcutter(const std::shared_ptr<Scene> scene, const PathShortcuttingOptions& options);

  /// @brief Attempts to shortcut a specified path.
  /// @details Each iteration samples two configurations along the path and, if they connect
  ///   collision-free, splices in the straight connection. Corner-cutting shortcuts add new
  ///   interpolated vertices, so a redundant-vertex removal pass runs periodically (see
  ///   `redundant_removal_iters`) and once more at the end, collapsing vertices whose neighbors
  ///   became directly connectable.
  ///
  ///   With constraints, a straight connection must also satisfy them. If it does not, the
  ///   shortcut is instead walked with the CBiRRT constrained extension (Algorithm 3 of Berenson
  ///   et al., 2009), and kept only if it is shorter than the section of path it replaces.
  /// @param path The JointPath to try to shorten. With constraints, it must already satisfy them.
  /// @param constraints Constraints that every configuration on the shortened path must satisfy.
  /// @return The shortcutted JointPath, or a copy of `path` if no shortcut applies.
  JointPath shortcut(const JointPath& path,
                     const std::vector<std::shared_ptr<Constraint>>& constraints = {});

  /// @brief Computes configuration distances from the start to each pose in a path.
  /// @param path The JointPath to evaluate.
  /// @return The cumulative distance at each waypoint (the first is 0), or an error if the path
  /// has fewer than two points.
  tl::expected<Eigen::VectorXd, std::string> getPathLengths(const JointPath& path);

  /// @brief Gets the joint configuration at a given length along a path.
  /// @param path A JointPath of joint poses.
  /// @param path_lengths The cumulative lengths of the path, from `getPathLengths`.
  /// @param length The length along the path at which to get the configuration.
  /// @return A pair containing the full-model joint configuration at that length along the
  ///         path, as well as the index corresponding to the next point along the path.
  std::pair<Eigen::VectorXd, size_t>
  getConfigurationAtPathLength(const JointPath& path, const Eigen::VectorXd& path_lengths,
                               double length);

private:
  /// @brief Removes interior vertices whose neighbors are directly connectable.
  /// @details Sweeps the path and deletes any interior vertex whose preceding and following
  ///   vertices can be connected by a collision-free straight segment, repeating until a full
  ///   sweep removes nothing. The endpoints are never removed. This collapses the redundant,
  ///   nearly-collinear vertices left behind by corner-cutting shortcuts. Endpoint collision
  ///   checks are skipped because every vertex is already an existing (collision-free) path node.
  /// @param path_configs The path configurations to prune in place.
  /// @param scene_context The collision context whose scratch backs the connection checks.
  /// @param projector If set, connections must also satisfy its constraints.
  /// @return The number of vertices removed.
  size_t removeRedundantVertices(std::vector<Eigen::VectorXd>& path_configs,
                                 const SceneContext& scene_context,
                                 std::optional<ConstraintProjector>& projector);

  /// @brief Connects two configurations along the constraints with the CBiRRT extension.
  /// @details Steps from `q_start` toward `q_end`, projecting each step onto the constraints, and
  ///   fails if a step does not get closer, collides, leaves the constraints, or the walk grows
  ///   longer than `max_length`.
  /// @param q_start The starting joint positions, which satisfy the constraints.
  /// @param q_end The ending joint positions, which satisfy the constraints.
  /// @param max_length The walk length at which to give up.
  /// @param scene_context The collision context whose scratch backs the connection checks.
  /// @param projector The projector for the constraints.
  /// @return The group joint positions after `q_start`, ending at `q_end`, or nullopt on failure.
  std::optional<std::vector<Eigen::VectorXd>> constrainedConnect(const Eigen::VectorXd& q_start,
                                                                 const Eigen::VectorXd& q_end,
                                                                 double max_length,
                                                                 const SceneContext& scene_context,
                                                                 ConstraintProjector& projector);

  /// @brief A pointer to the scene.
  std::shared_ptr<Scene> scene_;

  /// @brief The path shortcutting options.
  PathShortcuttingOptions options_;

  /// @brief The joint group info for the path shortcutter.
  JointGroupInfo joint_group_info_;

  /// @brief The full joint position vector for the scene (to prevent multiple allocations).
  Eigen::VectorXd q_full_;
};

}  // namespace roboplan
