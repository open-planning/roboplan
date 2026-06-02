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
  Adaptive,
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
  ///   - `SplineFittingMode::Adaptive`: Uses the cubic mode but iteratively collision checks
  ///     and adds intermediate points if it finds collisions, up to a maximum number of iterations.
  ///     If the path is not collision-free after the maximum iterations, falls back to Hermite
  ///     mode. Refer to Section 3.5 of https://groups.csail.mit.edu/rrg/papers/Richter_ISRR13.pdf
  ///     for more details on this approach.
  /// @param velocity_scale A scaling factor (between 0 and 1) for velocity limits.
  /// @param acceleration_scale A scaling factor (between 0 and 1) for acceleration limits.
  /// @param max_adaptive_iterations Maximum number of adaptive iterations, if adaptive mode is
  /// enabled.
  /// @param max_adaptive_step_size If adaptive mode is enabled, this is the maximum joint
  /// configuration
  ///   step size to sample generated splines for collision checking.
  /// @return A time-parameterized joint trajectory.
  tl::expected<JointTrajectory, std::string>
  generate(const JointPath& path, const double dt,
           const SplineFittingMode mode = SplineFittingMode::Hermite,
           const double velocity_scale = 1.0, const double acceleration_scale = 1.0,
           const int max_adaptive_iterations = 10, const double max_adaptive_step_size = 0.05);

private:
  /// @brief Helper function to convert the raw joint path to TOPP-RA compatible position vectors.
  /// @details If the joint group contains non-standard (Lie-group) joints -- continuous, planar, or
  /// floating -- each such joint is represented by a single normalized arc-length coordinate
  /// instead of its individual degrees of freedom. The normalized coordinate accumulates, per path
  /// segment, the minimum time to traverse that segment subject to the joint's velocity limits,
  /// computed from per-component configuration distances. Because the joint moves along the scene's
  /// geodesic (SE(2)/SE(3) screw, shortest rotation) the spline tracks the true Lie-group motion
  /// exactly without dense resampling.
  /// @param path The joint path to convert.
  /// @return The TOPP-RA compatible position vectors if successful, else a string describing the
  /// error.
  tl::expected<toppra::Vectors, std::string> getPathPositionVectors(const JointPath& path);

  /// @brief Returns whether a joint type is "non-standard" (a Lie group where the number of
  /// position DOFs and velocity DOFs differ, so a Euclidean spline would not track it).
  static bool isNonstandardJoint(JointType type);

  /// @brief Finds the path segment containing a normalized coordinate value and the fraction
  /// within.
  /// @param cumulative The cumulative normalized coordinate at each waypoint (monotonic
  /// increasing).
  /// @param value The normalized coordinate value to look up.
  /// @return A pair of {segment index (clamped), fraction within segment [0, 1]}.
  static std::pair<size_t, double> locateSegment(const std::vector<double>& cumulative,
                                                 double value);

  /// @brief Reconstructs the expanded group positions from a normalized position vector.
  /// @details Standard joints are copied through; non-standard joints are reconstructed onto their
  /// geodesic via `Scene::interpolate` at the fraction implied by the normalized coordinate.
  /// @param q_norm The normalized position vector.
  /// @return The expanded group position vector (the representation used by
  /// `toFullJointPositions`).
  Eigen::VectorXd normalizedToGroupPositions(const Eigen::VectorXd& q_norm) const;

  /// @brief Reconstructs collapsed-space velocities (and accelerations) from a normalized solution.
  /// @details Standard joints copy their normalized derivatives through. For non-standard joints,
  /// the geodesic's body-frame tangent (the Lie group log, constant along the segment) is scaled by
  /// the normalized coordinate's first/second time derivatives via the chain rule. Outputs are in
  /// the same collapsed velocity representation as the non-planar code path.
  /// @param q_norm The normalized positions at the sample.
  /// @param dq_norm The normalized first time derivatives at the sample.
  /// @param ddq_norm The normalized second time derivatives at the sample.
  /// @param[out] velocities The reconstructed velocity vector.
  /// @param[out] accelerations The reconstructed acceleration vector.
  void normalizedToVelocitiesAndAccelerations(const Eigen::VectorXd& q_norm,
                                              const Eigen::VectorXd& dq_norm,
                                              const Eigen::VectorXd& ddq_norm,
                                              Eigen::VectorXd& velocities,
                                              Eigen::VectorXd& accelerations) const;

  /// @brief Helper function to extract a natural cubic spline from a joint path.
  /// @details This defines zero velocity and acceleration at the endpoints only, meaning that
  /// intermediate waypoints are passed through smoothly, but could deviate from the original path
  /// and therefore lead to collisions.
  /// @param path_pos_vecs The joint path position vectors.
  /// @return The resulting cubic spline.
  std::shared_ptr<toppra::PiecewisePolyPath>
  generateCubicSpline(const toppra::Vectors& path_pos_vecs);

  /// @brief Helper function to extract a cubic Hermite spline from a joint path.
  /// @details This enforces zero velocity and acceleration at all waypoints, meaning the desired
  /// path is exactly adhered to. If the path was checked for collisions, this spline is also safe.
  /// @param path_pos_vecs The joint path position vectors.
  /// @return The resulting cubic Hermite spline.
  std::shared_ptr<toppra::PiecewisePolyPath>
  generateCubicHermiteSpline(const toppra::Vectors& path_pos_vecs);

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

  /// @brief Position indices of the joint group within the full model configuration.
  Eigen::VectorXi q_indices_;

  /// @brief Precomputed placement of one (non-mimic) joint across the several representations used
  /// while parameterizing. Computed once in the constructor so the position, reconstruction, and
  /// velocity routines can share a single iteration instead of each re-querying joint info and
  /// re-deriving the parallel column offsets.
  struct JointLayout {
    std::string joint_name;    ///< The joint name (for re-querying limits).
    JointType type;            ///< The joint type.
    bool is_nonstandard;       ///< Whether the joint collapses to a normalized arc-length coord.
    size_t num_position_dofs;  ///< Number of position DOFs in the expanded representation.
    size_t num_velocity_dofs;  ///< Number of velocity DOFs in the expanded representation.
    size_t normalized_col;     ///< First column in the normalized (TOPP-RA) representation.
    size_t collapsed_col;      ///< First column in the collapsed representation.
    size_t velocity_col;       ///< First column in the velocity/acceleration representation.
    size_t expanded_col;       ///< First column in the expanded group representation.
    Eigen::Index q_offset;     ///< Offset of the joint within the full model configuration.
    Eigen::Index v_offset;     ///< Offset of the joint within the full model tangent (velocity).
    int nonstandard_idx;       ///< Index into the cumulative-coordinate list, or -1 if standard.
  };

  /// @brief The layout of every non-mimic joint in the group, in joint order.
  std::vector<JointLayout> joint_layouts_;

  /// @brief Whether the joint group contains any non-standard (continuous/planar/floating) joints.
  /// @details When true, each such joint is represented in the TOPP-RA problem by a single
  /// normalized arc-length coordinate rather than its individual degrees of freedom, so the spline
  /// tracks the Lie-group motion exactly without dense resampling.
  bool has_nonstandard_joints_{false};

  /// @brief Dimension of the normalized representation used for the TOPP-RA problem.
  /// @details Equals the number of velocity DOFs in the group, but with each non-standard joint
  /// contributing a single normalized coordinate.
  size_t norm_dim_{0};

  /// @brief Velocity limit vectors in the normalized representation.
  toppra::Vector norm_vel_lower_limits_;
  toppra::Vector norm_vel_upper_limits_;

  /// @brief Acceleration limit vectors in the normalized representation.
  toppra::Vector norm_acc_lower_limits_;
  toppra::Vector norm_acc_upper_limits_;

  /// @brief Full model configurations at each path waypoint, used to reconstruct geodesics.
  /// @details Populated by getPathPositionVectors for the non-standard code path.
  std::vector<Eigen::VectorXd> full_waypoints_;

  /// @brief Cumulative normalized coordinate at each waypoint, one entry per non-standard joint.
  /// @details Indexed by the order in which non-standard joints appear in the group. Each inner
  /// vector is monotonically increasing and has one element per waypoint.
  std::vector<std::vector<double>> nonstandard_joint_cumulative_coordinates_;
};

}  // namespace roboplan
