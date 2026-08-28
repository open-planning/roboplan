#pragma once

#include <string>

#include <Eigen/Dense>

namespace roboplan {

// A constraint is a hard bound described in terms the user understands (per-DoF bounds, frame
// names, tolerances). Each is attached with `TrajectoryOptimizer::addConstraint(spec, window)`
// over a window of stages. Unlike costs, constraints are fixed at build time — rebuild via
// `resetProblem()` to change them.
//
// Bound vectors are in the optimizer's REDUCED-group layout (size nq for configurations, nv for
// velocities / torques — the same layout as `setInitialState`). A user-supplied bound only ever
// tightens the model's own physical limit (never loosens it); leaving a bound empty uses the model
// default.

/// @brief Box limit on the reduced-group configuration q.
/// @details Both bounds default to the model's position limits when left empty. Not supported for
/// continuous (unbounded-revolute) joints, whose configuration is a cos/sin pair — use a group
/// without them.
struct PositionLimit {
  /// @brief Lower configuration bound (size nq). Empty ⇒ the model's lower position limit.
  Eigen::VectorXd q_min;

  /// @brief Upper configuration bound (size nq). Empty ⇒ the model's upper position limit.
  Eigen::VectorXd q_max;
};

/// @brief Symmetric box limit on the reduced-group velocity v, giving the box [-v_max, +v_max].
/// @details Defaults to the model's velocity limits (which may be asymmetric) when left empty.
struct VelocityLimit {
  /// @brief Symmetric per-DoF velocity bound (size nv). Empty ⇒ the model's velocity limits.
  Eigen::VectorXd v_max;
};

/// @brief Symmetric box limit on the control (joint torque) u, giving the box [-tau_max, +tau_max].
/// @details Defaults to the reduced model's effort limits when left empty. A DoF whose model effort
/// limit is non-finite or zero is treated as unbounded. Not allowed on the terminal window (the
/// terminal node has no control).
struct TorqueLimit {
  /// @brief Symmetric per-DoF torque bound (size nv). Empty ⇒ the model's effort limits.
  Eigen::VectorXd tau_max;
};

/// @brief Keep the robot's articulated links clear of each other over a window of stages.
/// @details Constraints the `n_pairs` closest self-collision pairs (both geometries on articulated
/// robot links) at the initial configuration, requiring signed distance > `d_min`. The pair set is
/// fixed at the initial configuration and enforced at the stage knots only.
struct SelfCollisionConstraint {
  /// @brief Number of closest self-collision pairs to constrain. `<= 0` tracks every candidate
  /// pair.
  int n_pairs = 0;

  /// @brief Minimum allowed signed distance between the pair's geometries, in metres.
  double d_min = 0.02;
};

/// @brief Keep the robot's articulated links clear of static geometry over a window.
/// @details Same mechanism as `SelfCollisionConstraint`, but for robot-vs-static pairs (exactly one
/// geometry on the fixed world, e.g. the base or an environment obstacle).
struct CollisionConstraint {
  /// @brief Number of closest robot-vs-static pairs to constrain. `<= 0` tracks every candidate
  /// pair.
  int n_pairs = 0;

  /// @brief Minimum allowed signed distance between the pair's geometries, in metres.
  double d_min = 0.02;
};

/// @brief Hard bound on a frame's SE3 placement error from a target pose.
/// @details Bounds the 6-D log6 error, ordered [translation(3); rotation-log(3)]: translation
/// within ±`tol_pos` (metres) and rotation within ±`tol_rot` (radians) per axis. Commonly attached
/// to the Terminal window as a reach/grasp constraint.
struct FramePoseConstraint {
  /// @brief Name of the frame whose pose is constrained.
  std::string frame;

  /// @brief Target pose as a 4x4 homogeneous transform (world <- frame).
  Eigen::Matrix4d target = Eigen::Matrix4d::Identity();

  /// @brief Allowed translation error half-width, metres (applied to each of x, y, z). Nonnegative.
  double tol_pos = 0.0;

  /// @brief Allowed rotation-log error half-width, radians (applied to each axis). Nonnegative.
  double tol_rot = 0.0;
};

}  // namespace roboplan
