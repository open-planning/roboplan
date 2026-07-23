#pragma once

#include <string>

#include <Eigen/Dense>

namespace roboplan {

// Hard-constraint specifications (design §4.4). Like the cost specs (costs.hpp), these are plain
// value structs describing a constraint in terms the USER understands (per-DoF bounds, frame names,
// tolerances). They carry no aligator or pinocchio types and no reference to the optimizer's
// internal reduced model: the residual + aligator ConstraintSet are built inside
// TrajectoryOptimizer::addConstraint, which owns the reduced model and reads the limit defaults
// from core (§4.4 "defaults from Scene::getPositionLimitVectors / getVelocityLimitVectors, not
// re-derived").
//
// Each is attached with `TrajectoryOptimizer::addConstraint(spec, window)` over a window of stages
// (§3.3). Unlike costs, constraints have no mutable-target handle — they are fixed at build time
// (rebuild via resetProblem() to change them).
//
// Bound vectors are in the optimizer's REDUCED-group layout (size nq for configurations, nv for
// velocities / torques — the same layout as setInitialState). A user-supplied bound is clamped to
// the model's own physical limit per-DoF (it can only ever tighten, never loosen; maintainer
// decision, Prompt 7). Leaving a bound empty uses the model default verbatim.

/// @brief Box limit on the reduced-group configuration q over a window of stages (design §4.4).
/// @details Maps to a state-slice residual (StateError sliced to the q-tangent rows) + a box
/// constraint set. Both bounds default to the model's position limits
/// (Scene::getPositionLimitVectors) when left empty; a supplied bound is intersected with the
/// model's per-DoF. Bounds are expressed in the state manifold's tangent frame; for revolute and
/// prismatic joints this equals the raw configuration box, so it is exact. Continuous
/// (unbounded-revolute) joints — whose configuration is a cos/sin pair — are not supported by this
/// limit; use a group without them.
struct PositionLimit {
  /// @brief Lower configuration bound (size nq). Empty ⇒ the model's lower position limit.
  Eigen::VectorXd q_min;

  /// @brief Upper configuration bound (size nq). Empty ⇒ the model's upper position limit.
  Eigen::VectorXd q_max;
};

/// @brief Symmetric box limit on the reduced-group velocity v over a window of stages (design
/// §4.4).
/// @details Maps to a state-slice residual (StateError sliced to the v rows) + a box constraint set
/// [-v_max, +v_max]. Defaults to the model's velocity limits (Scene::getVelocityLimitVectors) when
/// left empty; a supplied bound is intersected with the model's per-DoF.
struct VelocityLimit {
  /// @brief Symmetric per-DoF velocity bound (size nv), giving the box [-v_max, +v_max]. Empty ⇒
  /// the model's velocity limits (which may be asymmetric).
  Eigen::VectorXd v_max;
};

/// @brief Symmetric box limit on the control (joint torque) u over a window of stages (design
/// §4.4).
/// @details Maps to a control residual (value = u) + a box constraint set [-tau_max, +tau_max].
/// Defaults to the reduced model's effort limits when left empty (design §4.4 "model effort
/// limits"); a supplied bound is intersected with the model's per-DoF. A DoF whose model effort
/// limit is non-finite or zero is treated as unbounded (±inf), never clamped to zero torque
/// (maintainer decision, Prompt 7). Illegal on the terminal window (the terminal node has no
/// control).
struct TorqueLimit {
  /// @brief Symmetric per-DoF torque bound (size nv), giving the box [-tau_max, +tau_max]. Empty ⇒
  /// the model's effort limits.
  Eigen::VectorXd tau_max;
};

/// @brief Keep the robot's articulated links clear of each other over a window of stages (§4.4,
/// §5).
/// @details Expands to one collision residual + box constraint (signed distance ≥ `d_min`)
/// per tracked collision pair. Self-collision pairs (both geometries on articulated robot links)
/// are taken from the reduced model's collision geometry; the `n_pairs` closest at the optimizer's
/// initial configuration are tracked (fixed set — the reuse path does not re-select per iteration,
/// so raise `n_pairs` / simplify geometry if a pair only becomes close mid-trajectory, §5 caveat).
/// Enforced at stage knots only; inter-stage clearance is not enforced (§5 caveat).
struct SelfCollisionConstraint {
  /// @brief Number of closest self-collision pairs to constrain. `<= 0` tracks every candidate
  /// pair.
  int n_pairs = 0;

  /// @brief Minimum allowed signed distance between the pair's geometries, in metres.
  double d_min = 0.02;
};

/// @brief Keep the robot's articulated links clear of static geometry over a window (§4.4, §5).
/// @details Same mechanism as SelfCollisionConstraint, but tracks robot-vs-static pairs: pairs
/// where exactly one geometry is attached to the world (the fixed base or an environment obstacle).
/// The `n_pairs` closest at the initial configuration are tracked (fixed set; §5 caveat as above).
struct CollisionConstraint {
  /// @brief Number of closest robot-vs-static pairs to constrain. `<= 0` tracks every candidate
  /// pair.
  int n_pairs = 0;

  /// @brief Minimum allowed signed distance between the pair's geometries, in metres.
  double d_min = 0.02;
};

/// @brief Hard bound on a frame's SE3 placement error from a target pose over a window (design
/// §4.4).
/// @details Maps to aligator FramePlacementResidual + a box constraint set on the 6-D log6 error,
/// ordered [translation(3); rotation-log(3)]: translation error within ±`tol_pos` (metres) and
/// rotation-log error within ±`tol_rot` (radians), per axis. Commonly attached to the Terminal
/// window as a reach/grasp constraint.
struct FramePoseConstraint {
  /// @brief Name of the frame whose pose is constrained (resolved against the reduced model).
  std::string frame;

  /// @brief Target pose as a 4x4 homogeneous transform (world <- frame).
  Eigen::Matrix4d target = Eigen::Matrix4d::Identity();

  /// @brief Allowed translation error half-width, metres (applied to each of x, y, z). Nonnegative.
  double tol_pos = 0.0;

  /// @brief Allowed rotation-log error half-width, radians (applied to each axis). Nonnegative.
  double tol_rot = 0.0;
};

}  // namespace roboplan
