#pragma once

#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>

namespace roboplan {

/// @brief A six-dimensional task space vector, ordered [x, y, z, roll, pitch, yaw].
/// @details Translations are in meters and rotations in radians. The rotation triplet is the
/// roll-pitch-yaw decomposition R = Rz(yaw) * Ry(pitch) * Rx(roll), to match Pinocchio.
using TaskSpaceVector = Eigen::Matrix<double, 6, 1>;

/// @brief Base class for kinematic constraints that planners enforce by projection.
/// @details A constraint reports a residual that is zero when it is satisfied and otherwise points
/// along the shortest way back into the allowed set, together with the Jacobian of that residual
/// with respect to a planning group's velocities.
///
/// @note Implementations are expected to own private kinematics scratch so that evaluating a
/// constraint does not disturb the Scene's shared data. That scratch is written on every
/// evaluation, so a single constraint instance must not be evaluated from multiple threads at once.
class Constraint {
public:
  virtual ~Constraint() = default;

  /// @brief The number of residual coordinates this constraint contributes.
  virtual size_t dimension() const = 0;

  /// @brief The joint group whose degrees of freedom a projection is allowed to move.
  virtual const std::string& getGroupName() const = 0;

  /// @brief Computes the constraint residual at a configuration.
  /// @details Each coordinate is zero where the constraint is satisfied, and otherwise the signed
  /// amount by which it overshoots the nearest bound.
  /// @param q The full (model-sized) joint positions to evaluate.
  /// @return The residual vector, of size `dimension()`.
  virtual Eigen::VectorXd computeError(const Eigen::VectorXd& q) = 0;

  /// @brief Computes the constraint residual and its Jacobian at a configuration.
  /// @details The Jacobian maps the group joint velocities to the rate of change of the residual
  /// coordinates, so a projection step can solve for the joint motion that cancels a violation.
  /// @param q The full (model-sized) joint positions to evaluate.
  /// @param error Output residual, pre-sized to `dimension()`.
  /// @param jacobian Output Jacobian, pre-sized to `dimension()` rows by the group's velocity DOFs.
  virtual void computeErrorAndJacobian(const Eigen::VectorXd& q, Eigen::Ref<Eigen::VectorXd> error,
                                       Eigen::Ref<Eigen::MatrixXd> jacobian) = 0;

  /// @brief Evaluates an already-computed residual against this constraint's tolerances.
  /// @details This lets a projection loop reuse the residual it just computed instead of running
  /// forward kinematics a second time to ask whether it is done.
  /// @param error A residual of size `dimension()`, as produced by `computeError`.
  /// @param tolerance_scale Scales every tolerance before comparing. A projection passes a value
  /// below one to land well inside the tolerance rather than exactly on its boundary, which leaves
  /// room for the interpolation between two projected configurations to stay inside it too.
  /// @return True if the residual is within the scaled tolerances.
  virtual bool satisfiesResidual(const Eigen::Ref<const Eigen::VectorXd>& error,
                                 double tolerance_scale = 1.0) const = 0;

  /// @brief Checks whether a configuration satisfies this constraint.
  /// @param q The full (model-sized) joint positions to check.
  /// @return True if the configuration is within the constraint's tolerances.
  bool satisfies(const Eigen::VectorXd& q) { return satisfiesResidual(computeError(q)); }
};

/// @brief Constrains the pose of a robot frame to a box of allowed task space displacements.
/// @details The pose of `frame_name` is measured relative to a *region frame*, placed by `tform`
/// relative to a reference frame (the world, unless `reference_frame` names one). The displacement
/// is expressed as [x, y, z, roll, pitch, yaw] in that region frame, and each coordinate is given
/// a lower and an upper bound. Coordinates left at their default of +/- infinity are unconstrained,
/// coordinates whose bounds are equal are pinned, and anything in between carves out a volume the
/// frame may move within. This is the Task Space Region of "Manipulation Planning on Constraint
/// Manifolds" (Berenson et al., 2009), sections 4a and 4b.
/// https://personalrobotics.cs.washington.edu/publications/berenson2009cbirrt.pdf
///
/// Leaving one rotation coordinate free while bounding the other two is the usual way to express an
/// axis constraint. Bounding roll and pitch to +/- theta with yaw free, for instance, keeps the
/// frame's z axis within acos(cos^2(theta)) of the frame's z axis while allowing spin about it.
///
/// @note The roll-pitch-yaw parameterization is singular at a pitch of +/- pi/2 relative to the
/// region frame. To avoid singularities, choose a region frame near the desired orientation.
class PoseConstraint : public Constraint {
public:
  /// @brief Constructor.
  /// @param scene A pointer to the scene the constraint is evaluated against.
  /// @param group_name The joint group whose degrees of freedom a projection may move.
  /// @param frame_name The name of the robot frame to constrain.
  /// @param lower_bounds Lower [x, y, z, roll, pitch, yaw] bounds of the frame's displacement in
  /// the region frame. Defaults to negative infinity, i.e. unconstrained.
  /// @param upper_bounds Upper [x, y, z, roll, pitch, yaw] bounds of the frame's displacement in
  /// the region frame. Defaults to positive infinity, i.e. unconstrained.
  /// @param tform The pose of the region frame relative to `reference_frame`.
  /// @param reference_frame The frame that `tform` is expressed in. Empty means the world frame.
  /// @param position_tolerance The position residual norm, in meters, that counts as satisfied.
  /// @param orientation_tolerance The orientation residual norm, in radians, that counts as
  /// satisfied.
  /// @note The tolerances are how far outside the bounds a configuration may sit and still count
  /// as satisfying the constraint, so they must be loose enough to cover the drift between two
  /// projected configurations a planner connects. That drift grows with the square of the
  /// planner's step, so the defaults are chosen for ConstraintProjectorOptions::path_step_size at
  /// its own default; tighten them alongside a smaller step, and loosen them for a larger one.
  PoseConstraint(const std::shared_ptr<Scene> scene, const std::string& group_name,
                 const std::string& frame_name,
                 const TaskSpaceVector& lower_bounds =
                     TaskSpaceVector::Constant(-std::numeric_limits<double>::infinity()),
                 const TaskSpaceVector& upper_bounds =
                     TaskSpaceVector::Constant(std::numeric_limits<double>::infinity()),
                 const Eigen::Matrix4d& tform = Eigen::Matrix4d::Identity(),
                 const std::string& reference_frame = "", double position_tolerance = 1.0e-3,
                 double orientation_tolerance = 5.0e-3);

  size_t dimension() const override { return 6; }

  const std::string& getGroupName() const override { return group_name_; }

  Eigen::VectorXd computeError(const Eigen::VectorXd& q) override;

  void computeErrorAndJacobian(const Eigen::VectorXd& q, Eigen::Ref<Eigen::VectorXd> error,
                               Eigen::Ref<Eigen::MatrixXd> jacobian) override;

  bool satisfiesResidual(const Eigen::Ref<const Eigen::VectorXd>& error,
                         double tolerance_scale = 1.0) const override;

  /// @brief Computes the constrained frame's displacement from the region frame.
  /// @details Unlike `computeError`, this reports where the frame actually is rather than how far
  /// outside the bounds it is, which is what you want when inspecting or plotting a path.
  /// @param q The full (model-sized) joint positions to evaluate.
  /// @return The [x, y, z, roll, pitch, yaw] displacement in the region frame.
  TaskSpaceVector computeDisplacement(const Eigen::VectorXd& q);

  /// @brief Gets the pose of the region frame relative to the reference frame.
  Eigen::Matrix4d getTransform() const { return region_tform_.toHomogeneousMatrix(); }

  /// @brief Gets the position residual norm, in meters, below which the constraint is satisfied.
  double getPositionTolerance() const { return position_tolerance_; }

  /// @brief Gets the orientation residual norm, in radians, below which the constraint is
  /// satisfied.
  double getOrientationTolerance() const { return orientation_tolerance_; }

  /// @brief Gets the name of the constrained frame.
  const std::string& getFrameName() const { return frame_name_; }

  /// @brief Gets the name of the frame the region transform is expressed in ("" means world).
  const std::string& getReferenceFrame() const { return reference_frame_; }

  /// @brief Gets the lower bounds on the frame's task space displacement.
  const TaskSpaceVector& getLowerBounds() const { return lower_bounds_; }

  /// @brief Gets the upper bounds on the frame's task space displacement.
  const TaskSpaceVector& getUpperBounds() const { return upper_bounds_; }

private:
  /// @brief Refreshes the constrained frame's displacement, and optionally its Jacobian.
  /// @details On return, `displacement_` holds the frame's [x, y, z, roll, pitch, yaw] pose in the
  /// region frame. When `with_jacobian` is true, `displacement_jacobian_` holds the derivative of
  /// those six coordinates with respect to the group's joint velocities.
  /// @param q The full (model-sized) joint positions to evaluate.
  /// @param with_jacobian Whether to also compute `displacement_jacobian_`.
  void updateKinematics(const Eigen::VectorXd& q, bool with_jacobian);

  /// @brief Computes how far a displacement lies outside the constraint's bounds.
  /// @param displacement The task space displacement to test.
  /// @return Zero for coordinates within bounds, else the signed overshoot past the nearest bound.
  TaskSpaceVector boundsResidual(const TaskSpaceVector& displacement) const;

  /// @brief A pointer to the scene.
  std::shared_ptr<Scene> scene_;

  /// @brief The joint group whose degrees of freedom a projection may move.
  std::string group_name_;

  /// @brief The name of the constrained frame.
  std::string frame_name_;

  /// @brief The name of the frame the region transform is expressed in ("" means world).
  std::string reference_frame_;

  /// @brief The pose of the region frame relative to the reference frame.
  pinocchio::SE3 region_tform_;

  /// @brief Lower bounds on the frame's task space displacement.
  TaskSpaceVector lower_bounds_;

  /// @brief Upper bounds on the frame's task space displacement.
  TaskSpaceVector upper_bounds_;

  /// @brief The position residual norm, in meters, below which the constraint is satisfied.
  double position_tolerance_;

  /// @brief The orientation residual norm, in radians, below which the constraint is satisfied.
  double orientation_tolerance_;

  /// @brief The joint group info for the constraint.
  JointGroupInfo joint_group_info_;

  /// @brief The Pinocchio frame ID of the constrained frame.
  pinocchio::FrameIndex frame_id_;

  /// @brief The Pinocchio frame ID of the reference frame, if one was named.
  std::optional<pinocchio::FrameIndex> reference_frame_id_;

  /// @brief Private Pinocchio data, so evaluations do not touch the Scene's shared scratch.
  pinocchio::Data data_;

  /// @brief The frame's task space displacement, at the last evaluation.
  TaskSpaceVector displacement_;

  /// @brief The derivative of `displacement_` with respect to the group's joint velocities.
  Eigen::MatrixXd displacement_jacobian_;

  /// @brief The full-model frame Jacobian (for allocating memory once).
  Eigen::MatrixXd world_jacobian_;

  /// @brief The full-model reference frame Jacobian (for allocating memory once).
  Eigen::MatrixXd reference_jacobian_;
};

/// @brief Options struct for constraint projection.
struct ConstraintProjectorOptions {
  /// @brief The configuration-space step size taken between projections.
  /// @details Projection is a local operation, so a step that is too large lands somewhere it
  /// cannot recover from. Smaller values track the constraints more faithfully and fail less
  /// often, at the cost of more projections per unit of progress.
  double path_step_size = 0.1;

  /// @brief The maximum number of projection iterations before giving up.
  size_t max_iters = 50;

  /// @brief The fraction of each computed correction to apply per projection iteration.
  /// @details The default takes full Gauss-Newton steps, which converge in a handful of iterations
  /// for the small violations a planner produces. Lower it if a projection oscillates instead of
  /// converging, which can happen when constraints fight each other near a kinematic singularity.
  double correction_step_size = 1.0;

  /// @brief Damping value for the Jacobian pseudoinverse.
  double damping = 1.0e-6;

  /// @brief The fraction of each constraint's tolerance the projection converges to.
  /// @details Stopping the moment the residual first dips under the tolerance leaves a
  /// configuration sitting exactly on the tolerance boundary, so interpolating between two such
  /// configurations immediately falls outside it. Converging well inside instead leaves the
  /// headroom that a planner's edges need. Must be in (0, 1].
  double convergence_ratio = 0.1;
};

/// @brief Projects joint configurations onto the intersection of a set of constraints.
/// @details Each iteration stacks the violated residual coordinates of every constraint into one
/// task and takes a damped least-squares step that cancels them, moving only the planning group's
/// degrees of freedom and clamping to joint limits.
class ConstraintProjector {
public:
  /// @brief Constructor.
  /// @param scene A pointer to the scene to project against.
  /// @param group_name The joint group whose degrees of freedom the projection may move.
  /// @param constraints The constraints to project onto. May be empty, in which case every
  /// configuration satisfies the (vacuous) constraint set and projection is a no-op.
  /// @param options A struct containing constraint projection options.
  ConstraintProjector(const std::shared_ptr<Scene> scene, const std::string& group_name,
                      const std::vector<std::shared_ptr<Constraint>>& constraints,
                      const ConstraintProjectorOptions& options = ConstraintProjectorOptions());

  /// @brief Checks whether a configuration satisfies every constraint.
  /// @param q The full (model-sized) joint positions to check.
  /// @return True if all constraints are satisfied within their tolerances.
  bool satisfies(const Eigen::VectorXd& q);

  /// @brief Projects a configuration onto the constraints, in place.
  /// @param q The full (model-sized) joint positions to project. Modified in place; on failure it
  /// holds the last iterate, which still violates at least one constraint.
  /// @return True if the projection converged within the iteration budget.
  bool project(Eigen::VectorXd& q);

  /// @brief Checks whether a straight-line configuration-space path stays inside the constraints.
  /// @details Samples the segment at the same resolution used for collision checking, so it shares
  /// that check's caveat: a violation confined to a gap between samples is not detected.
  /// @param q_start The starting joint positions.
  /// @param q_end The ending joint positions.
  /// @param max_step_size The maximum configuration distance step size for interpolation.
  /// @param check_endpoints If true, also checks the two endpoints. Callers whose endpoints are
  /// already known to satisfy the constraints (e.g. existing nodes of a search tree) can skip them.
  /// @return True if every sampled configuration satisfies all constraints.
  bool satisfiesAlongPath(const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_end,
                          double max_step_size, bool check_endpoints = true);

  /// @brief Whether the projector holds no constraints, and is therefore a no-op.
  bool empty() const { return constraints_.empty(); }

private:
  /// @brief A pointer to the scene.
  std::shared_ptr<Scene> scene_;

  /// @brief The constraints to project onto.
  std::vector<std::shared_ptr<Constraint>> constraints_;

  /// @brief The struct containing constraint projection options.
  ConstraintProjectorOptions options_;

  /// @brief The joint group info for the projector.
  JointGroupInfo joint_group_info_;

  /// @brief Lower position limits for the joint group, aligned with `q_indices`.
  Eigen::VectorXd lower_position_limits_;

  /// @brief Upper position limits for the joint group, aligned with `q_indices`.
  Eigen::VectorXd upper_position_limits_;

  /// @brief Per-constraint residuals (for allocating memory once).
  std::vector<Eigen::VectorXd> constraint_errors_;

  /// @brief Per-constraint Jacobians (for allocating memory once).
  std::vector<Eigen::MatrixXd> constraint_jacobians_;

  /// @brief The stacked residuals of the currently violated coordinates.
  Eigen::VectorXd active_error_;

  /// @brief The stacked Jacobian rows of the currently violated coordinates.
  Eigen::MatrixXd active_jacobian_;

  /// @brief The Jacobian times Jacobian transpose (for allocating memory once).
  Eigen::MatrixXd jjt_;

  /// @brief The joint group's velocity correction (for allocating memory once).
  Eigen::VectorXd group_velocity_;

  /// @brief The full joint velocity vector for integrating (for allocating memory once).
  Eigen::VectorXd velocity_;
};

}  // namespace roboplan
