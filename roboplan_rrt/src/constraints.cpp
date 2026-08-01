#include <cmath>
#include <stdexcept>
#include <string>

#include <pinocchio/math/rpy.hpp>

#include <roboplan/core/math_utils.hpp>
#include <roboplan_rrt/constraints.hpp>

namespace roboplan {

PoseConstraint::PoseConstraint(const std::shared_ptr<Scene> scene, const std::string& group_name,
                               const std::string& frame_name, const TaskSpaceVector& lower_bounds,
                               const TaskSpaceVector& upper_bounds, const Eigen::Matrix4d& tform,
                               const std::string& reference_frame, double position_tolerance,
                               double orientation_tolerance)
    : scene_{scene}, group_name_{group_name}, frame_name_{frame_name},
      reference_frame_{reference_frame}, region_tform_{pinocchio::SE3(tform)},
      lower_bounds_{lower_bounds}, upper_bounds_{upper_bounds},
      position_tolerance_{position_tolerance}, orientation_tolerance_{orientation_tolerance} {

  // Validate the joint group.
  const auto maybe_joint_group_info = scene_->getJointGroupInfo(group_name);
  if (!maybe_joint_group_info) {
    throw std::runtime_error("Could not initialize pose constraint: " +
                             maybe_joint_group_info.error());
  }
  joint_group_info_ = maybe_joint_group_info.value();

  // Validate the constrained frame and, if named, the reference frame.
  const auto maybe_frame_id = scene_->getFrameId(frame_name);
  if (!maybe_frame_id) {
    throw std::runtime_error("Could not initialize pose constraint: " + maybe_frame_id.error());
  }
  frame_id_ = maybe_frame_id.value();

  if (!reference_frame.empty()) {
    const auto maybe_reference_frame_id = scene_->getFrameId(reference_frame);
    if (!maybe_reference_frame_id) {
      throw std::runtime_error("Could not initialize pose constraint: " +
                               maybe_reference_frame_id.error());
    }
    reference_frame_id_ = maybe_reference_frame_id.value();
  }

  if ((lower_bounds_.array() > upper_bounds_.array()).any()) {
    throw std::runtime_error(
        "Could not initialize pose constraint: every lower bound must be at most its upper bound.");
  }
  if (position_tolerance_ <= 0.0 || orientation_tolerance_ <= 0.0) {
    throw std::runtime_error("Could not initialize pose constraint: tolerances must be positive.");
  }

  // Initialize private kinematics data, matrices, and vectors.
  const auto& model = scene_->getModel();
  data_ = pinocchio::Data(model);
  displacement_ = TaskSpaceVector::Zero();
  displacement_jacobian_ = Eigen::MatrixXd::Zero(6, joint_group_info_.v_indices.size());
  world_jacobian_ = Eigen::MatrixXd::Zero(6, model.nv);
  reference_jacobian_ = Eigen::MatrixXd::Zero(6, model.nv);
}

TaskSpaceVector PoseConstraint::boundsResidual(const TaskSpaceVector& displacement) const {
  // Coordinates within bounds clamp to themselves and so contribute exactly zero, including the
  // unbounded ones whose limits are infinite.
  return displacement - displacement.cwiseMax(lower_bounds_).cwiseMin(upper_bounds_);
}

TaskSpaceVector PoseConstraint::computeDisplacement(const Eigen::VectorXd& q) {
  updateKinematics(q, /*with_jacobian*/ false);
  return displacement_;
}

Eigen::VectorXd PoseConstraint::computeError(const Eigen::VectorXd& q) {
  updateKinematics(q, /*with_jacobian*/ false);
  return boundsResidual(displacement_);
}

void PoseConstraint::computeErrorAndJacobian(const Eigen::VectorXd& q,
                                             Eigen::Ref<Eigen::VectorXd> error,
                                             Eigen::Ref<Eigen::MatrixXd> jacobian) {
  updateKinematics(q, /*with_jacobian*/ true);
  error = boundsResidual(displacement_);
  jacobian = displacement_jacobian_;
}

bool PoseConstraint::satisfiesResidual(const Eigen::Ref<const Eigen::VectorXd>& error,
                                       double tolerance_scale) const {
  return error.head<3>().norm() <= tolerance_scale * position_tolerance_ &&
         error.tail<3>().norm() <= tolerance_scale * orientation_tolerance_;
}

void PoseConstraint::updateKinematics(const Eigen::VectorXd& q, bool with_jacobian) {
  const auto& model = scene_->getModel();
  if (q.size() != model.nq) {
    throw std::runtime_error(
        "Pose constraint on frame '" + frame_name_ + "' was evaluated at a configuration of size " +
        std::to_string(q.size()) + ", but the model has nq = " + std::to_string(model.nq) +
        ". Pass full joint positions (see Scene::toFullJointPositions).");
  }

  pinocchio::forwardKinematics(model, data_, q);
  pinocchio::updateFramePlacement(model, data_, frame_id_);

  // Place the region frame in the world. Without a reference frame it is already there.
  pinocchio::SE3 world_region_tform = region_tform_;
  Eigen::Vector3d reference_offset = Eigen::Vector3d::Zero();
  if (reference_frame_id_.has_value()) {
    pinocchio::updateFramePlacement(model, data_, reference_frame_id_.value());
    world_region_tform = data_.oMf.at(reference_frame_id_.value()) * region_tform_;
    reference_offset = data_.oMf.at(frame_id_).translation() -
                       data_.oMf.at(reference_frame_id_.value()).translation();
  }

  const Eigen::Matrix3d& region_rotation = world_region_tform.rotation();
  const pinocchio::SE3 pose = world_region_tform.actInv(data_.oMf.at(frame_id_));
  displacement_.head<3>() = pose.translation();
  displacement_.tail<3>() = pinocchio::rpy::matrixToRpy(pose.rotation());

  if (!with_jacobian) {
    return;
  }

  world_jacobian_.setZero();
  pinocchio::computeFrameJacobian(model, data_, q, frame_id_, pinocchio::LOCAL_WORLD_ALIGNED,
                                  world_jacobian_);

  if (reference_frame_id_.has_value()) {
    // The region frame rides along with the reference frame, so what matters is the constrained
    // frame's motion *relative* to it. Subtract the reference frame's own motion via the transport
    // theorem, exactly as Scene::computeRelativeFrameJacobian does, but against this constraint's
    // private data so the Scene's shared kinematics scratch is left untouched.
    //
    //   v_rel = v_frame - v_reference + skew(offset) * omega_reference
    //   omega_rel = omega_frame - omega_reference
    //
    // where offset is the world-frame vector from the reference origin to the frame origin.
    reference_jacobian_.setZero();
    pinocchio::computeFrameJacobian(model, data_, q, reference_frame_id_.value(),
                                    pinocchio::LOCAL_WORLD_ALIGNED, reference_jacobian_);

    const Eigen::Matrix3d offset_skew =
        (Eigen::Matrix3d() << 0.0, -reference_offset.z(), reference_offset.y(),
         reference_offset.z(), 0.0, -reference_offset.x(), -reference_offset.y(),
         reference_offset.x(), 0.0)
            .finished();
    world_jacobian_.topRows<3>() -=
        reference_jacobian_.topRows<3>() - offset_skew * reference_jacobian_.bottomRows<3>();
    world_jacobian_.bottomRows<3>() -= reference_jacobian_.bottomRows<3>();
  }

  // Restrict to the planning group's velocity degrees of freedom, then map the frame's world-axes
  // velocity into the region frame's displacement coordinates. The linear rows are a plain
  // rotation. The angular rows additionally chain through the inverse roll-pitch-yaw rate map,
  // without which the step would treat Euler angles as an angular velocity -- only true near zero
  // pitch.
  const auto group_jacobian =
      world_jacobian_(Eigen::placeholders::all, joint_group_info_.v_indices);
  displacement_jacobian_.topRows<3>().noalias() =
      region_rotation.transpose() * group_jacobian.topRows<3>();
  displacement_jacobian_.bottomRows<3>().noalias() =
      pinocchio::rpy::computeRpyJacobianInverse(displacement_.tail<3>(), pinocchio::WORLD) *
      (region_rotation.transpose() * group_jacobian.bottomRows<3>());
}

ConstraintProjector::ConstraintProjector(
    const std::shared_ptr<Scene> scene, const std::string& group_name,
    const std::vector<std::shared_ptr<Constraint>>& constraints,
    const ConstraintProjectorOptions& options)
    : scene_{scene}, constraints_{constraints}, options_{options} {

  // Validate the joint group.
  const auto maybe_joint_group_info = scene_->getJointGroupInfo(group_name);
  if (!maybe_joint_group_info) {
    throw std::runtime_error("Could not initialize constraint projector: " +
                             maybe_joint_group_info.error());
  }
  joint_group_info_ = maybe_joint_group_info.value();

  // Cache the group's position limits so we can clamp with a single cwise expression.
  const auto maybe_position_limits =
      scene_->getPositionLimitVectors(group_name, /*collapsed*/ false);
  if (!maybe_position_limits) {
    throw std::runtime_error("Could not initialize constraint projector: " +
                             maybe_position_limits.error());
  }
  std::tie(lower_position_limits_, upper_position_limits_) = maybe_position_limits.value();

  if (options_.path_step_size <= 0.0) {
    throw std::runtime_error(
        "Could not initialize constraint projector: path_step_size must be positive.");
  }

  if (options_.convergence_ratio <= 0.0 || options_.convergence_ratio > 1.0) {
    throw std::runtime_error(
        "Could not initialize constraint projector: convergence_ratio must be in (0, 1].");
  }

  // Every constraint must move the same degrees of freedom, since the projection solves for a
  // single joint velocity that satisfies all of them at once.
  const auto num_group_dofs = joint_group_info_.v_indices.size();
  size_t total_dimension = 0;
  constraint_errors_.reserve(constraints_.size());
  constraint_jacobians_.reserve(constraints_.size());
  for (const auto& constraint : constraints_) {
    if (!constraint) {
      throw std::runtime_error("Could not initialize constraint projector: null constraint.");
    }
    if (constraint->getGroupName() != group_name) {
      throw std::runtime_error("Could not initialize constraint projector: a constraint uses "
                               "joint group '" +
                               constraint->getGroupName() + "', but the projector uses '" +
                               group_name + "'.");
    }
    const auto dimension = static_cast<Eigen::Index>(constraint->dimension());
    total_dimension += constraint->dimension();
    constraint_errors_.emplace_back(Eigen::VectorXd::Zero(dimension));
    constraint_jacobians_.emplace_back(Eigen::MatrixXd::Zero(dimension, num_group_dofs));
  }

  // Size the scratch for the worst case, where every coordinate of every constraint is violated.
  const auto max_rows = static_cast<Eigen::Index>(total_dimension);
  active_error_ = Eigen::VectorXd::Zero(max_rows);
  active_jacobian_ = Eigen::MatrixXd::Zero(max_rows, num_group_dofs);
  jjt_ = Eigen::MatrixXd::Zero(max_rows, max_rows);
  group_velocity_ = Eigen::VectorXd::Zero(num_group_dofs);
  velocity_ = Eigen::VectorXd::Zero(scene_->getModel().nv);
}

bool ConstraintProjector::satisfies(const Eigen::VectorXd& q) {
  for (const auto& constraint : constraints_) {
    if (!constraint->satisfies(q)) {
      return false;
    }
  }
  return true;
}

bool ConstraintProjector::project(Eigen::VectorXd& q) {
  if (constraints_.empty()) {
    return true;
  }

  const auto& model = scene_->getModel();
  const auto& q_indices = joint_group_info_.q_indices;
  const auto& v_indices = joint_group_info_.v_indices;

  for (size_t iter = 0;; ++iter) {
    // Evaluate every constraint and stack only the violated coordinates into one task. Dropping the
    // coordinates that are already within bounds is what makes this a projection rather than a
    // pose-tracking IK step: an all-zero row would demand that a free coordinate not move at all,
    // whereas omitting it lets the correction slide along that direction.
    bool satisfied = true;
    Eigen::Index num_active = 0;
    for (size_t idx = 0; idx < constraints_.size(); ++idx) {
      auto& constraint = *constraints_.at(idx);
      auto& error = constraint_errors_.at(idx);
      auto& jacobian = constraint_jacobians_.at(idx);

      constraint.computeErrorAndJacobian(q, error, jacobian);
      if (!constraint.satisfiesResidual(error, options_.convergence_ratio)) {
        satisfied = false;
      }
      for (Eigen::Index row = 0; row < error.size(); ++row) {
        if (error(row) == 0.0) {
          continue;
        }
        active_error_(num_active) = error(row);
        active_jacobian_.row(num_active) = jacobian.row(row);
        ++num_active;
      }
    }

    if (satisfied) {
      return true;
    }
    if (iter >= options_.max_iters) {
      return false;
    }

    // Damped least-squares step that cancels the stacked violations.
    if (!dampedLeastSquaresStep(active_jacobian_.topRows(num_active),
                                active_error_.head(num_active), options_.damping,
                                jjt_.topLeftCorner(num_active, num_active), group_velocity_)) {
      return false;
    }

    velocity_.setZero();
    velocity_(v_indices) = options_.correction_step_size * group_velocity_;
    q = pinocchio::integrate(model, q, velocity_);
    q(q_indices) = q(q_indices).cwiseMax(lower_position_limits_).cwiseMin(upper_position_limits_);
  }
}

bool ConstraintProjector::satisfiesAlongPath(const Eigen::VectorXd& q_start,
                                             const Eigen::VectorXd& q_end,
                                             const double max_step_size,
                                             const bool check_endpoints) {
  if (constraints_.empty()) {
    return true;
  }
  if (check_endpoints && (!satisfies(q_start) || !satisfies(q_end))) {
    return false;
  }

  // Mirrors the stepping in hasCollisionsAlongPath, so a constrained edge is validated at exactly
  // the same interior configurations that its collision check visits.
  const auto distance = scene_->configurationDistance(q_start, q_end);
  if (distance <= max_step_size) {
    return true;
  }

  const auto num_steps = static_cast<size_t>(std::ceil(distance / max_step_size));
  for (size_t idx = 1; idx < num_steps; ++idx) {
    const auto fraction = static_cast<double>(idx) / static_cast<double>(num_steps);
    if (!satisfies(scene_->interpolate(q_start, q_end, fraction))) {
      return false;
    }
  }
  return true;
}

}  // namespace roboplan
