#include <algorithm>
#include <chrono>
#include <cmath>

#include <roboplan/core/collision_context.hpp>
#include <roboplan_simple_ik/simple_ik.hpp>

namespace roboplan {

SimpleIk::SimpleIk(const std::shared_ptr<Scene> scene, const SimpleIkOptions& options)
    : scene_{scene}, options_{options} {
  data_ = pinocchio::Data(scene_->getModel());

  // Validate the joint group.
  const auto maybe_joint_group_info = scene_->getJointGroupInfo(options.group_name);
  if (!maybe_joint_group_info) {
    throw std::runtime_error("Could not initialize IK solver: " + maybe_joint_group_info.error());
  }
  joint_group_info_ = maybe_joint_group_info.value();

  // Cache the group's position limits so we can clamp with a single cwise expression.
  const auto maybe_position_limits =
      scene_->getPositionLimitVectors(options.group_name, /*collapsed*/ false);
  if (!maybe_position_limits) {
    throw std::runtime_error("Could not initialize IK solver: " + maybe_position_limits.error());
  }
  std::tie(lower_position_limits_, upper_position_limits_) = maybe_position_limits.value();

  // Pre-compute the per-DOF limit centering mapping. Centering maps a position-space error onto
  // a velocity DOF one-to-one, so only 1-DOF joints with finite position limits participate;
  // the remaining DOFs (e.g. continuous joints, whose position representation is larger than
  // their velocity representation) simply contribute zero centering velocity.
  const Eigen::Index group_nv = joint_group_info_.v_indices.size();
  centering_q_slot_ = Eigen::VectorXi::Constant(group_nv, -1);
  centering_center_ = Eigen::VectorXd::Zero(group_nv);
  centering_half_span_ = Eigen::VectorXd::Ones(group_nv);
  Eigen::Index q_offset = 0;
  Eigen::Index v_offset = 0;
  for (const auto& joint_name : joint_group_info_.joint_names) {
    const auto maybe_joint_info = scene_->getJointInfo(joint_name);
    if (!maybe_joint_info) {
      throw std::runtime_error("Could not initialize IK solver: " + maybe_joint_info.error());
    }
    const auto& joint_info = maybe_joint_info.value();
    const auto nq = static_cast<Eigen::Index>(joint_info.num_position_dofs);
    const auto nv = static_cast<Eigen::Index>(joint_info.num_velocity_dofs);
    if (nq == 1 && nv == 1 && v_offset < group_nv) {
      const double lower = joint_info.limits.min_position[0];
      const double upper = joint_info.limits.max_position[0];
      if (std::isfinite(lower) && std::isfinite(upper)) {
        centering_q_slot_[v_offset] = static_cast<int>(q_offset);
        centering_center_[v_offset] = 0.5 * (lower + upper);
        // Guard against zero spans; huge spans naturally shrink the normalized centering
        // error toward zero, softening centering for wide-range joints.
        centering_half_span_[v_offset] = std::max(0.5 * (upper - lower), 1.0e-6);
        has_centering_dofs_ = true;
      }
    }
    q_offset += nq;
    v_offset += nv;
  }

  // Initialize matrices and vectors
  const auto& model = scene_->getModel();
  full_jacobian_ = Eigen::MatrixXd(6, model.nv);
  vel_ = Eigen::VectorXd::Zero(model.nv);
};

bool SimpleIk::solveIk(const std::vector<CartesianConfiguration>& goals,
                       const JointConfiguration& start, JointConfiguration& solution) {
  const auto start_time = std::chrono::steady_clock::now();
  const std::chrono::duration<double> timeout(options_.max_time);

  // Snapshot the scene geometry into a private collision context for this solve, so the IK
  // iterations check collisions against their own scratch rather than the Scene's shared data.
  const CollisionContext collision_context(*scene_);

  const auto& model = scene_->getModel();

  const auto& q_indices = joint_group_info_.q_indices;
  const auto& v_indices = joint_group_info_.v_indices;

  solution = start;
  centering_center_ = start.positions; // TEST
  auto q = scene_->toFullJointPositions(options_.group_name, start.positions);

  // Pre-compute the frame IDs and resize relevant matrices.
  const auto n_frames = goals.size();
  std::vector<pinocchio::FrameIndex> frame_ids;
  frame_ids.reserve(n_frames);
  for (const auto& goal : goals) {
    const auto frame_id_result = scene_->getFrameId(goal.tip_frame);
    if (!frame_id_result) {
      throw std::runtime_error("Failed to get frame ID: " + frame_id_result.error());
    }
    frame_ids.push_back(frame_id_result.value());
  }

  // Optionally set base_frame IDs for each of the tip links
  std::vector<std::optional<pinocchio::FrameIndex>> base_frame_ids;
  base_frame_ids.reserve(n_frames);
  for (const auto& goal : goals) {
    if (goal.base_frame.empty()) {
      base_frame_ids.push_back(std::nullopt);
    } else {
      const auto base_id_result = scene_->getFrameId(goal.base_frame);
      if (!base_id_result) {
        throw std::runtime_error("Failed to get the base frame ID: " + base_id_result.error());
      }
      base_frame_ids.push_back(base_id_result.value());
    }
  }
  const auto n_dims = 6 * n_frames;
  error_.resize(n_dims, 1);
  jacobian_.resize(n_dims, v_indices.size());
  jjt_.resize(n_dims, n_dims);

  // Used to track the closest ik solution to the starting configuration
  const auto q_seed = q;
  std::optional<Eigen::VectorXd> nearest_solution;
  double nearest_distance = std::numeric_limits<double>::max();

  size_t attempt = 0;
  bool timed_out = false;
  while (attempt <= options_.max_restarts && !timed_out) {
    if (attempt > 0) {
      const auto maybe_q_random = scene_->randomCollisionFreePositions();
      if (!maybe_q_random) {
        throw std::runtime_error("Failed to generate random collision free positions for IK.");
      }
      q = maybe_q_random.value();
    }

    size_t iter = 0;
    while (iter < options_.max_iters) {
      pinocchio::forwardKinematics(model, data_, q);

      // Loop through all the frames and accumulate errors and Jacobians.
      for (size_t idx = 0; idx < n_frames; ++idx) {
        const auto& goal = goals.at(idx);
        const auto goal_tform = pinocchio::SE3(goal.tform);
        const auto& frame_id = frame_ids.at(idx);

        pinocchio::updateFramePlacement(model, data_, frame_id);

        // Determine the world goal depending on whether a base_frame was configured for the target.
        pinocchio::SE3 world_goal;
        if (base_frame_ids[idx].has_value()) {
          pinocchio::updateFramePlacement(model, data_, base_frame_ids[idx].value());
          world_goal = data_.oMf[base_frame_ids[idx].value()] * goal_tform;
        } else {
          world_goal = goal_tform;
        }
        // Pose error of the current frame relative to the goal, in the frame's local coordinates.
        const pinocchio::SE3 frame_error = world_goal.actInv(data_.oMf[frame_id]);
        error_.segment(idx * 6, 6) = pinocchio::log6(frame_error).toVector();

        // Use the analytic (Gauss-Newton) Jacobian: chain the local frame Jacobian through the
        // derivative of log6 so the step accounts for the SE(3) curvature of the error near the
        // goal. This converges far faster than the first-order frame Jacobian alone.
        pinocchio::Jlog6(frame_error, Jlog_);

        full_jacobian_.setZero();
        pinocchio::computeFrameJacobian(model, data_, q, frame_id, pinocchio::ReferenceFrame::LOCAL,
                                        full_jacobian_);
        jacobian_.block(idx * 6, 0, 6, v_indices.size()) =
            Jlog_ * full_jacobian_(Eigen::placeholders::all, v_indices);
      }

      // Ensure every target frame meets both linear and angular error tolerances.
      bool converged = true;
      for (size_t idx = 0; idx < n_frames; ++idx) {
        const auto target_frame_error = error_.segment(idx * 6, 6);
        const auto linear_error = target_frame_error.head<3>().norm();
        const auto angular_error = target_frame_error.tail<3>().norm();
        if (linear_error > options_.max_linear_error_norm ||
            angular_error > options_.max_angular_error_norm) {
          converged = false;
          break;
        }
      }

      if (converged) {
        if (!options_.check_collisions || !collision_context.hasCollisions(q)) {
          // Return immedaiately if requested
          if (options_.fast_return) {
            solution.positions = q(q_indices);
            return true;
          }

          // Otherwise record the distance and continue iterating
          const double dist = (q(q_indices) - q_seed(q_indices)).squaredNorm();
          if (!nearest_solution.has_value() || dist < nearest_distance) {
            nearest_solution = q(q_indices);
            nearest_distance = dist;
          }
          break;
        }
      }

      jjt_.noalias() = jacobian_ * jacobian_.transpose();
      jjt_.diagonal().array() += options_.damping;
      const auto jjt_ldlt = jjt_.ldlt();
      vel_(v_indices) = -jacobian_.transpose() * jjt_ldlt.solve(error_);
      if (has_centering_dofs_ && options_.limit_centering_gain > 0.0) {
        // Nullspace-projected limit centering: pull the 1-DOF bounded joints toward mid-range
        // using only the redundant degrees of freedom, so the pose error is unaffected to first
        // order. DOFs without a valid mapping (e.g. continuous joints) contribute zero
        // centering velocity but still participate in the nullspace projection.
        // N = I - J+ J with the damped pseudoinverse J+ = J^T (J J^T + lambda I)^-1.
        Eigen::VectorXd v_center = Eigen::VectorXd::Zero(centering_q_slot_.size());
        for (Eigen::Index i = 0; i < centering_q_slot_.size(); ++i) {
          if (centering_q_slot_[i] >= 0) {
            const double position = q(q_indices[centering_q_slot_[i]]);
            v_center[i] = options_.limit_centering_gain * (centering_center_[i] - position) /
                          centering_half_span_[i];
          }
        }
        vel_(v_indices) += v_center - jacobian_.transpose() * jjt_ldlt.solve(jacobian_ * v_center);
      }
      if (vel_.hasNaN()) {
        break;
      }

      q = pinocchio::integrate(model, q, vel_ * options_.step_size);
      q(q_indices) = q(q_indices).cwiseMax(lower_position_limits_).cwiseMin(upper_position_limits_);
      ++iter;

      // On timeout, stop iterating and fall through to return the best solution found so far.
      if (std::chrono::steady_clock::now() - start_time > timeout) {
        timed_out = true;
        break;
      }
    }

    ++attempt;
  }

  if (nearest_solution.has_value()) {
    solution.positions = nearest_solution.value();
    return true;
  } else {
    return false;
  }
}

}  // namespace roboplan
