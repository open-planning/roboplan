#include <chrono>
#include <cmath>
#include <limits>

#include <roboplan/core/math_utils.hpp>
#include <roboplan/core/scene_context.hpp>
#include <roboplan_simple_ik/simple_ik.hpp>

namespace roboplan {

SimpleIk::SimpleIk(const std::shared_ptr<Scene> scene, const SimpleIkOptions& options)
    : scene_{scene}, options_{options} {
  std::random_device rd;
  rng_gen_ = std::mt19937(rd());

  // Validate the joint group.
  const auto maybe_joint_group_info = scene_->getJointGroupInfo(options.group_name);
  if (!maybe_joint_group_info) {
    throw std::runtime_error("Could not initialize IK solver: " + maybe_joint_group_info.error());
  }
  joint_group_info_ = maybe_joint_group_info.value();

  // Cache the group's position limits for the saturation step.
  const auto maybe_position_limits =
      scene_->getPositionLimitVectors(options.group_name, /*collapsed*/ false);
  if (!maybe_position_limits) {
    throw std::runtime_error("Could not initialize IK solver: " + maybe_position_limits.error());
  }
  std::tie(lower_position_limits_, upper_position_limits_) = maybe_position_limits.value();

  // Pre-compute the per-DOF limit handling mapping. Saturation maps a position slot onto a
  // velocity DOF one-to-one, so only 1-DOF joints participate; the remaining DOFs (e.g. planar
  // or continuous joints, whose position representation is larger than their velocity
  // representation) are exempt from limit handling.
  const auto group_nv = static_cast<Eigen::Index>(joint_group_info_.v_indices.size());
  limit_q_slot_ = Eigen::VectorXi::Constant(group_nv, -1);
  Eigen::Index q_offset = 0;
  Eigen::Index v_offset = 0;
  for (const auto& joint_name : joint_group_info_.joint_names) {
    const auto maybe_joint_info = scene_->getJointInfo(joint_name);
    if (!maybe_joint_info) {
      throw std::runtime_error("Could not initialize IK solver: " + maybe_joint_info.error());
    }
    const auto nq = static_cast<Eigen::Index>(maybe_joint_info.value().num_position_dofs);
    const auto nv = static_cast<Eigen::Index>(maybe_joint_info.value().num_velocity_dofs);
    if (nq == 1 && nv == 1) {
      limit_q_slot_[v_offset] = static_cast<int>(q_offset);
    }
    q_offset += nq;
    v_offset += nv;
  }
  saturation_bound_ = Eigen::VectorXd::Zero(group_nv);
  group_vel_ = Eigen::VectorXd::Zero(group_nv);

  // Initialize matrices and vectors
  const auto& model = scene_->getModel();
  full_jacobian_ = Eigen::MatrixXd(6, model.nv);
  vel_ = Eigen::VectorXd::Zero(model.nv);
};

bool SimpleIk::solveIk(const std::vector<CartesianConfiguration>& goals,
                       const JointConfiguration& start, JointConfiguration& solution) {
  const auto start_time = std::chrono::steady_clock::now();
  const std::chrono::duration<double> timeout(options_.max_time);

  // Snapshot the scene into a private context for this solve, so solves can run
  // concurrently on one Scene.
  SceneContext context(*scene_);
  context.setRngSeed(static_cast<unsigned int>(rng_gen_()));

  const auto& model = scene_->getModel();
  pinocchio::Data& data = context.getData();

  const auto& q_indices = joint_group_info_.q_indices;
  const auto& v_indices = joint_group_info_.v_indices;

  solution = start;
  auto q = context.toFullJointPositions(options_.group_name, start.positions);

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
      const auto maybe_q_random = context.randomCollisionFreePositions();
      if (!maybe_q_random) {
        throw std::runtime_error("Failed to generate random collision free positions for IK.");
      }
      q = maybe_q_random.value();
    }

    size_t iter = 0;
    while (iter < options_.max_iters) {
      pinocchio::forwardKinematics(model, data, q);

      // Loop through all the frames and accumulate errors and Jacobians.
      for (size_t idx = 0; idx < n_frames; ++idx) {
        const auto& goal = goals.at(idx);
        const auto goal_tform = pinocchio::SE3(goal.tform);
        const auto& frame_id = frame_ids.at(idx);

        pinocchio::updateFramePlacement(model, data, frame_id);

        // Determine the world goal depending on whether a base_frame was configured for the target.
        pinocchio::SE3 world_goal;
        if (base_frame_ids[idx].has_value()) {
          pinocchio::updateFramePlacement(model, data, base_frame_ids[idx].value());
          world_goal = data.oMf[base_frame_ids[idx].value()] * goal_tform;
        } else {
          world_goal = goal_tform;
        }
        // Pose error of the current frame relative to the goal, in the frame's local coordinates.
        const pinocchio::SE3 frame_error = world_goal.actInv(data.oMf[frame_id]);
        error_.segment(idx * 6, 6) = pinocchio::log6(frame_error).toVector();

        // Use the analytic (Gauss-Newton) Jacobian: chain the local frame Jacobian through the
        // derivative of log6 so the step accounts for the SE(3) curvature of the error near the
        // goal. This converges far faster than the first-order frame Jacobian alone.
        pinocchio::Jlog6(frame_error, Jlog_);

        full_jacobian_.setZero();
        pinocchio::computeFrameJacobian(model, data, q, frame_id, pinocchio::ReferenceFrame::LOCAL,
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
        if (!options_.check_collisions || !context.hasCollisions(q)) {
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

      if (!dampedLeastSquaresStep(jacobian_, error_, options_.damping, jjt_, group_vel_)) {
        break;
      }
      if (!saturateStep(q)) {
        break;
      }

      q = pinocchio::integrate(model, q, vel_ * options_.step_size);
      // Snap saturated joints exactly onto their bound to kill integration round-off.
      for (Eigen::Index i = 0; i < saturation_bound_.size(); ++i) {
        if (!std::isnan(saturation_bound_[i])) {
          q[q_indices[limit_q_slot_[i]]] = saturation_bound_[i];
        }
      }
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

bool SimpleIk::saturateStep(const Eigen::VectorXd& q) {
  const auto& q_indices = joint_group_info_.q_indices;
  const auto& v_indices = joint_group_info_.v_indices;
  const auto group_nv = static_cast<Eigen::Index>(v_indices.size());

  saturation_bound_.setConstant(std::numeric_limits<double>::quiet_NaN());

  // Each pass either saturates at least one more DOF or exits,
  // so this terminates in at most group_nv passes.
  for (Eigen::Index pass = 0; pass < group_nv; ++pass) {
    bool any_new_saturation = false;
    for (Eigen::Index i = 0; i < group_nv; ++i) {
      if (limit_q_slot_[i] < 0 || !std::isnan(saturation_bound_[i])) {
        continue;
      }
      const double position = q[q_indices[limit_q_slot_[i]]];
      const double candidate = position + group_vel_[i] * options_.step_size;
      double bound;
      if (candidate < lower_position_limits_[limit_q_slot_[i]]) {
        bound = lower_position_limits_[limit_q_slot_[i]];
      } else if (candidate > upper_position_limits_[limit_q_slot_[i]]) {
        bound = upper_position_limits_[limit_q_slot_[i]];
      } else {
        continue;
      }
      // Land exactly on the bound this step and drop the joint from the task.
      saturation_bound_[i] = bound;
      group_vel_[i] = (bound - position) / options_.step_size;
      any_new_saturation = true;
    }
    if (!any_new_saturation) {
      break;
    }

    // Move the saturated joints' contributions to the error side of the task, then re-solve the
    // damped least-squares step for the remaining joints. This ensures the motion discarded by
    // saturation is redistributed instead of lost.
    task_jacobian_ = jacobian_;
    task_error_ = error_;
    for (Eigen::Index i = 0; i < group_nv; ++i) {
      if (!std::isnan(saturation_bound_[i])) {
        task_error_ += task_jacobian_.col(i) * group_vel_[i];
        task_jacobian_.col(i).setZero();
      }
    }
    if (!dampedLeastSquaresStep(task_jacobian_, task_error_, options_.damping, jjt_, group_vel_)) {
      return false;
    }
    for (Eigen::Index i = 0; i < group_nv; ++i) {
      if (!std::isnan(saturation_bound_[i])) {
        group_vel_[i] =
            (saturation_bound_[i] - q[q_indices[limit_q_slot_[i]]]) / options_.step_size;
      }
    }
  }

  vel_(v_indices) = group_vel_;
  return true;
}

void SimpleIk::setRngSeed(unsigned int seed) { rng_gen_ = std::mt19937(seed); }

}  // namespace roboplan
