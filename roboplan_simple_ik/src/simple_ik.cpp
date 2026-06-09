#include <chrono>

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

  bool result = false;
  const auto& model = scene_->getModel();

  const auto& q_indices = joint_group_info_.q_indices;
  const auto& v_indices = joint_group_info_.v_indices;

  solution = start;
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
  while (attempt <= options_.max_restarts) {
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
      vel_(v_indices) = -jacobian_.transpose() * jjt_.ldlt().solve(error_);
      if (vel_.hasNaN()) {
        break;
      }

      q = pinocchio::integrate(model, q, vel_ * options_.step_size);
      q(q_indices) = q(q_indices).cwiseMax(lower_position_limits_).cwiseMin(upper_position_limits_);
      ++iter;

      // Check for timeouts.
      if (std::chrono::steady_clock::now() - start_time > timeout) {
        return result;
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
