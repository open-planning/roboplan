#include <chrono>

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

  // Initialize matrices and vectors
  const auto& model = scene_->getModel();
  full_jacobian_ = Eigen::MatrixXd(6, model.nv);
  vel_ = Eigen::VectorXd::Zero(model.nv);
};

bool SimpleIk::solveIk(const std::vector<CartesianConfiguration>& goals,
                       const JointConfiguration& start, JointConfiguration& solution) {
  const auto start_time = std::chrono::steady_clock::now();
  const std::chrono::duration<double> timeout(options_.max_time);

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
  const auto n_dims = 6 * n_frames;
  error_.resize(n_dims, 1);
  jacobian_.resize(n_dims, v_indices.size());
  jjt_.resize(n_dims, n_dims);

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

        error_.segment(idx * 6, 6) =
            pinocchio::log6(goal_tform.actInv(data_.oMf[frame_id])).toVector();

        full_jacobian_.setZero();
        pinocchio::computeFrameJacobian(model, data_, q, frame_id, pinocchio::ReferenceFrame::LOCAL,
                                        full_jacobian_);
        jacobian_.block(idx * 6, 0, 6, v_indices.size()) =
            full_jacobian_(Eigen::placeholders::all, v_indices);
      }

      // Finalize solution if the error is below threshold.
      // Note the error norm is accumulated per target frame (batch of 6).
      double error_norm = 0.0;
      for (size_t idx = 0; idx < n_frames; ++idx) {
        error_norm += error_.segment(idx * 6, 6).norm();
      }
      if (error_norm <= options_.max_error_norm) {
        if (!options_.check_collisions || !scene_->hasCollisions(q)) {
          solution.positions = q(q_indices);
        }
        return true;
      }

      jjt_.noalias() = jacobian_ * jacobian_.transpose();
      jjt_.diagonal().array() += options_.damping;
      vel_(v_indices) = -jacobian_.transpose() * jjt_.ldlt().solve(error_);
      if (vel_.hasNaN()) {
        break;
      }

      q = pinocchio::integrate(model, q, vel_ * options_.step_size);
      ++iter;

      // Check for timeouts.
      if (std::chrono::steady_clock::now() - start_time > timeout) {
        return result;
      }
    }

    ++attempt;
  }

  return result;
}

}  // namespace roboplan
