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
  jacobian_ = Eigen::MatrixXd(6, joint_group_info_.v_indices.size());
  jjt_ = Eigen::MatrixXd(6, 6);
  vel_ = Eigen::VectorXd::Zero(model.nv);
};

bool SimpleIk::solveIk(const CartesianConfiguration& goal, const JointConfiguration& start,
                       JointConfiguration& solution) {
  const auto start_time = std::chrono::steady_clock::now();
  const std::chrono::duration<double> timeout(options_.max_time);

  bool result = false;
  const auto& model = scene_->getModel();
  const auto frame_id_result = scene_->getFrameId(goal.tip_frame);
  if (!frame_id_result) {
    throw std::runtime_error("Failed to get frame ID: " + frame_id_result.error());
  }
  auto frame_id = frame_id_result.value();

  const auto& q_indices = joint_group_info_.q_indices;
  const auto& v_indices = joint_group_info_.v_indices;

  const auto goal_tform = pinocchio::SE3(goal.tform);
  solution = start;
  auto q = scene_->toFullJointPositions(options_.group_name, start.positions);

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
      pinocchio::updateFramePlacement(model, data_, frame_id);

      const auto error = pinocchio::log6(goal_tform.actInv(data_.oMf[frame_id])).toVector();

      if (error.norm() <= options_.max_error_norm) {
        if (!options_.check_collisions || !scene_->hasCollisions(q)) {
          solution.positions = q(q_indices);
        }
        return true;
      }

      pinocchio::computeFrameJacobian(model, data_, q, frame_id, pinocchio::ReferenceFrame::LOCAL,
                                      full_jacobian_);
      jacobian_ = full_jacobian_(Eigen::placeholders::all, v_indices);

      jjt_.noalias() = jacobian_ * jacobian_.transpose();
      jjt_.diagonal().array() += options_.damping;
      vel_(v_indices) = -jacobian_.transpose() * jjt_.ldlt().solve(error);

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
