#include <roboplan_simple_ik/simple_ik.hpp>

namespace roboplan {

SimpleIk::SimpleIk(const std::shared_ptr<Scene> scene, const SimpleIkOptions& options)
    : scene_{scene}, options_{options} {};

bool SimpleIk::solveIk(const CartesianConfiguration& goal, const JointConfiguration& start,
                       JointConfiguration& solution) {

  size_t iter = 0;
  bool result = false;
  const auto& model = scene_->getModel();
  pinocchio::Data data(model);
  const auto frame_id_result = scene_->getFrameId(goal.tip_frame);
  if (!frame_id_result) {
    throw std::runtime_error("Failed to get frame ID: " + frame_id_result.error());
  }
  auto frame_id = frame_id_result.value();

  const auto goal_tform = pinocchio::SE3(goal.tform);
  solution = start;
  auto q = start.positions;
  Eigen::MatrixXd jacobian(6, model.nv);
  Eigen::MatrixXd jjt(6, 6);
  Eigen::VectorXd vel(model.nv);

  while (iter < options_.max_iters) {
    pinocchio::forwardKinematics(model, data, q);
    pinocchio::updateFramePlacement(model, data, frame_id);

    const auto error = pinocchio::log6(goal_tform.actInv(data.oMf[frame_id])).toVector();

    if (error.norm() <= options_.max_error_norm) {
      solution.positions = q;
      return true;
    }

    pinocchio::computeFrameJacobian(model, data, q, frame_id, pinocchio::ReferenceFrame::LOCAL,
                                    jacobian);

    jjt.noalias() = jacobian * jacobian.transpose();
    jjt.diagonal().array() += options_.damping;
    vel.noalias() = -jacobian.transpose() * jjt.ldlt().solve(error);

    if (vel.hasNaN()) {
      break;
    }

    q = pinocchio::integrate(model, q, vel * options_.step_size);

    ++iter;
  }

  return result;
}

}  // namespace roboplan
