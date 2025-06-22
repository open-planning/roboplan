#include <roboplan_simple_ik/simple_ik.hpp>

namespace roboplan {

SimpleIk::SimpleIk(const Scene& scene, const SimpleIkOptions& options)
    : scene_{scene}, options_{options} {};

bool SimpleIk::solveIk(const CartesianConfiguration& goal, const JointConfiguration& start,
                       JointConfiguration& solution) {

  size_t iter = 0;
  bool result = false;
  const auto& model = scene_.getModel();
  pinocchio::Data data(model);
  const auto frame_id = model.getFrameId(goal.tip_frame);

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

    // std::cout << "Iter " << iter << ", error: " << error << "\n";

    if (error.norm() <= options_.max_error_norm) {
      std::cout << "Solved IK within error norm!\n";
      solution.positions = q;
      return true;
    }

    pinocchio::computeFrameJacobian(model, data, q, frame_id, pinocchio::ReferenceFrame::LOCAL,
                                    jacobian);

    jjt.noalias() = jacobian * jacobian.transpose();
    jjt.diagonal().array() += options_.damping;
    vel.noalias() = -jacobian.transpose() * jjt.ldlt().solve(error);
    q = pinocchio::integrate(model, q, vel * options_.step_size);

    ++iter;
  }

  return result;
}

} // namespace roboplan
