#include <roboplan_oink/constraints/relative_pose.hpp>

#include <stdexcept>

#include <pinocchio/spatial/explog.hpp>

namespace roboplan {

RelativePoseConstraint::RelativePoseConstraint(const Oink& oink, const Scene& scene,
                                               const std::string& frame_a,
                                               const std::string& frame_b,
                                               const Eigen::Matrix4d& target_pose,
                                               const Eigen::Vector3d& position_tolerance,
                                               const Eigen::Vector3d& orientation_tolerance)
    : frame_a(frame_a), frame_b(frame_b), target_pose(target_pose),
      position_tolerance(position_tolerance), orientation_tolerance(orientation_tolerance),
      v_indices(oink.v_indices), full_jacobian(Eigen::MatrixXd::Zero(6, scene.getModel().nv)) {
  const auto maybe_a = scene.getFrameId(frame_a);
  if (!maybe_a) {
    throw std::runtime_error("RelativePoseConstraint: " + maybe_a.error());
  }
  const auto maybe_b = scene.getFrameId(frame_b);
  if (!maybe_b) {
    throw std::runtime_error("RelativePoseConstraint: " + maybe_b.error());
  }
  frame_a_id = maybe_a.value();
  frame_b_id = maybe_b.value();
}

int RelativePoseConstraint::getNumConstraints(const SceneContext& /*context*/) const { return 6; }

Eigen::Matrix<double, 6, 1>
RelativePoseConstraint::computeError(const SceneContext& context) const {
  const auto& data = context.getData();
  const pinocchio::SE3 T_err =
      pinocchio::SE3(target_pose).actInv(data.oMf[frame_a_id].actInv(data.oMf[frame_b_id]));
  Eigen::Matrix<double, 6, 1> error;
  error << T_err.translation(), pinocchio::log3(T_err.rotation());
  return error;
}

tl::expected<void, std::string> RelativePoseConstraint::computeQpConstraints(
    const SceneContext& context, Eigen::Ref<Eigen::MatrixXd> constraint_matrix,
    Eigen::Ref<Eigen::VectorXd> lower_bounds, Eigen::Ref<Eigen::VectorXd> upper_bounds) const {
  // Relative twist of frame_b w.r.t. frame_a, at b's origin in world-aligned axes. Its linear part
  // is R_a * d(p_ab)/dt and its angular part is the world relative angular velocity.
  context.computeRelativeFrameJacobian(context.getJointPositions(), frame_b_id, frame_a,
                                       pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                                       full_jacobian);
  const auto& data = context.getData();
  const Eigen::Matrix3d R_target = target_pose.topLeftCorner<3, 3>();
  const Eigen::Matrix3d R_err = R_target.transpose() * data.oMf[frame_a_id].rotation().transpose() *
                                data.oMf[frame_b_id].rotation();

  // d(e_pos) = (R_a R_target)^T J_lin, and d(e_rot) = Jlog3(R_err) R_b^T J_ang
  // The body angular velocity of R_err equals R_b^T times the world relative angular velocity.
  Eigen::Matrix3d Jlog;
  pinocchio::Jlog3(R_err, Jlog);
  constraint_matrix.topRows<3>() = (data.oMf[frame_a_id].rotation() * R_target).transpose() *
                                   full_jacobian.topRows<3>()(Eigen::placeholders::all, v_indices);
  constraint_matrix.bottomRows<3>() =
      Jlog * data.oMf[frame_b_id].rotation().transpose() *
      full_jacobian.bottomRows<3>()(Eigen::placeholders::all, v_indices);

  const Eigen::Matrix<double, 6, 1> error = computeError(context);
  Eigen::Matrix<double, 6, 1> tolerance;
  tolerance << position_tolerance, orientation_tolerance;
  lower_bounds = -tolerance - error;
  upper_bounds = tolerance - error;
  return {};
}

}  // namespace roboplan
