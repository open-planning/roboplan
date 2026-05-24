#include <roboplan_oink/barriers/manipulability_barrier.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

namespace roboplan {

ManipulabilityBarrier::ManipulabilityBarrier(const Oink& oink, const Scene& scene,
                                             const std::string& frame_name, double dt,
                                             double sigma_safe, double gain,
                                             double safe_displacement_gain, double safety_margin,
                                             double fd_epsilon)
    : Barrier(gain, dt, safe_displacement_gain, safety_margin), frame_name(frame_name),
      sigma_safe(sigma_safe), fd_epsilon(fd_epsilon), v_indices(oink.v_indices),
      q_indices(oink.q_indices) {
  const auto maybe_frame_id = scene.getFrameId(frame_name);
  if (!maybe_frame_id) {
    throw std::runtime_error("ManipulabilityBarrier: frame '" + frame_name +
                             "' not found in scene");
  }
  frame_id = maybe_frame_id.value();

  model_ = scene.getModel();
  data_ = pinocchio::Data(model_);
  full_jacobian_ = Eigen::MatrixXd::Zero(6, model_.nv);

  // One barrier constraint: σ_min(q) − σ_safe ≥ 0
  initializeStorage(1, oink.num_variables);
}

int ManipulabilityBarrier::getNumBarriers(const Scene& /*scene*/) const { return 1; }

double ManipulabilityBarrier::computeSigmaMin(const Eigen::VectorXd& q) {
  full_jacobian_.setZero();
  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED,
                              full_jacobian_);
  // Extract the arm-joint columns and compute the smallest singular value.
  Eigen::MatrixXd J_arm(6, v_indices.size());
  for (int i = 0; i < v_indices.size(); ++i) {
    J_arm.col(i) = full_jacobian_.col(v_indices[i]);
  }
  return Eigen::BDCSVD<Eigen::MatrixXd>(J_arm).singularValues().minCoeff();
}

tl::expected<void, std::string> ManipulabilityBarrier::computeBarrier(const Scene& scene) {
  cached_sigma0_ = computeSigmaMin(scene.getCurrentJointPositions());
  barrier_values[0] = cached_sigma0_ - sigma_safe;
  return {};
}

tl::expected<void, std::string> ManipulabilityBarrier::computeJacobian(const Scene& scene) {
  // Forward finite differences: perturb each arm joint position by fd_epsilon.
  const Eigen::VectorXd q = scene.getCurrentJointPositions();
  Eigen::VectorXd q_pert = q;
  for (int i = 0; i < v_indices.size(); ++i) {
    q_pert[q_indices[i]] += fd_epsilon;
    const double sigma_plus = computeSigmaMin(q_pert);
    jacobian_container(0, i) = (sigma_plus - cached_sigma0_) / fd_epsilon;
    q_pert[q_indices[i]] = q[q_indices[i]];
  }
  return {};
}

tl::expected<double, std::string>
ManipulabilityBarrier::evaluateAtConfiguration(const pinocchio::Model& model, pinocchio::Data& data,
                                               const Eigen::VectorXd& q) const {
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model.nv);
  pinocchio::computeJointJacobians(model, data, q);
  pinocchio::updateFramePlacements(model, data);
  pinocchio::getFrameJacobian(model, data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J);
  Eigen::MatrixXd J_arm(6, v_indices.size());
  for (int i = 0; i < static_cast<int>(v_indices.size()); ++i) {
    J_arm.col(i) = J.col(v_indices[i]);
  }
  const double sigma_min = Eigen::BDCSVD<Eigen::MatrixXd>(J_arm).singularValues().minCoeff();
  return sigma_min - sigma_safe;
}

}  // namespace roboplan
