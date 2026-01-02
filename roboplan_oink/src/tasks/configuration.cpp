#include <roboplan_oink/tasks/configuration.hpp>

#include <pinocchio/algorithm/joint-configuration.hpp>

namespace roboplan {

ConfigurationTask::ConfigurationTask(const Eigen::VectorXd& target_q,
                                     const Eigen::VectorXd& joint_weights,
                                     const ConfigurationTaskParams& params)
    : Task(createWeightMatrix(joint_weights), params.task_gain, params.lm_damping),
      target_q(target_q), joint_weights(joint_weights) {
  // Validate joint weights are non-negative
  for (int i = 0; i < joint_weights.size(); ++i) {
    if (joint_weights(i) < 0.0) {
      throw std::invalid_argument("ConfigurationTask: joint_weights[" + std::to_string(i) +
                                  "] must be non-negative, got " +
                                  std::to_string(joint_weights(i)));
    }
  }

  // Pre-allocate storage: nv×nv Jacobian, nv error, and nv×nv H_dense
  const int nv = static_cast<int>(joint_weights.size());
  initializeStorage(nv, nv);
}

tl::expected<void, std::string> ConfigurationTask::computeError(const Scene& scene) {
  const auto& model = scene.getModel();
  const Eigen::VectorXd& q = scene.getCurrentJointPositions();

  // Validate dimensions
  if (target_q.size() != model.nq) {
    return tl::make_unexpected("ConfigurationTask: target_q size (" +
                               std::to_string(target_q.size()) + ") does not match model.nq (" +
                               std::to_string(model.nq) + ")");
  }

  // Compute difference in tangent space: error = target_q (-) q
  error_container = pinocchio::difference(model, q, target_q);

  return {};
}

tl::expected<void, std::string> ConfigurationTask::computeJacobian(const Scene& scene) {
  const auto& model = scene.getModel();

  // Validate joint_weights dimension against model
  if (joint_weights.size() != model.nv) {
    return tl::make_unexpected("ConfigurationTask: joint_weights size (" +
                               std::to_string(joint_weights.size()) +
                               ") does not match model.nv (" + std::to_string(model.nv) + ")");
  }

  // The Jacobian for configuration error is negative identity (-I) (nv × nv)
  // The negative sign matches the QP formulation: minimize ||J*dq + alpha*e||^2
  // With e = difference(q, target) pointing toward target and J = -I,
  // the optimal dq = -alpha * J^{-1} * e = alpha * e moves toward target.
  jacobian_container.setIdentity();
  jacobian_container *= -1.0;

  return {};
}

Eigen::MatrixXd ConfigurationTask::createWeightMatrix(const Eigen::VectorXd& joint_weights) {
  const int nv = static_cast<int>(joint_weights.size());
  Eigen::MatrixXd W = Eigen::MatrixXd::Zero(nv, nv);

  for (int i = 0; i < nv; ++i) {
    W(i, i) = std::sqrt(joint_weights(i));
  }

  return W;
}

}  // namespace roboplan
