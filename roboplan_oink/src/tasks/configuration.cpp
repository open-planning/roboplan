#include <roboplan_oink/tasks/configuration.hpp>

#include <pinocchio/algorithm/joint-configuration.hpp>

namespace roboplan {

ConfigurationTask::ConfigurationTask(const Oink& oink, const Eigen::VectorXd& target_q,
                                     const Eigen::VectorXd& joint_weights,
                                     const ConfigurationTaskOptions& options)
    : Task(createWeightMatrix(joint_weights), options.task_gain, options.lm_damping),
      target_q(target_q), joint_weights(joint_weights), q_indices(oink.q_indices),
      v_indices(oink.v_indices) {
  // Validate joint weights size matches group DOF count
  if (joint_weights.size() != oink.num_variables) {
    throw std::invalid_argument(
        "ConfigurationTask: joint_weights size (" + std::to_string(joint_weights.size()) +
        ") does not match oink.num_variables (" + std::to_string(oink.num_variables) + ")");
  }
  // Validate joint weights are non-negative
  for (int i = 0; i < joint_weights.size(); ++i) {
    if (joint_weights(i) < 0.0) {
      throw std::invalid_argument("ConfigurationTask: joint_weights[" + std::to_string(i) +
                                  "] must be non-negative, got " +
                                  std::to_string(joint_weights(i)));
    }
  }

  // Pre-allocate storage: nv_group×nv_group Jacobian, nv_group error, nv_group×nv_group H_dense
  const int nv = oink.num_variables;
  initializeStorage(nv, nv);
}

tl::expected<void, std::string> ConfigurationTask::computeError(const Scene& scene) {
  const auto& model = scene.getModel();
  const Eigen::VectorXd& q = scene.getCurrentJointPositions();

  // Validate target_q size against the group's position indices
  if (target_q.size() != q_indices.size()) {
    return tl::make_unexpected(
        "ConfigurationTask: target_q size (" + std::to_string(target_q.size()) +
        ") does not match group q_indices size (" + std::to_string(q_indices.size()) + ")");
  }

  // Build full-robot target: start from current q (non-group joints stay put),
  // then overwrite the group's positions with the desired target.
  Eigen::VectorXd q_target_full = q;
  q_target_full(q_indices) = target_q;

  // Compute difference in full tangent space, then extract group joints via v_indices
  const Eigen::VectorXd full_diff = pinocchio::difference(model, q, q_target_full);
  error_container = full_diff(v_indices);

  return {};
}

tl::expected<void, std::string> ConfigurationTask::computeJacobian(const Scene& /*scene*/) {
  // The Jacobian for configuration error is negative identity (-I) (nv_group × nv_group)
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
