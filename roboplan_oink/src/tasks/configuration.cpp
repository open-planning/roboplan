#include <roboplan_oink/tasks/configuration.hpp>

#include <pinocchio/algorithm/joint-configuration.hpp>

namespace roboplan {

ConfigurationTask::ConfigurationTask(const Oink& oink, const Eigen::VectorXd& target_q,
                                     const Eigen::VectorXd& joint_weights,
                                     const ConfigurationTaskOptions& options)
    : Task(options.priority, createWeightMatrix(joint_weights), options.task_gain,
           options.lm_damping),
      target_q(target_q), joint_weights(joint_weights), q_indices(oink.q_indices),
      v_indices(oink.v_indices) {
  if (joint_weights.size() != oink.num_variables) {
    throw std::invalid_argument(
        "ConfigurationTask: joint_weights size (" + std::to_string(joint_weights.size()) +
        ") does not match oink.num_variables (" + std::to_string(oink.num_variables) + ")");
  }
  for (int i = 0; i < joint_weights.size(); ++i) {
    if (joint_weights(i) < 0.0) {
      throw std::invalid_argument("ConfigurationTask: joint_weights[" + std::to_string(i) +
                                  "] must be non-negative, got " +
                                  std::to_string(joint_weights(i)));
    }
  }

  const int nv = oink.num_variables;
  initializeStorage(nv, nv);
}

void ConfigurationTask::setTargetConfiguration(const Eigen::VectorXd& target) {
  if (target.size() != q_indices.size()) {
    throw std::invalid_argument(
        "ConfigurationTask::setTargetConfiguration: target size (" + std::to_string(target.size()) +
        ") does not match group q_indices size (" + std::to_string(q_indices.size()) + ")");
  }
  target_q = target;
}

tl::expected<void, std::string> ConfigurationTask::computeError(const SceneContext& context) {
  const auto& model = context.getModel();
  const Eigen::VectorXd& q = context.getJointPositions();

  if (target_q.size() != q_indices.size()) {
    return tl::make_unexpected(
        "ConfigurationTask: target_q size (" + std::to_string(target_q.size()) +
        ") does not match group q_indices size (" + std::to_string(q_indices.size()) + ")");
  }

  // Full-robot target: non-group joints stay at their current positions.
  Eigen::VectorXd q_target_full = q;
  q_target_full(q_indices) = target_q;

  // Difference in the full tangent space, restricted to the group's velocity indices.
  const Eigen::VectorXd full_diff = pinocchio::difference(model, q, q_target_full);
  error_container = full_diff(v_indices);

  return {};
}

tl::expected<void, std::string>
ConfigurationTask::computeJacobian(const SceneContext& /*context*/) {
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
