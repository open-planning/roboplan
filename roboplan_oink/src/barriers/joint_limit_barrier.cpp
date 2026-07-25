#include <roboplan_oink/barriers/joint_limit_barrier.hpp>

#include <algorithm>
#include <cmath>
#include <vector>

namespace roboplan {

JointLimitBarrier::JointLimitBarrier(const Oink& oink, const Scene& scene,
                                     const std::string& group_name, double dt,
                                     double margin_fraction, double gain,
                                     double safe_displacement_gain)
    : Barrier(gain, dt, safe_displacement_gain, /*safety_margin=*/0.0),
      num_variables(oink.num_variables) {
  const auto maybe_group_info = scene.getJointGroupInfo(group_name);
  if (!maybe_group_info) {
    throw std::runtime_error("JointLimitBarrier: " + maybe_group_info.error());
  }
  const auto& group_info = maybe_group_info.value();

  // Walk the group's joints, accumulating position/velocity offsets, and collect the 1-DOF
  // joints with finite position limits. Multi-DOF joints (e.g. continuous joints) have no
  // meaningful box limits and are skipped.
  std::vector<int> q_indices_out;
  std::vector<int> v_slots_out;
  std::vector<double> lower_out;
  std::vector<double> upper_out;
  std::vector<double> margin_out;

  Eigen::Index q_offset = 0;
  Eigen::Index v_offset = 0;
  for (const auto& joint_name : group_info.joint_names) {
    const auto maybe_joint_info = scene.getJointInfo(joint_name);
    if (!maybe_joint_info) {
      throw std::runtime_error("JointLimitBarrier: " + maybe_joint_info.error());
    }
    const auto& joint_info = maybe_joint_info.value();
    const auto nq = static_cast<Eigen::Index>(joint_info.num_position_dofs);
    const auto nv = static_cast<Eigen::Index>(joint_info.num_velocity_dofs);

    if (nq == 1 && nv == 1 && q_offset < group_info.q_indices.size() &&
        v_offset < static_cast<Eigen::Index>(num_variables)) {
      const double lower = joint_info.limits.min_position[0];
      const double upper = joint_info.limits.max_position[0];
      const double span = upper - lower;
      if (std::isfinite(lower) && std::isfinite(upper) && span > 0.0) {
        // Clamp the margin so the shrunken box can never invert.
        const double joint_margin = std::clamp(margin_fraction * span, 0.0, 0.45 * span);
        q_indices_out.push_back(static_cast<int>(group_info.q_indices[q_offset]));
        v_slots_out.push_back(static_cast<int>(v_offset));
        lower_out.push_back(lower + joint_margin);
        upper_out.push_back(upper - joint_margin);
        margin_out.push_back(joint_margin);
      }
    }
    q_offset += nq;
    v_offset += nv;
  }

  const auto n = static_cast<Eigen::Index>(q_indices_out.size());
  q_index = Eigen::Map<const Eigen::VectorXi>(q_indices_out.data(), n);
  v_slot = Eigen::Map<const Eigen::VectorXi>(v_slots_out.data(), n);
  lower_bound = Eigen::Map<const Eigen::VectorXd>(lower_out.data(), n);
  upper_bound = Eigen::Map<const Eigen::VectorXd>(upper_out.data(), n);
  margin = Eigen::Map<const Eigen::VectorXd>(margin_out.data(), n);


  // Two rows per DOF: [lower_0, upper_0, lower_1, upper_1, ...]. The Jacobian rows are
  // constant selector rows: dh_lower/dq = +e_slot, dh_upper/dq = -e_slot.
  initializeStorage(static_cast<int>(2 * n), num_variables);
  jacobian_container.setZero();
  for (Eigen::Index k = 0; k < n; ++k) {
    jacobian_container(2 * k, v_slot[k]) = 1.0;
    jacobian_container(2 * k + 1, v_slot[k]) = -1.0;
  }
}

int JointLimitBarrier::getNumBarriers(const Scene& /*scene*/) const {
  return static_cast<int>(barrier_values.size());
}

tl::expected<void, std::string> JointLimitBarrier::computeBarrier(const Scene& scene) {
  // Clamp violated barrier values ("don't worsen" relaxation): when a joint is already
  // inside its margin zone, a negative h would make the CBF *demand* recovery each step,
  // which can conflict with the VelocityLimit constraint (QP infeasibility for slow joints)
  // or with converging the tool to a fixed terminal pose. Clamping h at zero only forbids
  // sinking deeper; actual recovery is left to the soft safe-displacement term, which
  // yields to the task when there is no slack.
  const Eigen::VectorXd& q = scene.getCurrentJointPositions();
  for (Eigen::Index k = 0; k < q_index.size(); ++k) {
    barrier_values[2 * k] = std::max(q[q_index[k]] - lower_bound[k], 0.0);
    barrier_values[2 * k + 1] = std::max(upper_bound[k] - q[q_index[k]], 0.0);
  }
  return {};
}

tl::expected<void, std::string> JointLimitBarrier::computeJacobian(const Scene& /*scene*/) {
  // The Jacobian rows are constant and set at construction.
  return {};
}

Eigen::VectorXd JointLimitBarrier::computeSafeDisplacement(const Scene& scene) const {
  Eigen::VectorXd dq_safe = Eigen::VectorXd::Zero(num_variables);
  const Eigen::VectorXd& q = scene.getCurrentJointPositions();
  for (Eigen::Index k = 0; k < q_index.size(); ++k) {
    // Only push once a joint is inside its margin zone (between the raw bound and the
    // margin-shrunken bound plus one extra margin of ramp), so the regularization is
    // inactive in the interior.
    const double lower_deficit = (lower_bound[k] + margin[k]) - q[q_index[k]];
    const double upper_deficit = q[q_index[k]] - (upper_bound[k] - margin[k]);
    if (lower_deficit > 0.0) {
      dq_safe[v_slot[k]] = std::min(lower_deficit, margin[k]);
    } else if (upper_deficit > 0.0) {
      dq_safe[v_slot[k]] = -std::min(upper_deficit, margin[k]);
    }
  }
  return dq_safe;
}

}  // namespace roboplan
