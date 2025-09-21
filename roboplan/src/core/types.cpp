#include <exception>
#include <limits>

#include <roboplan/core/types.hpp>

namespace roboplan {

JointInfo::JointInfo(JointType joint_type) : type{joint_type} {

  switch (type) {
  case (JointType::PRISMATIC):
  case (JointType::REVOLUTE):
    num_position_dofs = 1;
    num_velocity_dofs = 1;
    break;
  case (JointType::CONTINUOUS):
    num_position_dofs = 2;
    num_velocity_dofs = 1;
    break;
  case (JointType::PLANAR):
    num_position_dofs = 4;
    num_velocity_dofs = 3;
    break;
  case (JointType::FLOATING):
    num_position_dofs = 7;
    num_velocity_dofs = 6;
    break;
  default:
    throw std::runtime_error("Got invalid joint type.");
  }

  limits.min_position =
      Eigen::VectorXd::Constant(num_position_dofs, std::numeric_limits<double>::lowest());
  limits.max_position =
      Eigen::VectorXd::Constant(num_position_dofs, std::numeric_limits<double>::max());
  limits.max_velocity =
      Eigen::VectorXd::Constant(num_velocity_dofs, std::numeric_limits<double>::max());
  limits.max_acceleration =
      Eigen::VectorXd::Constant(num_velocity_dofs, std::numeric_limits<double>::max());
  limits.max_jerk =
      Eigen::VectorXd::Constant(num_velocity_dofs, std::numeric_limits<double>::max());
};

std::ostream& operator<<(std::ostream& os, const JointGroupInfo& info) {
  os << "Joint group with " << info.joint_names.size() << " joints:\n";
  os << "  Names:";
  for (const auto& name : info.joint_names) {
    std::cout << " " << name;
  }
  os << "\n";
  os << "  q indices:";
  for (const auto q_idx : info.q_indices) {
    std::cout << " " << q_idx;
  }
  os << "\n";
  os << "  v indices:";
  for (const auto v_idx : info.v_indices) {
    std::cout << " " << v_idx;
  }
  os << "\n";
  return os;
}

std::ostream& operator<<(std::ostream& os, const JointPath& path) {
  os << "Joint Path with " << path.positions.size() << " points:\n";
  for (size_t idx = 0; idx < path.positions.size(); ++idx) {
    const auto& pos = path.positions.at(idx);
    os << "  " << (idx + 1) << ": " << pos.transpose() << "\n";
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const JointTrajectory& traj) {
  os << "Joint Trajectory with " << traj.times.size() << " points:\n";
  for (size_t idx = 0; idx < traj.times.size(); ++idx) {
    const auto& t = traj.times.at(idx);
    const auto& pos = traj.positions.at(idx);
    os << "  [t=" << t << "]\n"
       << "    q: " << pos.transpose() << "\n";
  }
  return os;
}

}  // namespace roboplan
