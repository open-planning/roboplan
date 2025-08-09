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

std::ostream& operator<<(std::ostream& os, const JointPath& path) {
  std::cout << "Joint Path with " << path.positions.size() << " points:\n";
  for (size_t idx = 0; idx < path.positions.size(); ++idx) {
    const auto& pos = path.positions.at(idx);
    std::cout << "  " << (idx + 1) << ": " << pos.transpose() << "\n";
  }
  return os;
}

std::ostream& operator<<(std::ostream& os, const JointTrajectory& traj) {
  std::cout << "Joint Trajectory with " << traj.times.size() << " points:\n";
  for (size_t idx = 0; idx < traj.times.size(); ++idx) {
    const auto& t = traj.times.at(idx);
    const auto& pos = traj.positions.at(idx);
    std::cout << "  [t=" << t << "]\n"
              << "    q: " << pos.transpose() << "\n";
  }
  return os;
}

}  // namespace roboplan
