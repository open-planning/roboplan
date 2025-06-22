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
};

} // namespace roboplan
