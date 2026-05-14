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

  // We use num_velocity_dofs even for position limits because any continuous position DOFs
  // should anyway be collapsed down to the tangent space.
  limits.min_position =
      Eigen::VectorXd::Constant(num_velocity_dofs, std::numeric_limits<double>::lowest());
  limits.max_position =
      Eigen::VectorXd::Constant(num_velocity_dofs, std::numeric_limits<double>::max());
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
    os << " " << name;
  }
  os << "\n";
  os << "  q indices: " << info.q_indices.transpose() << "\n";
  os << "  v indices: " << info.v_indices.transpose() << "\n";
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

std::ostream& operator<<(std::ostream& os, const CartesianPath& path) {
  os << "Cartesian Path with " << path.tip_frames.size() << " frame(s):\n";

  for (size_t frame_idx = 0; frame_idx < path.tip_frames.size(); ++frame_idx) {
    os << " frame " << frame_idx << ":\n";

    if (frame_idx < path.base_frames.size()) {
      os << "  base_frame: " << path.base_frames.at(frame_idx) << "\n";
    }

    os << "  tip_frame: " << path.tip_frames.at(frame_idx) << "\n";

    if (frame_idx >= path.tforms.size()) {
      os << "  no transforms\n";
      continue;
    }

    for (size_t path_idx = 0; path_idx < path.tforms.at(frame_idx).size(); ++path_idx) {
      os << "  [idx=" << path_idx << "]\n";
      os << path.tforms.at(frame_idx).at(path_idx) << "\n";
    }
  }

  return os;
}

std::ostream& operator<<(std::ostream& os, const CartesianTrajectory& traj) {
  os << "Cartesian Trajectory with " << traj.times.size() << " point(s)";
  os << " and " << traj.tip_frames.size() << " frame(s):\n";

  for (size_t frame_idx = 0; frame_idx < traj.tip_frames.size(); ++frame_idx) {
    os << " frame " << frame_idx << ":\n";

    if (frame_idx < traj.base_frames.size()) {
      os << "  base_frame: " << traj.base_frames.at(frame_idx) << "\n";
    }

    os << "  tip_frame: " << traj.tip_frames.at(frame_idx) << "\n";

    if (frame_idx >= traj.tforms.size()) {
      os << "  no transforms\n";
      continue;
    }

    for (size_t time_idx = 0; time_idx < traj.times.size(); ++time_idx) {
      os << "  [t=" << traj.times.at(time_idx) << "]\n";

      if (time_idx < traj.tforms.at(frame_idx).size()) {
        os << traj.tforms.at(frame_idx).at(time_idx) << "\n";
      } else {
        os << "  missing transform\n";
      }
    }
  }

  return os;
}

}  // namespace roboplan
