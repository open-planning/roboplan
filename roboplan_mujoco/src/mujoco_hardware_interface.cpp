#include <utility>

#include <roboplan_mujoco/mujoco_hardware_interface.hpp>

namespace roboplan {
MujocoHardwareInterface::MujocoHardwareInterface(MujocoSimulation simulation, MujocoJointMap joint_map)
    : simulation_(std::move(simulation)), joint_map_(std::move(joint_map)) {}

tl::expected<MujocoHardwareInterface, std::string> MujocoHardwareInterface::create(MujocoSimulation simulation, const Scene& scene, const std::vector<std::string>& controlled_joint_names) {
  auto joint_map = MujocoJointMap::create(scene, simulation.model(), controlled_joint_names);
  if (!joint_map) {
    return tl::unexpected(joint_map.error());
  }
  return MujocoHardwareInterface(std::move(simulation), std::move(joint_map.value()));
}

tl::expected<RoboPlanRobotState, std::string> MujocoHardwareInterface::readState() {
  return joint_map_.readState(simulation_.data());
}

tl::expected<void, std::string> MujocoHardwareInterface::writePositionCommand(const Eigen::VectorXd& q) {
  return joint_map_.writePositionCommand(q, simulation_.data());
}

tl::expected<void, std::string> MujocoHardwareInterface::reset(const Eigen::VectorXd& q) {
  simulation_.reset();
  auto result = joint_map_.setConfiguration(q, simulation_.data());
  if (!result) {
    return result;
  }

  auto command_result = joint_map_.writePositionCommand(q, simulation_.data());
  if (!command_result) {
    return command_result;
  }
  simulation_.forward();
  return {};
}

tl::expected<Eigen::VectorXd, std::string> MujocoHardwareInterface::toMujocoQpos(const Eigen::VectorXd& q) const {
  return joint_map_.toMujocoQpos(q, simulation_.data());
}

MujocoSimulation& MujocoHardwareInterface::simulation() { return simulation_; }
const MujocoSimulation& MujocoHardwareInterface::simulation() const { return simulation_; }
}  // namespace roboplan