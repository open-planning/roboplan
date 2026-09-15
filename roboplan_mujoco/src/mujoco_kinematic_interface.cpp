#include <utility>

#include <roboplan_mujoco/mujoco_kinematic_interface.hpp>

namespace roboplan {

MujocoKinematicInterface::MujocoKinematicInterface(MujocoSimulation simulation, MujocoJointMap joint_map)
    : simulation_(std::move(simulation)), joint_map_(std::move(joint_map)) {}

tl::expected<MujocoKinematicInterface, std::string> MujocoKinematicInterface::create(MujocoSimulation simulation, const Scene& scene, const std::vector<std::string>& mapped_joint_names) {
  auto joint_map = MujocoJointMap::create(scene, simulation.model(), mapped_joint_names, false);
  if (!joint_map) {
    return tl::unexpected(joint_map.error());
  }
  return MujocoKinematicInterface(std::move(simulation), std::move(joint_map.value()));
}

tl::expected<void, std::string> MujocoKinematicInterface::setConfiguration(const Eigen::VectorXd& q) {
  auto result = joint_map_.setConfiguration(q, simulation_.data());
  if (!result) {
    return result;
  }
  simulation_.forward();
  return {};
}

tl::expected<Eigen::VectorXd, std::string> MujocoKinematicInterface::toMujocoQpos(const Eigen::VectorXd& q) const {
  return joint_map_.toMujocoQpos(q, simulation_.data());
}

MujocoSimulation& MujocoKinematicInterface::simulation() { return simulation_; }

const MujocoSimulation& MujocoKinematicInterface::simulation() const { return simulation_; }

}  // namespace roboplan