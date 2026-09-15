#pragma once

#include <string>
#include <vector>

#include <roboplan_mujoco/mujoco_joint_map.hpp>
#include <roboplan_mujoco/mujoco_simulation.hpp>

namespace roboplan {

/// @brief Maps RoboPlan configurations into a kinematic only MuJoCo simulation.
/// @details Unlike MujocoHardwareInterface, this interface writes joint positions directly and
/// does not actuate joints or step physics. It is intended for planner playback and other uses
/// analogous to a kinematic model visualizer.
class MujocoKinematicInterface {
public:
  /// @brief Creates an interface for a selected set of MuJoCo joints.
  /// @param simulation Simulation whose model and data will be owned by the interface.
  /// @param scene RoboPlan scene that defines joint names, indices, and encodings.
  /// @param mapped_joint_names Joints whose positions will be mapped to MuJoCo.
  /// @return A kinematic interface on success, else a string describing the invalid mapping.
  static tl::expected<MujocoKinematicInterface, std::string> create(MujocoSimulation simulation, const Scene& scene, const std::vector<std::string>& mapped_joint_names);

  MujocoKinematicInterface(const MujocoKinematicInterface&) = delete;
  MujocoKinematicInterface& operator=(const MujocoKinematicInterface&) = delete;
  MujocoKinematicInterface(MujocoKinematicInterface&&) noexcept = default;

  /// @brief Sets mapped positions from a full RoboPlan configuration.
  /// @details Writes MuJoCo `qpos`, zeros mapped velocities, applies mimic relationships,
  /// and calls `mj_forward` to update derived kinematic quantities.
  /// @param q Full RoboPlan configuration vector.
  /// @return Success, or a string describing an invalid configuration.
  tl::expected<void, std::string> setConfiguration(const Eigen::VectorXd& q);

  /// @brief Converts a RoboPlan configuration to a complete MuJoCo `qpos` vector.
  /// @details The simulation itself is not modified.
  /// @param q Full RoboPlan configuration vector.
  /// @return The converted MuJoCo configuration, or a string describing invalid input.
  tl::expected<Eigen::VectorXd, std::string> toMujocoQpos(const Eigen::VectorXd& q) const;

  /// @brief Returns the owned simulation.
  MujocoSimulation& simulation();

  /// @brief Returns the owned simulation.
  const MujocoSimulation& simulation() const;

private:
  /// @brief Constructs an interface from a validated joint map.
  MujocoKinematicInterface(MujocoSimulation simulation, MujocoJointMap joint_map);

  /// @brief Simulation used as the kinematic visualization state.
  MujocoSimulation simulation_;

  /// @brief Precomputed RoboPlan to MuJoCo joint representation mapping.
  MujocoJointMap joint_map_;
};
}  // namespace roboplan