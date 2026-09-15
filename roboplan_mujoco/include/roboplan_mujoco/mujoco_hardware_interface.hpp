#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <tl/expected.hpp>

#include <roboplan_hardware_interface/hardware_interface.hpp>
#include <roboplan_mujoco/mujoco_joint_map.hpp>
#include <roboplan_mujoco/mujoco_simulation.hpp>

namespace roboplan {
/// @brief Position control hardware interface backed by MuJoCo simulation.
/// @details Owns the simulation and translates between RoboPlan/Pinocchio state vectors and
/// MuJoCo state and actuator arrays. Only the selected joints are measured and commanded;
/// unmapped RoboPlan positions retain their reference values, while unmapped velocities and
/// efforts are reported as zero.
class MujocoHardwareInterface final : public HardwareInterface {
public:
  /// @brief Creates an interface for a selected set of actuated MuJoCo joints.
  /// @details Validates that each RoboPlan joint has a compatible MuJoCo representation and a unit gear position servo.
  /// @param simulation Simulation whose model and data will be owned by the interface.
  /// @param scene RoboPlan scene that defines joint names, indices, and encodings.
  /// @param controlled_joint_names Joints whose state and position commands are mapped.
  /// @return A hardware interface on success, else a string describing the invalid mapping.
  static tl::expected<MujocoHardwareInterface, std::string> create(MujocoSimulation simulation, const Scene& scene, const std::vector<std::string>& controlled_joint_names);

  MujocoHardwareInterface(const MujocoHardwareInterface&) = delete;
  MujocoHardwareInterface& operator=(const MujocoHardwareInterface&) = delete;
  MujocoHardwareInterface(MujocoHardwareInterface&&) noexcept = default;

  /// @brief Reads the current MuJoCo state in RoboPlan/Pinocchio coordinates.
  /// @return The full model position, velocity, and actuator effort vectors.
  tl::expected<RoboPlanRobotState, std::string> readState() override;

  /// @brief Sends a full RoboPlan configuration to the mapped position actuators.
  /// @details Continuous and planar angles are unwrapped to the nearest equivalent angle
  /// to avoid discontinuous actuator targets. Unmapped entries are ignored.
  /// @param q Full RoboPlan configuration vector of size `scene.getModel().nq`.
  /// @return Success, or a string describing an invalid configuration.
  tl::expected<void, std::string> writePositionCommand(const Eigen::VectorXd& q) override;

  /// @brief Resets the simulation to a full RoboPlan configuration.
  /// @details Resets MuJoCo data, writes mapped positions and matching actuator targets,
  /// zeros mapped velocities, and runs forward dynamics.
  /// @param q Full RoboPlan configuration vector.
  /// @return Success, or a string describing an invalid configuration.
  tl::expected<void, std::string> reset(const Eigen::VectorXd& q);

  /// @brief Converts a RoboPlan configuration to a complete MuJoCo `qpos` vector.
  /// @details Starts from the simulation's current `qpos`, overwrites mapped joints, and
  /// applies mimic relationships. The simulation itself is not modified.
  /// @param q Full RoboPlan configuration vector.
  /// @return The converted MuJoCo configuration, or a string describing invalid input.
  tl::expected<Eigen::VectorXd, std::string> toMujocoQpos(const Eigen::VectorXd& q) const;

  /// @brief Returns the owned simulation.
  MujocoSimulation& simulation();

  /// @brief Returns the owned simulation.
  const MujocoSimulation& simulation() const;

private:
  /// @brief Constructs the interface from a valid joint mapping.
  MujocoHardwareInterface(MujocoSimulation simulation, MujocoJointMap joint_map);

  /// @brief Simulation receives commands and returns measured state.
  MujocoSimulation simulation_;

  /// @brief Precomputed RoboPlan to MuJoCo joint representation mapping.
  MujocoJointMap joint_map_;
};
}  // namespace roboplan