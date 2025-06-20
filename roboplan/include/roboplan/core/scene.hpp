#pragma once

#include <filesystem>
#include <iostream>
#include <map>
#include <optional>
#include <string>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>

#include <roboplan/core/types.hpp>

namespace roboplan {
/// @brief Primary scene representation for planning and control.
class Scene {
public:
  /// @brief Basic constructor
  /// @param name The name of the scene.
  /// @param urdf_path Path to the URDF file.
  /// @param srdf_path Path to the SRDF file.
  /// @param package_paths A vector of package paths to look for packages.
  Scene(const std::string& name, const std::filesystem::path& urdf_path,
        const std::filesystem::path& srdf_path,
        const std::vector<std::filesystem::path>& package_paths =
            std::vector<std::filesystem::path>());

  /// @brief Gets the scene's internal Pinocchio model.
  /// @return The Pinocchio model.
  pinocchio::Model getModel() { return model_; };

  /// @brief Gets the scene's joint names.
  /// @return A vector of joint names..
  std::vector<std::string> getJointNames() { return joint_names_; };

  /// @brief Sets the seed for the random number generator (RNG).
  /// @param seed The seed to set.
  void setRngSeed(unsigned int seed);

  /// @brief Generates random positions for the robot model.
  /// @return The random positions.
  Eigen::VectorXd randomPositions();

  /// @brief Prints basic information about the scene.
  void print();

private:
  /// @brief The name of the scene.
  std::string name_;

  /// @brief The underlying Pinocchio model representing the robot and its
  /// environment.
  pinocchio::Model model_;

  /// @brief The list of joint names in the model.
  std::vector<std::string> joint_names_;

  /// @brief Map from joint names to their corresponding information.
  std::map<std::string, JointInfo> joint_info_;

  /// @brief A random number generator for the scene.
  std::mt19937 rng_gen_;

  /// @brief The current state of the model (used to fill in partial states).
  JointConfiguration cur_state_;
};

} // namespace roboplan
