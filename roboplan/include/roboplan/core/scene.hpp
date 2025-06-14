#pragma once

#include <filesystem>
#include <iostream>
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

  /// @brief Prints basic information about the scene.
  void print();

private:
  /// @brief The name of the scene.
  std::string name_;

  /// @brief The underlying Pinocchio model representing the robot and its
  /// environment.
  pinocchio::Model model_;

  /// @brief The current state of the model (used to fill in partial states).
  JointConfiguration cur_state_;
};

} // namespace roboplan
