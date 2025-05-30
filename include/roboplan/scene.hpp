#pragma once

#include <filesystem>
#include <iostream>
#include <optional>
#include <string>

#include <pinocchio/algorithm/geometry.hpp>

#include <roboplan/types.hpp>

namespace roboplan {
/// @brief Primary scene representation for planning and control.
class Scene {
public:
  /// @brief Basic constructor
  /// @param urdf_path Path to the URDF file.
  /// @param srdf_path Path to the SRDF file.
  Scene(const std::filesystem::path& urdf_path,
        const std::filesystem::path& srdf_path);

  /// @brief Prints basic information
  void print();

private:
  pinocchio::Model model_;

  JointConfiguration cur_state_;
};

} // namespace roboplan
