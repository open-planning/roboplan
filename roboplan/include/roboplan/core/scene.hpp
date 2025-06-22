#pragma once

#include <filesystem>
#include <iostream>
#include <map>
#include <optional>
#include <string>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

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

  /// @brief Gets the scene's name.
  /// @return The scene name.
  std::string getName() { return name_; };

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

  /// @brief Checks collisions at specified joint positions.
  /// @param q The joint positions.
  /// @return True if there are collisions, else false.
  bool hasCollisions(const Eigen::VectorXd& q);

  /// @brief Checks collisions along a specified configuration space path.
  /// @param q_start The starting joint positions.
  /// @param q_end The ending joint positions.
  /// @param min_step_size The minimum configuration distance step size for interpolation.
  /// @return True if there are collisions, else false.
  bool hasCollisionsAlongPath(const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_end,
                              const double min_step_size);

  /// @brief Prints basic information about the scene.
  void print();

private:
  /// @brief The name of the scene.
  std::string name_;

  /// @brief The Pinocchio model representing the robot and its environment.
  pinocchio::Model model_;

  /// @brief The default data structure for the underlying Pinocchio model.
  /// @details This won't be thread-safe unless each thread uses its own data.
  pinocchio::Data model_data_;

  /// @brief The Pinocchio collision model representing the robot and its environment.
  pinocchio::GeometryModel collision_model_;

  /// @brief The default data structure for the underlying Pinocchio collision model.
  /// @details This won't be thread-safe unless each thread uses its own data.
  pinocchio::GeometryData collision_model_data_;

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
