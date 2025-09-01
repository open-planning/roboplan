#pragma once

#include <filesystem>
#include <iostream>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <tl/expected.hpp>

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
  /// @param yaml_config_path Path to the YAML configuration file with additional information.
  Scene(const std::string& name, const std::filesystem::path& urdf_path,
        const std::filesystem::path& srdf_path,
        const std::vector<std::filesystem::path>& package_paths =
            std::vector<std::filesystem::path>(),
        const std::filesystem::path& yaml_config_path = std::filesystem::path());

  /// @brief Gets the scene's name.
  /// @return The scene name.
  std::string getName() { return name_; };

  /// @brief Gets the scene's internal Pinocchio model.
  /// @return The Pinocchio model.
  pinocchio::Model getModel() { return model_; };

  /// @brief Gets the scene's joint names.
  /// @return A vector of joint names..
  std::vector<std::string> getJointNames() { return joint_names_; };

  /// @brief Gets the information for a specific joint.
  /// @param joint_name The name of the joint.
  /// @return The joint information struct.
  JointInfo getJointInfo(const std::string& joint_name) {
    if (!joint_info_.contains(joint_name)) {
      throw std::runtime_error("Joint '" + joint_name + "' is not in the scene.");
    }
    return joint_info_.at(joint_name);
  }

  /// @brief Gets the distance between two joint configurations.
  /// @param q_start The starting joint positions.
  /// @param q_end The ending joint positions.
  /// @return The configuration-space distance between the two positions.
  double configurationDistance(const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_end) const;

  /// @brief Sets the seed for the random number generator (RNG).
  /// @param seed The seed to set.
  void setRngSeed(unsigned int seed);

  /// @brief Generates random positions for the robot model.
  /// @return The random positions.
  Eigen::VectorXd randomPositions();

  /// @brief Generates random collision-free positions for the robot model.
  /// @param max_tries The maximum number of samples to attempt.
  /// @return The random positions, if successful, else std::nullopt.
  std::optional<Eigen::VectorXd> randomCollisionFreePositions(size_t max_samples = 1000);

  /// @brief Checks collisions at specified joint positions.
  /// @param q The joint positions.
  /// @return True if there are collisions, else false.
  bool hasCollisions(const Eigen::VectorXd& q) const;

  /// @brief Checks if the specified joint positions are valid with respect to joint limits.
  /// @param q The joint positions.
  /// @return True if the positions respect joint limits, else false.
  bool isValidPose(const Eigen::VectorXd& q) const;

  /// @brief Interpolates between two joint configurations.
  /// @param q_start The starting joint configuration.
  /// @param q_end The ending joint configuration.
  /// @param fraction The interpolation coefficient, between 0 and 1.
  Eigen::VectorXd interpolate(const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_end,
                              const double fraction) const;

  /// @brief Calculates forward kinematics for a specific frame.
  /// @param q The joint configuration.
  /// @param frame_name The name of the frame for which to perform forward kinematics.
  /// @return The 4x4 matrix denoting the transform of the specified frame.
  Eigen::Matrix4d forwardKinematics(const Eigen::VectorXd& q, const std::string& frame_name) const;

  /// @brief Prints basic information about the scene.
  friend std::ostream& operator<<(std::ostream& os, const Scene& scene);

  /// @brief Helper function to get the pinocchio ID of a frame through its name.
  /// @param name The name of the frame to look up.
  /// @return The pinocchio frame ID if successful, else a string describing the error.
  tl::expected<pinocchio::FrameIndex, std::string> getFrameId(const std::string& name) const;

private:
  /// @brief The name of the scene.
  std::string name_;

  /// @brief The Pinocchio model representing the robot and its environment.
  pinocchio::Model model_;

  /// @brief The default data structure for the underlying Pinocchio model.
  /// @details This won't be thread-safe unless each thread uses its own data.
  mutable pinocchio::Data model_data_;

  /// @brief The Pinocchio collision model representing the robot and its environment.
  pinocchio::GeometryModel collision_model_;

  /// @brief The default data structure for the underlying Pinocchio collision model.
  /// @details This won't be thread-safe unless each thread uses its own data.
  mutable pinocchio::GeometryData collision_model_data_;

  /// @brief The list of joint names in the model.
  std::vector<std::string> joint_names_;

  /// @brief Map from joint names to their corresponding information.
  std::map<std::string, JointInfo> joint_info_;

  /// @brief A random number generator for the scene.
  std::mt19937 rng_gen_;

  /// @brief The current state of the model (used to fill in partial states).
  JointConfiguration cur_state_;

  /// @brief Maps each frame name to its respective Pinocchio frame ID.
  std::unordered_map<std::string, pinocchio::FrameIndex> frame_map_;

  /// @brief Helper function to create a map of the robot's frame names to IDs.
  /// @param model The Pinocchio model.
  void createFrameMap(const pinocchio::Model& model);
};

}  // namespace roboplan
