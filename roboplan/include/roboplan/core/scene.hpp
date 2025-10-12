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

#include <roboplan/core/geometry_wrappers.hpp>
#include <roboplan/core/types.hpp>

namespace roboplan {

/// @brief Returns the contents of a file as a string.
/// @param name path The path to the file.
std::string readFile(const std::filesystem::path& path);

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

  /// @brief Basic constructor with pre-parsed URDF and SRDF options.
  /// @param name The name of the scene.
  /// @param urdf XML String of the URDF.
  /// @param srdf XML String of the SRDF.
  /// @param package_paths A vector of package paths to look for packages.
  /// @param yaml_config_path Path to the YAML configuration file with additional information.
  Scene(const std::string& name, const std::string& urdf, const std::string& srdf,
        const std::vector<std::filesystem::path>& package_paths =
            std::vector<std::filesystem::path>(),
        const std::filesystem::path& yaml_config_path = std::filesystem::path());

  /// @brief Gets the scene's name.
  /// @return The scene name.
  std::string getName() { return name_; };

  /// @brief Gets the scene's internal Pinocchio model.
  /// @return The Pinocchio model.
  pinocchio::Model getModel() { return model_; };

  /// @brief Gets the scene's full joint names, including mimic joints.
  /// @return A vector of joint names.
  std::vector<std::string> getJointNames() const { return joint_names_; };

  /// @brief Gets the scene's actuated (non-mimic) joint names.
  /// @return A vector of joint names.
  std::vector<std::string> getActuatedJointNames() const { return actuated_joint_names_; };

  /// @brief Gets the information for a specific joint.
  /// @param joint_name The name of the joint.
  /// @return The joint information struct if successful, else a string describing the error.
  tl::expected<JointInfo, std::string> getJointInfo(const std::string& joint_name) const;

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

  /// @brief Applies mimic joint relationships to a position vector.
  /// @param q The joint positions.
  void applyMimics(Eigen::VectorXd& q) const;

  /// @brief Converts partial joint positions to full joint positions.
  /// @details This includes adding new joints and applying mimic relationships.
  /// @param group_name The name of the joint group.
  /// @param q The original (partial) joint positions.
  /// @return The full joint positions.
  Eigen::VectorXd toFullJointPositions(const std::string& group_name,
                                       const Eigen::VectorXd& q) const;

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

  /// @brief Get the Pinocchio model ID of a frame by its name.
  /// @param name The name of the frame to look up.
  /// @return The Pinocchio frame ID if successful, else a string describing the error.
  tl::expected<pinocchio::FrameIndex, std::string> getFrameId(const std::string& name) const;

  /// @brief Get the joint group information of a scene by its name.
  /// @param name The name of the joint group to look up.
  /// @return The joint group information if successful, else a string describing the error.
  tl::expected<JointGroupInfo, std::string> getJointGroupInfo(const std::string& name) const;

  /// @brief Get the current joint positions for the full robot state.
  /// @return The current joint position vector.
  Eigen::VectorXd getCurrentJointPositions() const { return cur_state_.positions; }

  /// @brief Set the joint positions for the full robot state.
  /// @return The desired joint position vector.
  void setJointPositions(const Eigen::VectorXd& positions) { cur_state_.positions = positions; }

  /// @brief Get the joint position indices for a set of joint names.
  /// @param joint_names The joint names for which to look up position indices.
  /// @return The corresponding joint position indices.
  Eigen::VectorXi getJointPositionIndices(const std::vector<std::string>& joint_names) const;

  /// @brief Adds a box geometry to the scene.
  /// @param name The name of the object to add.
  /// @param parent_frame The name of the parent frame to add the object to.
  /// @param box The box geometry instance to add.
  /// @param tform The transform between the parent frame and the geometry.
  /// @param color The color of the geometry, in RGBA vector format.
  /// @return Void if successful, else a string describing the error.
  tl::expected<void, std::string> addBoxGeometry(const std::string& name,
                                                 const std::string& parent_frame, const Box& box,
                                                 const Eigen::Matrix4d& tform,
                                                 const Eigen::Vector4d& color);

  /// @brief Adds a sphere geometry to the scene.
  /// @param name The name of the object to add.
  /// @param parent_frame The name of the parent frame to add the object to.
  /// @param sphere The sphere geometry instance to add.
  /// @param tform The transform between the parent frame and the geometry.
  /// @param color The color of the geometry, in RGBA vector format.
  /// @return Void if successful, else a string describing the error.
  tl::expected<void, std::string>
  addSphereGeometry(const std::string& name, const std::string& parent_frame, const Sphere& sphere,
                    const Eigen::Matrix4d& tform, const Eigen::Vector4d& color);

  /// @brief Adds a Pinocchio geometry object to the scene.
  /// @details This can be made the sole public entrypoint to add a geometry once
  /// Pinocchio and Coal have working nanobind bindings compatible with this library.
  /// @param geom_obj The geometry object instance to add.
  /// @return Void if successful, else a string describing the error.
  tl::expected<void, std::string> addGeometry(const pinocchio::GeometryObject& geom_obj);

  /// @brief Updates the placement of an object geometry in the scene.
  /// @param name The name of the object to update.
  /// @param parent_frame The parent frame of the transformation.
  /// @param tform The transform between the parent frame and the geometry.
  tl::expected<void, std::string> updateGeometryPlacement(const std::string& name,
                                                          const std::string& parent_frame,
                                                          Eigen::Matrix4d& tform);

  /// @brief Removes a geometry from the scene.
  /// @param name The name of the object to remove.
  tl::expected<void, std::string> removeGeometry(const std::string& name);

  /// @brief Prints basic information about the scene.
  friend std::ostream& operator<<(std::ostream& os, const Scene& scene);

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

  /// @brief The full list of joint names in the model (including mimic joints).
  std::vector<std::string> joint_names_;

  /// @brief The list of actuated (non-mimic) joint names in the model.
  std::vector<std::string> actuated_joint_names_;

  /// @brief Map from joint names to their corresponding information.
  std::unordered_map<std::string, JointInfo> joint_info_map_;

  /// @brief Map from joint group names to their corresponding information.
  std::unordered_map<std::string, JointGroupInfo> joint_group_info_map_;

  /// @brief A random number generator for the scene.
  std::mt19937 rng_gen_;

  /// @brief The current state of the model (used to fill in partial states).
  JointConfiguration cur_state_;

  /// @brief Maps each frame name to its respective Pinocchio frame ID.
  std::unordered_map<std::string, pinocchio::FrameIndex> frame_map_;

  /// @brief Maps each added collision geometry to its respective Pinocchio geometry ID.
  std::unordered_map<std::string, pinocchio::FrameIndex> collision_geometry_map_;
};

}  // namespace roboplan
