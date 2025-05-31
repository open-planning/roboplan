#pragma once

#include <string>
#include <vector>

#include <Eigen/Dense>

namespace roboplan {

/// @brief Represents a robot joint configuration.
/// @details Creating and validating these structures are handled by separate
/// utility functions.
struct JointConfiguration {
  /// @brief The names of the joints.
  std::vector<std::string> joint_names;

  /// @brief The joint positions, in the same order as the names.
  Eigen::VectorXd positions;

  /// @brief The joint velocities, in the same order as the names.
  Eigen::VectorXd velocities;

  /// @brief The joint accelerations, in the same order as the names.
  Eigen::VectorXd accelerations;
};

/// @brief Represents a robot Cartesian configuration.
/// @details This comprises a transform, as well as the names of the frames in
/// the robot model.
struct CartesianConfiguration {
  /// @brief The name of the base (or reference) frame.
  std::string base_frame;

  /// @brief The name of the tip frame.
  std::string tip_frame;

  /// @brief The transformation matrix from the base to the tip frame.
  /// NOTE: I'd like this to be an Isometry3d but nanobind doesn't have off the
  /// shelf bindings for this.
  Eigen::Matrix4d tform = Eigen::Matrix4d::Identity();
};

} // namespace roboplan
