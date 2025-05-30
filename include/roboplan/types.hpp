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

} // namespace roboplan
