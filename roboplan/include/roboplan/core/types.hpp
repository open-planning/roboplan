#pragma once

#include <iostream>
#include <optional>
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

  /// @brief The name of the tip (or target) frame.
  std::string tip_frame;

  /// @brief The transformation matrix from the base to the tip frame.
  /// NOTE: I'd like this to be an Isometry3d but nanobind doesn't have off the
  /// shelf bindings for this.
  Eigen::Matrix4d tform = Eigen::Matrix4d::Identity();
};

/// @brief Enumeration that describes different types of joints.
enum JointType {
  UNKNOWN = 0,
  PRISMATIC = 1,
  REVOLUTE = 2,
  CONTINUOUS = 3,
  PLANAR = 4,
  FLOATING = 5,
};

/// @brief Contains joint limit information.
/// @details Values are all vectorized to denote multi-DOF joints.
struct JointLimits {
  /// @brief The minimum positions of the joint.
  Eigen::VectorXd min_position;

  /// @brief The maximum positions of the joint.
  Eigen::VectorXd max_position;

  /// @brief The maximum (symmetric) velocities of the joint.
  Eigen::VectorXd max_velocity;

  /// @brief The maximum (symmetric) accelerations of the joint.
  Eigen::VectorXd max_acceleration;

  /// @brief The maximum (symmetric) jerks of the joint.
  Eigen::VectorXd max_jerk;
};

/// @brief Contains joint mimic information.
struct JointMimicInfo {
  /// @brief The name of the joint being mimicked.
  std::string mimicked_joint_name;

  /// @brief The scaling factor for the mimic relationship.
  double scaling = 1.0;

  /// @brief The offset for the mimic relationship.
  double offset = 0.0;
};

/// @brief Contains joint information relevant to motion planning and control.
struct JointInfo {
  /// @brief Constructor for joint info.
  /// @param joint_type The type of the joint. All other variables will be
  /// initialized according to this value.
  JointInfo(JointType joint_type);

  /// @brief The type of the joint.
  JointType type;

  /// @brief The number of positional degrees of freedom.
  size_t num_position_dofs;

  /// @brief The number of velocity degrees of freedom.
  /// @details This also corresponds to higher derivatives like acceleration and jerk.
  size_t num_velocity_dofs;

  /// @brief The joint limit information for each degree of freedom.
  JointLimits limits;

  /// @brief The joint mimic information.
  std::optional<JointMimicInfo> mimic_info;
};

/// @brief Contains information about a named group of joints.
struct JointGroupInfo {
  /// @brief The joint names that make up the group.
  std::vector<std::string> joint_names;

  /// @brief The joint indices in the group.
  std::vector<size_t> joint_indices;

  /// @brief The position vector indices in the group.
  Eigen::VectorXi q_indices;

  /// @brief The velocity vector indices in the group.
  Eigen::VectorXi v_indices;

  /// @brief Whether the group has any continuous degrees of freedom.
  bool has_continuous_dofs{false};

  /// @brief The number of collapsed degrees of freedom.
  /// @details To get the full (expanded) value, this is q_indices.size().
  size_t nq_collapsed;

  /// @brief Prints basic information about the joint group.
  friend std::ostream& operator<<(std::ostream& os, const JointGroupInfo& info);
};

/// @brief Contains a path of joint configurations.
struct JointPath {
  /// @brief The list of joint names.
  std::vector<std::string> joint_names;

  /// @brief The list of joint configuration positions.
  std::vector<Eigen::VectorXd> positions;

  // TODO: Add higher-order terms as needed.

  /// @brief Prints basic information about the path.
  friend std::ostream& operator<<(std::ostream& os, const JointPath& path);
};

/// @brief Contains a trajectory of joint configurations.
struct JointTrajectory {
  /// @brief The list of joint names.
  std::vector<std::string> joint_names;

  /// @brief The list of times.
  std::vector<double> times;

  /// @brief The list of joint positions.
  std::vector<Eigen::VectorXd> positions;

  /// @brief The list of joint velocities.
  std::vector<Eigen::VectorXd> velocities;

  /// @brief The list of joint accelerations.
  std::vector<Eigen::VectorXd> accelerations;

  /// @brief Prints basic information about the trajectory.
  friend std::ostream& operator<<(std::ostream& os, const JointTrajectory& traj);
};

}  // namespace roboplan
