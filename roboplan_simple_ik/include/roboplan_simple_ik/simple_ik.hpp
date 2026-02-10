#pragma once

#include <memory>
#include <string>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>

namespace roboplan {

/// @brief Options struct for simple IK solver.
struct SimpleIkOptions {
  /// @brief The joint group name to be used by the solver.
  std::string group_name = "";

  /// @brief Max iterations for one try of the solver.
  size_t max_iters = 100;

  /// @brief Max total computation time, in seconds.
  double max_time = 0.01;

  /// @brief Maximum number of restarts until success.
  size_t max_restarts = 2;

  /// @brief The integration step for the solver.
  double step_size = 0.01;

  /// @brief Damping value for the Jacobian pseudoinverse.
  double damping = 0.001;

  /// @brief The maximum error norm.
  /// TODO: Separate into linear and angular?
  double max_error_norm = 0.001;

  /// @brief Whether to check collisions.
  bool check_collisions = true;
};

/// @brief Simple inverse kinematics (IK) solver based on the Jacobian pseudoinverse.
class SimpleIk {
public:
  /// @brief Constructor.
  /// @param scene A pointer to the scene to use for solving IK.
  /// @param options A struct containing IK solver options.
  SimpleIk(const std::shared_ptr<Scene> scene, const SimpleIkOptions& options);

  /// @brief Solves inverse kinematics.
  /// @param goal The goal Cartesian configuration.
  /// @param start The starting joint configuration. (should be optional)
  /// @param solution The IK solution, as a joint configuration.
  /// @return Whether the IK solve succeeded.
  bool solveIk(const CartesianConfiguration& goal, const JointConfiguration& start,
               JointConfiguration& solution);

private:
  /// @brief A pointer to the scene.
  std::shared_ptr<Scene> scene_;

  /// @brief The struct containing IK solver options.
  SimpleIkOptions options_;

  /// @brief Pinocchio data for the IK solver.
  pinocchio::Data data_;

  /// @brief The joint group info for the IK solver.
  JointGroupInfo joint_group_info_;

  /// @brief The full model Jacobian (for allocating memory once).
  Eigen::MatrixXd full_jacobian_;

  /// @brief The joint group's Jacobian (for allocating memory once).
  Eigen::MatrixXd jacobian_;

  /// @brief The Jacobian times Jacobian transpose (for allocating memory once).
  Eigen::MatrixXd jjt_;

  /// @brief The full joint velocity vector for integrating (for allocating memory once).
  Eigen::VectorXd vel_;
};

}  // namespace roboplan
