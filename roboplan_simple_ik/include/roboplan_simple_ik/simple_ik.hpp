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
  double max_time = 0.005;

  /// @brief Maximum number of restarts until success.
  size_t max_restarts = 2;

  /// @brief The integration step for the solver.
  double step_size = 0.25;

  /// @brief Damping value for the Jacobian pseudoinverse.
  double damping = 0.001;

  /// @brief The maximum linear error norm, in meters.
  double max_linear_error_norm = 0.001;

  /// @brief The maximum angular error norm, in radians.
  double max_angular_error_norm = 0.001;

  /// @brief Whether to check collisions.
  bool check_collisions = true;

  /// @brief If true, returns when the first ik solution is found.
  /// @details Otherwise the entire time budget will be consumed to attempt to find
  /// a solution that is closest to the starting configuration.
  bool fast_return = true;
};

/// @brief Simple inverse kinematics (IK) solver based on the Jacobian pseudoinverse.
class SimpleIk {
public:
  /// @brief Constructor.
  /// @param scene A pointer to the scene to use for solving IK.
  /// @param options A struct containing IK solver options.
  SimpleIk(const std::shared_ptr<Scene> scene, const SimpleIkOptions& options);

  /// @brief Solves inverse kinematics (single goal).
  /// @details This just calls the multiple goal version internally.
  /// @param goal The goal Cartesian configuration.
  /// @param start The starting joint configuration. (should be optional)
  /// @param solution The IK solution, as a joint configuration.
  /// @return Whether the IK solve succeeded.
  bool solveIk(const CartesianConfiguration& goal, const JointConfiguration& start,
               JointConfiguration& solution) {
    return solveIk(std::vector<CartesianConfiguration>{goal}, start, solution);
  }

  /// @brief Solves inverse kinematics (multiple goal).
  /// @param goals The goal Cartesian configurations.
  /// @param start The starting joint configuration. (should be optional)
  /// @param solution The IK solution, as a joint configuration.
  /// @return Whether the IK solve succeeded.
  bool solveIk(const std::vector<CartesianConfiguration>& goals, const JointConfiguration& start,
               JointConfiguration& solution);

  /// @brief Sets the seed used to draw restart configurations.
  /// @param seed The seed to set.
  void setRngSeed(unsigned int seed);

private:
  /// @brief Re-solves the current step so joints that would cross their position limits land
  /// exactly on their bound while the remaining joints take up the task.
  /// @param q The current full configuration.
  /// @details Joint position limits are enforced by saturation: joints whose step would cross a
  /// limit are pinned exactly on the bound and removed from the task Jacobian, then the damped
  /// least-squares step is re-solved so the remaining joints take up the task. This keeps every
  /// iterate within the limits while preserving a descent direction for the pose error (a hard
  /// clamp instead turns limit faces into attractors that stall convergence). Only single-DOF
  /// joints participate; multi-DOF joints (e.g., planar or continuous joints, whose position
  /// representation is larger than their velocity representation) are not limit-enforced.
  /// @return False if a re-solve produced NaN, leaving `vel_` unusable.
  bool saturateStep(const Eigen::VectorXd& q);

  /// @brief A pointer to the scene.
  std::shared_ptr<Scene> scene_;

  /// @brief The struct containing IK solver options.
  SimpleIkOptions options_;

  /// @brief Random number generator used to seed each solve's SceneContext.
  std::mt19937 rng_gen_;

  /// @brief The joint group info for the IK solver.
  JointGroupInfo joint_group_info_;

  /// @brief Lower position limits for the joint group, aligned with `q_indices`.
  Eigen::VectorXd lower_position_limits_;

  /// @brief Upper position limits for the joint group, aligned with `q_indices`.
  Eigen::VectorXd upper_position_limits_;

  /// @brief Group position slot enforced by each group velocity DOF, or -1 for DOFs exempt from
  /// limit handling (multi-DOF joints such as planar or continuous joints).
  Eigen::VectorXi limit_q_slot_;

  /// @brief The bound each saturated DOF lands on this iteration; NaN when unsaturated (for
  /// allocating memory once).
  Eigen::VectorXd saturation_bound_;

  /// @brief The joint group's velocities for the step being solved, carried from the damped
  /// least-squares solve through the saturation re-solves (for allocating memory once).
  Eigen::VectorXd group_vel_;

  /// @brief The group Jacobian with saturated columns zeroed (for allocating memory once).
  Eigen::MatrixXd task_jacobian_;

  /// @brief The task error including saturated joints' contributions (for allocating memory
  /// once).
  Eigen::VectorXd task_error_;

  /// @brief The full error vector (for allocating memory once).
  Eigen::VectorXd error_;

  /// @brief The full model Jacobian (for allocating memory once).
  Eigen::MatrixXd full_jacobian_;

  /// @brief The derivative of log6 at a frame's pose error (for allocating memory once).
  Eigen::Matrix<double, 6, 6> Jlog_;

  /// @brief The joint group's Jacobian (for allocating memory once).
  Eigen::MatrixXd jacobian_;

  /// @brief The Jacobian times Jacobian transpose (for allocating memory once).
  Eigen::MatrixXd jjt_;

  /// @brief The full joint velocity vector for integrating (for allocating memory once).
  Eigen::VectorXd vel_;
};

}  // namespace roboplan
