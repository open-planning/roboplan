#pragma once

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>

namespace roboplan {

struct SimpleIkOptions {

  /// @brief Max iterations for one try of the solver.
  size_t max_iters = 1000;

  /// @brief The integration step for the solver.
  double step_size = 0.01;

  /// @brief Damping value for the Jacobian pseudoinverse.
  double damping = 0.001;

  /// @brief The maximum error norm.
  /// TODO: Separate into linear and angular?
  double max_error_norm = 0.001;
};

class SimpleIk {
public:
  /// @brief Constructor.
  /// @param scene The scene to use for solving IK.
  /// @param options A structcontaining IK solver options.
  SimpleIk(const Scene& scene, const SimpleIkOptions& options);

  /// @brief Solves inverse kinematics.
  /// @param goal The goal Cartesian configuration.
  /// @param start The starting joint configuration. (should be optional)
  /// @param solution The IK solution, as a joint configuration.
  /// @return Whether the IK solve succeeded.
  bool solveIk(const CartesianConfiguration& goal, const JointConfiguration& start,
               JointConfiguration& solution);

private:
  /// @brief The scene. (should this be a pointer?)
  Scene scene_;

  /// @brief The struct containing IK solver options.
  SimpleIkOptions options_;
};

} // namespace roboplan
