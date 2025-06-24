#pragma once

#include <dynotree/KDTree.h>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>

namespace roboplan {

using CombinedStateSpace = dynotree::Combined<double>;

struct RRTOptions {

  /// @brief The maximum connection distance.
  double max_connection_distance = 1.0;

  /// @brief The configuration-space step size for collision checking along edges.
  double collision_check_step_size = 0.05;
};

class RRT {
public:
  /// @brief Constructor.
  /// @param scene The scene to use for motion planning.
  /// @param options A struct containing RRT options.
  RRT(const Scene& scene, const RRTOptions& options);

  /// @brief Plan a path from start to goal.
  /// @param start The starting joint configuration.
  /// @param goal The goal joint configuration.
  void plan(const JointConfiguration& start, const JointConfiguration& goal);

private:
  /// @brief The scene. (should this be a pointer?)
  Scene scene_;

  /// @brief The struct containing IK solver options.
  RRTOptions options_;

  /// @brief A state space for the k-d tree for nearest neighbor lookup.
  CombinedStateSpace state_space_;

  /// @brief A k-d tree for nearest neighbor lookup.
  dynotree::KDTree<int, -1, 32, double, CombinedStateSpace> tree_;
};

}  // namespace roboplan
