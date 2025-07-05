#pragma once

#include <memory>
#include <optional>
#include <random>

#include <dynotree/KDTree.h>
#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>
#include <roboplan_rrt/graph.hpp>

namespace roboplan {

using CombinedStateSpace = dynotree::Combined<double>;

struct RRTOptions {
  /// @brief The maximum number of nodes to sample.
  size_t max_nodes = 1000;

  /// @brief The maximum configuration distance between two nodes.
  double max_connection_distance = 3.0;

  /// @brief The configuration-space step size for collision checking along edges.
  double collision_check_step_size = 0.05;

  /// @brief The probability of sampling the goal node instead of a random node.
  /// @details Must be between 0 and 1.
  double goal_biasing_probability = 0.15;

  /// @brief The maximum amount of time to allow for planning, in seconds.
  /// @details If <= 0 then planning will never timeout.
  double max_planning_time = 0;
};

class RRT {
public:
  /// @brief Constructor.
  /// @param scene A pointer to the scene to use for motion planning.
  /// @param options A struct containing RRT options.
  RRT(const std::shared_ptr<Scene> scene, const RRTOptions& options = RRTOptions());

  /// @brief Plan a path from start to goal.
  /// @param start The starting joint configuration.
  /// @param goal The goal joint configuration.
  /// @return A joint-space path, if planning succeeds, else std::nullopt.
  std::optional<JointPath> plan(const JointConfiguration& start, const JointConfiguration& goal);

  /// @brief Sets the seed for the random number generator (RNG).
  /// @details For reproducibility, this also seeds the underlying scene.
  /// For now, this means it would break multi-threaded applications.
  /// @param seed The seed to set.
  void setRngSeed(unsigned int seed);

private:
  /// @brief Runs the RRT extend operation from a start node to a goal node.
  /// @param q_start The start configuration.
  /// @param q_goal The goal configuration.
  /// @param max_connection_dist The maximum configuration distance to extend to.
  /// @return The extended configuration.
  Eigen::VectorXd extend(const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_goal,
                         double max_connection_dist);

  /// @brief A pointer to the scene.
  std::shared_ptr<Scene> scene_;

  /// @brief The struct containing IK solver options.
  RRTOptions options_;

  /// @brief A state space for the k-d tree for nearest neighbor lookup.
  CombinedStateSpace state_space_;

  /// @brief A k-d tree for nearest neighbor lookup.
  dynotree::KDTree<int, -1, 32, double, CombinedStateSpace> kd_tree_;

  /// @brief A list of sampled nodes.
  std::vector<Node> nodes_;

  /// @brief A random number generator for the planner.
  std::mt19937 rng_gen_;

  /// @brief A uniform distribution for goal biasing sampling.
  std::uniform_real_distribution<double> uniform_dist_{0.0, 1.0};
};

}  // namespace roboplan
