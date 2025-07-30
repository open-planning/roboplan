#pragma once

#include <memory>
#include <optional>
#include <random>
#include <vector>

#include <dynotree/KDTree.h>
#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>
#include <roboplan_rrt/graph.hpp>
#include <tl/expected.hpp>

namespace roboplan {

using CombinedStateSpace = dynotree::Combined<double>;
using KdTree = dynotree::KDTree<int, -1, 32, double, CombinedStateSpace>;

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

  /// @brief If true, use the RRT-Connect algorithm to grow the search trees.
  bool rrt_connect = false;
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
  /// @return A joint-space path, if planning succeeds, otherwise an error message.
  tl::expected<JointPath, std::string> plan(const JointConfiguration& start,
                                            const JointConfiguration& goal);

  /// @brief Sets the seed for the random number generator (RNG).
  /// @details For reproducibility, this also seeds the underlying scene.
  /// For now, this means it would break multi-threaded applications.
  /// @param seed The seed to set.
  void setRngSeed(unsigned int seed);

  /// @brief Initializes the search tree with the specified start pose.
  /// @param tree Reference to an empty tree.
  /// @param nodes Reference to the nodes vector.
  /// @param q_init The first node to add to the tree.
  /// @param max_size The maximum size of the tree.
  void initializeTree(KdTree& tree, std::vector<Node>& nodes, const Eigen::VectorXd& q_init,
                      size_t max_size = 1000);

  /// @brief Attempt to add a sampled node to the provided tree and node set.
  /// @param tree The tree to grow.
  /// @param nodes The set of sampled nodes so far.
  /// @param q_sample Randomly sampled node to extend towards (or connect).
  /// @return True if node(s) were added to the tree, false otherwise.
  bool growTree(KdTree& tree, std::vector<Node>& nodes, const Eigen::VectorXd& q_sample);

  /// @brief Attempts to connect the `target_tree` to the latest added node in `nodes`.
  /// @details The "latest added node" refers to `nodes.back()`. The function will identify the
  /// nearest node in the target_tree, and attempt to make a connection. If successful, will
  /// return a path from the start node to the goal node.
  /// @param nodes The list of source tree nodes, `nodes.back()` is the most recently added node.
  /// @param target_tree The tree to connect to the nodes list.
  /// @param target_nodes The nodes in the target tree.
  /// @param grow_start_tree If true, the target_tree is the goal tree.
  /// @return A completed path from the start to the goal node if it exists, otherwise none.
  std::optional<JointPath> joinTrees(const std::vector<Node>& nodes, const KdTree& target_tree,
                                     const std::vector<Node>& target_nodes, bool grow_start_tree);

  /// @brief Returns a path from the specified index to the first added node.
  /// @param nodes The list of nodes in the tree.
  /// @param end_node The index of the goal destination in the nodes list.
  /// @return A JointPath between end_node and nodes[0].
  JointPath getPath(const std::vector<Node>& nodes, const Node& end_node);

  /// @brief Returns the start and goal node vectors, for visualization purposes.
  std::pair<std::vector<Node>, std::vector<Node>> getNodes() { return {start_nodes_, goal_nodes_}; }

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

  /// @brief A random number generator for the planner.
  std::mt19937 rng_gen_;

  /// @brief A uniform distribution for goal biasing sampling.
  std::uniform_real_distribution<double> uniform_dist_{0.0, 1.0};

  /// @brief The start tree nodes.
  std::vector<Node> start_nodes_;

  /// @brief The goal tree nodes.
  std::vector<Node> goal_nodes_;
};

}  // namespace roboplan
