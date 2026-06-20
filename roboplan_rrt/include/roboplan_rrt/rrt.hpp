#pragma once

#include <memory>
#include <optional>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include <dynotree/KDTree.h>
#include <tl/expected.hpp>

#include <roboplan/core/collision_context.hpp>
#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>
#include <roboplan_rrt/graph.hpp>

namespace roboplan {

using CombinedStateSpace = dynotree::Combined<double>;
using KdTree = dynotree::KDTree<int, -1, 32, double, CombinedStateSpace>;

/// @brief Options struct for RRT planner.
struct RRTOptions {
  /// @brief The joint group name to be used by the planner.
  std::string group_name = "";

  /// @brief The maximum number of nodes to sample.
  size_t max_nodes = 1000;

  /// @brief The maximum configuration distance between two nodes.
  double max_connection_distance = 3.0;

  /// @brief The configuration-space step size for collision checking along edges.
  double collision_check_step_size = 0.05;

  /// @brief If true, uses bisection instead of linear search for collision checking along edges.
  bool collision_check_use_bisection = true;

  /// @brief The probability of sampling the goal node instead of a random node.
  /// @details Must be between 0 and 1.
  double goal_biasing_probability = 0.15;

  /// @brief The maximum amount of time to allow for planning, in seconds.
  /// @details If <= 0 then planning will never timeout.
  double max_planning_time = 0;

  /// @brief If true, use the RRT-Connect algorithm to grow the search trees.
  bool rrt_connect = false;

  /// @brief If true, use the RRT* algorithm to grow asymptotically optimal trees.
  /// @details As new nodes are added, RRT* picks the lowest-cost parent among nearby nodes and
  /// rewires nearby nodes through the new node when that lowers their cost. Unlike plain RRT, it
  /// does not stop at the first solution: it keeps sampling and rewiring until the node or time
  /// budget is exhausted, then returns the lowest-cost path found. Compatible with `rrt_connect`,
  /// in which case both trees are rewired.
  bool rrt_star = false;

  /// @brief The configuration-space radius used to find neighbors for RRT* rewiring.
  /// @details Only used when `rrt_star` is true. Expressed in the same units as
  /// `max_connection_distance`, and should generally be at least that large so that neighbors a
  /// single connection step away are considered. Larger values consider more neighbors when
  /// choosing parents and rewiring, improving path quality at the cost of more collision checks.
  double rewire_distance = 5.0;

  /// @brief If true, return as soon as the first path is found; if false, keep planning until the
  /// node or time budget is exhausted and return the lowest-cost path found.
  /// @details Applies to every mode. With RRT* (`rrt_star`), set this to false to obtain the
  /// asymptotically optimal behavior; with plain RRT or RRT-Connect, setting it to false simply
  /// keeps the cheapest path discovered across the whole budget.
  bool fast_return = true;
};

/// @brief Motion planner based on the Rapidly-exploring Random Tree (RRT) algorithm.
class RRT {
public:
  /// @brief Constructor.
  /// @param scene A pointer to the scene to use for motion planning.
  /// @param options A struct containing RRT options.
  RRT(const std::shared_ptr<Scene> scene, const RRTOptions& options);

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

  /// @brief Attempt to add node(s) to the provided tree and node set, growing toward `q_sample`.
  /// @param tree The tree to grow.
  /// @param nodes The set of sampled nodes so far.
  /// @param q_sample The configuration to extend towards (or connect to).
  /// @param collision_context This plan's private collision context, used for all collision checks.
  /// @param greedy If true (the RRT-Connect CONNECT step), repeatedly extend toward `q_sample`
  /// until it is reached or an obstacle is hit. If false (a single EXTEND step), add at most one
  /// node, one `max_connection_distance` step toward `q_sample`.
  /// @return True if node(s) were added to the tree, false otherwise.
  bool growTree(KdTree& tree, std::vector<Node>& nodes, const Eigen::VectorXd& q_sample,
                const CollisionContext& collision_context, bool greedy);

  /// @brief Attempts to connect the `target_tree` to the latest added node in `nodes`.
  /// @details The "latest added node" refers to `nodes.back()`. The function will identify the
  /// nearest node in the target_tree, and attempt to make a connection. If successful, will
  /// return a path from the start node to the goal node.
  /// @param nodes The list of source tree nodes, `nodes.back()` is the most recently added node.
  /// @param target_tree The tree to connect to the nodes list.
  /// @param target_nodes The nodes in the target tree.
  /// @param grow_start_tree If true, the target_tree is the goal tree.
  /// @param collision_context This plan's private collision context, used for all collision checks.
  /// @return If a path is found, a pair of the completed start-to-goal path and its total
  /// cost-to-come (the two connected nodes' costs plus the connecting edge length); otherwise none.
  /// The cost is only meaningful when the planner tracks node costs (RRT*, or any mode with
  /// fast_return disabled); callers returning the first path can ignore it.
  std::optional<std::pair<JointPath, double>> joinTrees(const std::vector<Node>& nodes,
                                                        const KdTree& target_tree,
                                                        const std::vector<Node>& target_nodes,
                                                        bool grow_start_tree,
                                                        const CollisionContext& collision_context);

  /// @brief Returns a path from the specified index to the first added node.
  /// @param nodes The list of nodes in the tree.
  /// @param end_node The index of the goal destination in the nodes list.
  /// @return A JointPath between end_node and nodes[0].
  JointPath getPath(const std::vector<Node>& nodes, const Node& end_node);

  /// @brief Returns the start and goal trees' node vectors, for visualization purposes.
  std::pair<std::vector<Node>, std::vector<Node>> getNodes() { return {start_nodes_, goal_nodes_}; }

private:
  /// @brief Runs the RRT extend operation from a start node to a goal node.
  /// @param q_start The start configuration.
  /// @param q_goal The goal configuration.
  /// @param max_connection_dist The maximum configuration distance to extend to.
  /// @return The extended configuration.
  Eigen::VectorXd extend(const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_goal,
                         double max_connection_dist);

  /// @brief Finds the IDs of all nodes in a tree within `rewire_distance` (in configuration
  /// distance) of a configuration.
  /// @details Used by the RRT* variant to gather candidate parents and rewiring targets. Only nodes
  /// already inserted into `tree` are returned, so calling this before inserting the query node
  /// excludes it from the result.
  /// @param tree The tree to search.
  /// @param nodes The nodes backing `tree`, used to measure configuration distance to candidates.
  /// @param q The query configuration, as full (model-sized) joint positions.
  /// @return The IDs of the neighboring nodes.
  std::vector<int> findNearNodes(const KdTree& tree, const std::vector<Node>& nodes,
                                 const Eigen::VectorXd& q) const;

  /// @brief Inserts a node into an RRT* tree, choosing the best parent and rewiring its neighbors.
  /// @details The RRT* insertion step: among the new node's neighbors (within `rewire_distance`) it
  /// picks the collision-free parent that minimizes the new node's cost-to-come, inserts the node,
  /// then reconnects any neighbor through the new node when that lowers the neighbor's cost-to-come
  /// (propagating the change through that neighbor's subtree).
  /// @param kd_tree The tree to insert into.
  /// @param nodes The nodes backing `kd_tree`.
  /// @param q_new The configuration to insert. Must already be validated as collision-free.
  /// @param default_parent_id The node `q_new` was extended from, used as the fallback parent.
  /// @param collision_context This plan's private collision context, used for all collision checks.
  /// @return The ID of the newly inserted node.
  int rewire(KdTree& kd_tree, std::vector<Node>& nodes, const Eigen::VectorXd& q_new,
             int default_parent_id, const CollisionContext& collision_context);

  /// @brief Propagates an RRT* cost change down a node's subtree.
  /// @details Call after a node's parent and cost have been updated by a rewire. Each descendant's
  /// cost-to-come is recomputed from its (already updated) parent's cost plus the edge length.
  /// @param nodes The list of nodes in the tree.
  /// @param root_id The node whose cost just changed.
  void propagateCost(std::vector<Node>& nodes, int root_id);

  /// @brief Collapses group joint positions to the k-d tree state space coordinates.
  /// @details Thin wrapper around `collapseContinuousJointPositions` that throws on failure, so the
  /// tree operations can call it without repeating the error handling at each call site.
  /// @param q_group The group joint positions, in expanded (original) coordinates.
  /// @return The collapsed configuration used for nearest-neighbor lookups.
  Eigen::VectorXd collapse(const Eigen::VectorXd& q_group) const;

  /// @brief A pointer to the scene.
  std::shared_ptr<Scene> scene_;

  /// @brief The struct containing IK solver options.
  RRTOptions options_;

  /// @brief The joint group info for the IK solver.
  JointGroupInfo joint_group_info_;

  /// @brief A state space for the k-d tree for nearest neighbor lookup.
  CombinedStateSpace state_space_;

  /// @brief The runtime dimension of the (collapsed) k-d tree state space.
  /// @details Cached at construction because `CombinedStateSpace::get_runtime_dim()` is not
  /// const-qualified in the vendored dynotree, so it cannot be called from the const
  /// `findNearNodes` where the dimension is needed.
  int state_dim_ = 0;

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
