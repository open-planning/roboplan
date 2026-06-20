#pragma once

#include <vector>

#include <Eigen/Dense>

namespace roboplan {

/// @brief Defines a graph node for search-based planners.
struct Node {
  /// @brief The configuration (e.g., joint positions) of this node.
  Eigen::VectorXd config;

  /// @brief The parent node ID.
  /// @details -1 means this is the root node.
  int parent_id;

  /// @brief The cost-to-come from the tree root to this node.
  /// @details Only maintained by the RRT* variant; left at zero by the other planners.
  double cost;

  /// @brief The IDs of this node's children in the tree.
  /// @details Maintained by the RRT* variant so cost changes can be propagated through a node's
  /// subtree after a rewire. Left empty by the other planners.
  std::vector<int> children;

  /// @brief Constructor
  /// @param config_ The configuration of the node.
  /// @param parent_id_ The parent id of the node.
  /// @param cost_ The cost-to-come from the tree root to this node.
  Node(const Eigen::VectorXd& config_, int parent_id_, double cost_ = 0.0)
      : config(config_), parent_id(parent_id_), cost(cost_) {};
};

}  // namespace roboplan
