#pragma once

#include <Eigen/Dense>

namespace roboplan {

/// @brief Defines a graph node for search-based planners.
struct Node {
  /// @brief The configuration (e.g., joint positions) of this node.
  Eigen::VectorXd config;

  /// @brief The parent node ID.
  /// @details -1 means this is the root node.
  int parent_id;

  /// @brief Constructor
  /// @param config_ The configuration of the node.
  /// @param parent_id_ The parent id of the node.
  Node(const Eigen::VectorXd& config_, int parent_id_) : config(config_), parent_id(parent_id_) {};
};

}  // namespace roboplan
