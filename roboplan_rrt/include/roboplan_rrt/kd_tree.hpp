#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include <Eigen/Dense>

namespace roboplan {

/// @brief Incremental k-d tree over joint positions, for nearest-neighbor lookup in tree planners.
/// @details The metric is the product of R^1 and SO(2) factors, which is the scene's configuration
/// distance for revolute, prismatic, and continuous joints.
class KdTree {
public:
  KdTree();
  ~KdTree();

  /// @brief Empties the tree and sets its state space.
  /// @param circular Per coordinate, true if it is an SO(2) angle in [-pi, pi].
  /// @param capacity The number of points for which to reserve space.
  void reset(const std::vector<bool>& circular, size_t capacity = 0);

  /// @brief Adds a point, whose ID is the number of points inserted before it.
  void addPoint(const Eigen::VectorXd& x);

  /// @brief Returns the ID of the nearest point to `x`, or -1 if the tree is empty.
  int nearest(const Eigen::VectorXd& x) const;

  /// @brief Returns the IDs of all points within `radius` of `x`.
  std::vector<int> withinRadius(const Eigen::VectorXd& x, double radius) const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace roboplan
