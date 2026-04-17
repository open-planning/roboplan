#pragma once

#include <hpp/fcl/octree.h>
#include <hpp/fcl/shape/geometric_shapes.h>

namespace roboplan {

// NOTE: These are temporary structs to represent specific geometry objects.
// When Pinocchio and Coal release nanobind bindings, these can be replaced with the built-in types.

/// @brief Temporary wrapper struct to represent a box geometry.
struct Box {
  /// @brief Construct a Box object wrapper
  /// @param x The X dimension of the box.
  /// @param y The y dimension of the box.
  /// @param z The z dimension of the box.
  Box(double x, double y, double z) { geom_ptr = std::make_shared<hpp::fcl::Box>(x, y, z); };

  /// @brief The underlying Coal box geometry.
  std::shared_ptr<hpp::fcl::Box> geom_ptr;
};

/// @brief Temporary wrapper struct to represent a sphere geometry.
struct Sphere {
  /// @brief Construct a Sphere object wrapper
  /// @param radius The radius of the sphere.
  Sphere(double radius) { geom_ptr = std::make_shared<hpp::fcl::Sphere>(radius); };

  /// @brief The underlying Coal sphere geometry.
  std::shared_ptr<hpp::fcl::Sphere> geom_ptr;
};

struct OcTree {
  OcTree(const std::vector<Eigen::Matrix<double, 6, 1>>& boxes, const double resolution) {
    auto octree = std::make_shared<octomap::OcTree>(resolution);

    if (!boxes.empty()) {
      const double thresh = boxes[0][5];
      octree->setOccupancyThres(thresh);
    }

    for (const auto& box : boxes) {
      octree->updateNode(box[0], box[1], box[2],    // x, y and z coordinates
                         octomap::logodds(box[4]),  // occupancy of cell
                         true                       // enable lazy update
      );
    }

    octree->updateInnerOccupancy();

    geom_ptr = std::make_shared<hpp::fcl::OcTree>(octree);
  }

  OcTree(const std::shared_ptr<hpp::fcl::OcTree>& octree_geom) { geom_ptr = octree_geom; }

  std::shared_ptr<hpp::fcl::OcTree> geom_ptr;
};

}  // namespace roboplan
