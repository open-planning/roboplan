#pragma once

#include <hpp/fcl/shape/geometric_shapes.h>

namespace roboplan {

// NOTE: These are temporary structs to represent specific geometry objects.
// When Pinocchio and Coal release nanobind bindings, these can be replaced with the built-in types.

struct Box {
  /// @brief Construct a Box object wrapper
  /// @param x The X dimension of the box.
  /// @param y The y dimension of the box.
  /// @param z The z dimension of the box.
  Box(double x, double y, double z) { geom_ptr = std::make_shared<hpp::fcl::Box>(x, y, z); };

  /// @brief The underlying Coal box geometry.
  std::shared_ptr<hpp::fcl::Box> geom_ptr;
};

struct Sphere {
  /// @brief Construct a Sphere object wrapper
  /// @param radius The radius of the sphere.
  Sphere(double radius) { geom_ptr = std::make_shared<hpp::fcl::Sphere>(radius); };

  /// @brief The underlying Coal sphere geometry.
  std::shared_ptr<hpp::fcl::Sphere> geom_ptr;
};

}  // namespace roboplan
