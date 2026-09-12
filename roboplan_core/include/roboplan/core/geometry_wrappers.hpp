#pragma once

#include <filesystem>

// hppfcl was renamed to coal. Prefer the coal headers whenever they are available,
// since modern hpp-fcl releases declare `namespace coal { }` as a real namespace
// (with `hpp::fcl` as the backward-compatibility alias), which conflicts with
// declaring `namespace coal = hpp::fcl;` ourselves.
#if defined(__has_include) && __has_include(<coal/fwd.hh>)
#include <coal/BVH/BVH_model.h>
#include <coal/mesh_loader/loader.h>
#include <coal/octree.h>
#include <coal/shape/geometric_shapes.h>
#else
#include <hpp/fcl/BVH/BVH_model.h>
#include <hpp/fcl/mesh_loader/loader.h>
#include <hpp/fcl/octree.h>
#include <hpp/fcl/shape/geometric_shapes.h>
namespace coal = hpp::fcl;
#endif

namespace roboplan {

// NOTE: These are temporary structs to represent specific geometry objects.
// When Pinocchio and Coal release nanobind bindings, these can be replaced with the built-in types.

/// @brief Temporary wrapper struct to represent a box geometry.
struct Box {
  /// @brief Construct a Box object wrapper
  /// @param x The X dimension of the box.
  /// @param y The y dimension of the box.
  /// @param z The z dimension of the box.
  Box(double x, double y, double z) { geom_ptr = std::make_shared<coal::Box>(x, y, z); };

  /// @brief The underlying Coal box geometry.
  std::shared_ptr<coal::Box> geom_ptr;
};

/// @brief Temporary wrapper struct to represent a sphere geometry.
struct Sphere {
  /// @brief Construct a Sphere object wrapper
  /// @param radius The radius of the sphere.
  Sphere(double radius) { geom_ptr = std::make_shared<coal::Sphere>(radius); };

  /// @brief The underlying Coal sphere geometry.
  std::shared_ptr<coal::Sphere> geom_ptr;
};

/// @brief Temporary wrapper struct to represent a cylinder geometry (oriented along the Z axis).
struct Cylinder {
  /// @brief Construct a Cylinder object wrapper
  /// @param radius The radius of the cylinder.
  /// @param length The total length of the cylinder along its Z axis.
  Cylinder(double radius, double length) {
    geom_ptr = std::make_shared<coal::Cylinder>(radius, length);
  };

  /// @brief The underlying Coal cylinder geometry.
  std::shared_ptr<coal::Cylinder> geom_ptr;
};

/// @brief Temporary wrapper struct to represent a triangle mesh geometry loaded from a file.
struct Mesh {
  /// @brief Construct a Mesh object wrapper by loading from a mesh file (e.g. STL, OBJ, DAE).
  /// @param filename Path to the mesh file to load.
  /// @param scale Per-axis scale factors applied to the loaded mesh. Defaults to (1, 1, 1).
  Mesh(const std::filesystem::path& filename,
       const Eigen::Vector3d& scale = Eigen::Vector3d::Ones()) {
    coal::MeshLoader loader;
    geom_ptr = loader.load(filename.string(), scale);
  };

  /// @brief Construct a Mesh object wrapper from a pre-loaded Coal BVH model.
  Mesh(const std::shared_ptr<coal::BVHModelBase>& mesh_geom) { geom_ptr = mesh_geom; };

  /// @brief The underlying Coal BVH mesh geometry.
  std::shared_ptr<coal::BVHModelBase> geom_ptr;
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

    geom_ptr = std::make_shared<coal::OcTree>(octree);
  }

  OcTree(const std::shared_ptr<coal::OcTree>& octree_geom) { geom_ptr = octree_geom; }

  std::shared_ptr<coal::OcTree> geom_ptr;
};

}  // namespace roboplan
