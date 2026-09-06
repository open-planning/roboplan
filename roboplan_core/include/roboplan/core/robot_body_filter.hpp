#pragma once

#include <cstddef>
#include <memory>
#include <optional>
#include <vector>

#include <roboplan/core/scene.hpp>

namespace roboplan {

// The transform type was renamed from Transform3f (hpp-fcl) to Transform3s (coal). Pick the
// right one for whichever library the coal namespace resolves to (see geometry_wrappers.hpp).
#if defined(__has_include) && __has_include(<coal/fwd.hh>)
using CoalTransform = coal::Transform3s;
#else
using CoalTransform = coal::Transform3f;
#endif

/// @brief The test used by RobotBodyFilter to classify points near the robot geometry.
enum class RobotBodyFilterMethod {
  /// Exact: after the broadphase AABB cull, each candidate point is checked with a Coal
  /// narrowphase collision query (point vs. geometry, with the padding as the security margin).
  /// Exact for every geometry type, including meshes, at the cost of one GJK/BVH query per
  /// candidate point.
  NARROWPHASE,
  /// Conservative: after the broadphase AABB cull, each candidate point is checked against the
  /// geometry's padded local AABB (an oriented box in world frame). Much faster since it is a
  /// few arithmetic operations per candidate, but over-removes points near the corners of the
  /// oriented boxes. The set of points it removes is always a superset of NARROWPHASE's.
  PADDED_OBB,
};

/// @brief Options struct for the robot body filter.
struct RobotBodyFilterOptions {
  /// @brief Distance, in meters, around the robot's collision geometry within which points are
  /// considered part of the robot body. Must be non-negative.
  double padding = 0.05;

  /// @brief The classification test to use.
  RobotBodyFilterMethod method = RobotBodyFilterMethod::NARROWPHASE;

  /// @brief Number of threads used to classify points, or 0 to use all hardware threads. The
  /// points are split into blocks that the threads pull from a shared queue, so at most one
  /// thread per block is ever spawned and small clouds are processed serially either way.
  size_t num_threads = 0;
};

/// @brief Filters points that lie on or near the robot's own collision geometry.
///
/// This is the usual "self filter" that removes the robot's body from a sensor point cloud (or
/// the occupied cells of an octree) before the cloud is turned into a collision object, so that
/// the robot does not see itself as an obstacle.
///
/// Both methods share a broadphase stage that culls points against the padded world-frame AABB of
/// every robot collision geometry at the query configuration; they differ only in the exactness
/// (and cost) of the test run on the surviving candidates. See RobotBodyFilterMethod.
///
/// @par Thread safety
/// The filter owns private Pinocchio scratch over the Scene's immutable robot description, so
/// distinct filters may run concurrently on one Scene, but a single filter must not be shared
/// across threads. The robot geometry is snapshotted at construction; objects added to (or
/// removed from) the scene afterwards do not affect it.
class RobotBodyFilter {
public:
  /// @brief Points are given as an N x 3 row-major matrix (one point per row, matching the
  /// default NumPy layout so the bindings can map arrays without copying).
  using PointMatrix = Eigen::Matrix<double, Eigen::Dynamic, 3, Eigen::RowMajor>;

  /// @brief Boolean mask over points; true means the point belongs to the robot body.
  using Mask = Eigen::Matrix<bool, Eigen::Dynamic, 1>;

  /// @brief Constructs a filter over the scene's current robot collision geometry.
  /// @param scene The scene whose robot geometry to filter against.
  /// @param options The filter options.
  RobotBodyFilter(const std::shared_ptr<Scene>& scene, const RobotBodyFilterOptions& options);

  /// @brief Classifies each point against the padded robot geometry at a joint configuration.
  /// @param q The joint configuration at which to place the robot (size model.nq).
  /// @param points The points to classify, one per row, in world frame.
  /// @param extra_padding Optional per-point padding, in meters, added to the configured padding.
  /// Useful when the points stand in for finite-sized cells (e.g. octree leaves), in which case
  /// passing each cell's half-diagonal keeps the test conservative.
  /// @return A mask sized to the number of points; true marks a point within the padded body.
  Mask computeMask(const Eigen::VectorXd& q, const Eigen::Ref<const PointMatrix>& points,
                   const std::optional<Eigen::VectorXd>& extra_padding = std::nullopt);

  /// @brief Returns only the points outside the padded robot body at a joint configuration.
  /// @param q The joint configuration at which to place the robot (size model.nq).
  /// @param points The points to filter, one per row, in world frame.
  /// @param extra_padding Optional per-point padding, in meters, added to the configured padding.
  /// @return The rows of `points` whose computeMask() entry is false, in their original order.
  PointMatrix filterPoints(const Eigen::VectorXd& q, const Eigen::Ref<const PointMatrix>& points,
                           const std::optional<Eigen::VectorXd>& extra_padding = std::nullopt);

  /// @brief The filter options.
  const RobotBodyFilterOptions& getOptions() const { return options_; }

private:
  /// @brief The scene whose robot description (model and collision geometry) is filtered against.
  std::shared_ptr<Scene> scene_;

  /// @brief The filter options.
  RobotBodyFilterOptions options_;

  /// @brief A snapshot of the robot's own collision geometries (the ones loaded from the URDF,
  /// excluding objects added to the scene), so later scene edits cannot invalidate the filter.
  pinocchio::GeometryModel robot_geom_model_;

  /// @brief Private kinematics scratch for placing the robot geometry at the query configuration.
  pinocchio::Data data_;

  /// @brief Private geometry scratch holding the world placements of robot_geom_model_.
  pinocchio::GeometryData robot_geom_data_;

  /// @brief Per-geometry scratch recomputed by computeMask() at each query configuration.
  struct GeometryScratch {
    /// @brief World-frame rotation and translation of the geometry.
    Eigen::Matrix3d rotation;
    Eigen::Vector3d translation;
    /// @brief Center and half extents of the geometry's local AABB, in the geometry frame.
    Eigen::Vector3d local_center;
    Eigen::Vector3d local_half_extents;
    /// @brief World-frame AABB of the placed geometry, already padded by the configured padding.
    Eigen::Vector3d aabb_min;
    Eigen::Vector3d aabb_max;
    /// @brief The underlying Coal geometry and its placement, for narrowphase queries.
    const coal::CollisionGeometry* geometry;
    CoalTransform transform;
  };
  std::vector<GeometryScratch> geom_scratch_;
};

}  // namespace roboplan
