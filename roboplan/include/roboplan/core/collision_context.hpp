#pragma once

#include <optional>
#include <vector>

#include <Eigen/Dense>
#include <pinocchio/collision/broadphase-manager.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

// The concrete broadphase manager (AABB tree).
#if defined(__has_include) && __has_include(<coal/fwd.hh>)
#include <coal/broadphase/broadphase_dynamic_AABB_tree.h>
#else
#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h>
namespace coal = hpp::fcl;
#endif

namespace roboplan {

class Scene;

/// @brief Per-consumer scratch space for collision queries (Data + GeometryData + broadphase tree).
/// @details A Scene's collision-query path mutates shared scratch state (joint/frame placements,
/// the geometry world transforms, and the broadphase AABB tree), so a single Scene cannot answer
/// collision queries from multiple threads concurrently. A CollisionContext owns its own copy of
/// that scratch over the Scene's *immutable* Model and GeometryModel (shared by pointer), so an
/// algorithm such as the RRT can run its collision queries without contending with anything else.
///
/// A context is a snapshot of the scene's collision geometry at construction time. If the scene
/// geometry changes, discard the context and build a new one; it does not auto-sync.
class CollisionContext {
public:
  /// @brief Snapshots the current collision geometry of `scene`.
  explicit CollisionContext(const Scene& scene);

  // Non-copyable and non-movable: the broadphase manager caches a raw pointer to `geom_data_`, so
  // the object's address must remain stable for its whole lifetime. Hold one behind a pointer
  // (e.g. std::unique_ptr) if it needs to be relocated or rebuilt.
  CollisionContext(const CollisionContext&) = delete;
  CollisionContext& operator=(const CollisionContext&) = delete;
  CollisionContext(CollisionContext&&) = delete;
  CollisionContext& operator=(CollisionContext&&) = delete;

  /// @brief Checks collisions at `q`, stopping at the first collision. Uses this context's own
  /// scratch, so it is safe to call concurrently with queries on other contexts or the Scene.
  bool hasCollisions(const Eigen::VectorXd& q) const;

  /// @brief Refreshes geometry placements at `q` and computes the distance for every active
  /// collision pair into this context's own GeometryData.
  /// @details Runs on this context's private scratch, so it is safe to call concurrently with
  /// queries on other contexts or the Scene. Results are available via getCollisionData().
  ///
  /// @param broadphase_margin Broadphase cull distance. Pairs whose world axis-aligned bounding
  /// boxes are farther apart than this are skipped: their (cheap) AABB-gap lower bound is stored as
  /// the distance and their witness points are collapsed to the origin, so any Jacobian built from
  /// them is a zero row. Such pairs are, by construction, farther than the margin and cannot be the
  /// binding constraint of a barrier whose minimum distance is well inside it. The exact
  /// narrow-phase distance and witness points are still computed for every pair within the margin.
  /// Pass std::nullopt (the default) to disable culling and compute the exact distance for every
  /// pair (equivalent to pinocchio::computeDistances).
  void computeDistances(const Eigen::VectorXd& q,
                        std::optional<double> broadphase_margin = std::nullopt) const;

  /// @brief Computes the joint Jacobians at `q` into this context's own Data, so consumers that
  /// build task/barrier Jacobians (e.g. self-collision) do not have to touch the Scene's scratch.
  /// Results are available via getData().
  void computeJointJacobians(const Eigen::VectorXd& q) const;

  /// @brief The immutable model shared with the originating Scene.
  const pinocchio::Model& getModel() const { return model_; }

  /// @brief This context's private Data scratch (forward kinematics, joint Jacobians).
  pinocchio::Data& getData() const { return data_; }

  /// @brief The immutable collision geometry model shared with the originating Scene.
  const pinocchio::GeometryModel& getCollisionModel() const { return collision_model_; }

  /// @brief This context's private GeometryData scratch (geometry placements, distance results).
  pinocchio::GeometryData& getCollisionData() const { return geom_data_; }

private:
  using BroadPhaseManager = pinocchio::BroadPhaseManagerTpl<coal::DynamicAABBTreeCollisionManager>;

  const pinocchio::Model& model_;                    ///< Borrowed; immutable; owned by the Scene.
  const pinocchio::GeometryModel& collision_model_;  ///< Borrowed; immutable; owned by the Scene.
  mutable pinocchio::Data data_;                     ///< Owned scratch for forward kinematics.
  mutable pinocchio::GeometryData geom_data_;        ///< Owned scratch for geometry placements.
  mutable std::optional<BroadPhaseManager>
      manager_;  ///< Owned; bound to this context's geom_data_.

  /// @brief One coal collision object per geometry object (parallel to collision_model_'s
  /// geometryObjects), reused by computeDistances() to refresh world AABBs for broadphase culling.
  mutable std::vector<coal::CollisionObject> aabb_objects_;
};

}  // namespace roboplan
