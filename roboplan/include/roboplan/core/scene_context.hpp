#pragma once

#include <optional>
#include <random>
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

/// @brief Per-consumer scratch space for scene queries: Pinocchio data, geometry data, the
/// broadphase tree, a random number generator, and a current configuration.
///
/// @details A Scene's query methods mutate shared scratch (joint and frame placements, geometry
/// world transforms, the broadphase AABB tree) and shared bookkeeping (its RNG and current joint
/// positions), so a single Scene cannot answer those queries from multiple threads concurrently.
/// A SceneContext owns its own copy of all of that over the Scene's immutable model and geometry
/// model (shared by reference), so an algorithm can run without contending with anything else.
///
/// Every method is the exact query of the same name on Scene, run against this context's private
/// scratch instead of the Scene's. Give each thread its own context and none of them interact.
///
/// A context borrows the scene's model and collision geometry and sizes its own scratch from them
/// at construction. Adding/removing geometry or changing collision pairs leaves that scratch stale
/// (see Scene::getGeometryVersion): the collision queries report the mismatch, while kinematics
/// and sampling are unaffected. Moving existing geometry invalidates nothing.
class SceneContext {
public:
  /// @brief Snapshots the current collision geometry of `scene`.
  /// @details The context's configuration starts at the scene's current joint positions and its
  /// RNG is seeded pseudorandomly; use setJointPositions() and setRngSeed() to pin either.
  explicit SceneContext(const Scene& scene);

  // Non-copyable and non-movable: the broadphase manager caches a raw pointer to `geom_data_`,
  // so the object's address must remain stable for its whole lifetime.
  /// Hold one behind a pointer e.g., std::unique_ptr) if it needs to be relocated or rebuilt.
  SceneContext(const SceneContext&) = delete;
  SceneContext& operator=(const SceneContext&) = delete;
  SceneContext(SceneContext&&) = delete;
  SceneContext& operator=(SceneContext&&) = delete;

  /// @brief Checks collisions at `q` using this context's own scratch.
  /// @param q The joint positions.
  /// @param debug If true, prints every colliding pair instead of stopping at the first collision.
  /// @return True if there are collisions, else false.
  bool hasCollisions(const Eigen::VectorXd& q, const bool debug = false) const;

  /// @brief Refreshes geometry placements at `q` and computes the distance for every active
  /// collision pair into this context's own GeometryData.
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

  /// @brief Computes the joint Jacobians at `q` into this context's own Data.
  /// @details Results are available via getData().
  void computeJointJacobians(const Eigen::VectorXd& q) const;

  /// @brief Runs forward kinematics at `q` and refreshes every frame placement in getData().oMf.
  void updateFramePlacements(const Eigen::VectorXd& q) const;

  /// @brief Calculates forward kinematics for a specific frame, into this context's Data.
  /// @param q The joint configuration.
  /// @param frame_name The name of the frame for which to perform forward kinematics.
  /// @param base_frame Optional base frame. If empty, returns the world-frame pose.
  /// @return The 4x4 matrix denoting the transform of the specified frame.
  Eigen::Matrix4d forwardKinematics(const Eigen::VectorXd& q, const std::string& frame_name,
                                    const std::string& base_frame = "") const;

  /// @brief Computes the frame Jacobian for a specific frame, into this context's own Data.
  void computeFrameJacobian(const Eigen::VectorXd& q, pinocchio::FrameIndex frame_id,
                            pinocchio::ReferenceFrame reference_frame,
                            Eigen::Ref<Eigen::MatrixXd> jacobian) const;

  /// @brief Computes the Jacobian of a frame relative to a (possibly moving) base frame, into this
  /// context's own Data.
  void computeRelativeFrameJacobian(const Eigen::VectorXd& q, pinocchio::FrameIndex frame_id,
                                    const std::string& base_frame,
                                    pinocchio::ReferenceFrame reference_frame,
                                    Eigen::Ref<Eigen::MatrixXd> jacobian) const;

  /// @brief Sets the seed of this context's random number generator.
  /// @details Only affects this context, so two seeded algorithms sharing a Scene no longer
  /// reseed one another.
  void setRngSeed(unsigned int seed);

  /// @brief Generates random positions for the robot model, using this context's RNG.
  Eigen::VectorXd randomPositions();

  /// @brief Randomizes the positions of the specified joints in-place, using this context's RNG.
  /// @param joint_names The names of the joints to randomize.
  /// @param q The full configuration vector to modify in-place. Must be sized to the model's nq.
  void randomizeJointPositions(const std::vector<std::string>& joint_names, Eigen::VectorXd& q);

  /// @brief Generates random collision-free positions, using this context's RNG and scratch.
  /// @param max_samples The maximum number of samples to attempt.
  /// @return The random positions, if successful, else std::nullopt.
  std::optional<Eigen::VectorXd> randomCollisionFreePositions(size_t max_samples = 1000);

  /// @brief This context's current joint positions (size model.nq).
  const Eigen::VectorXd& getJointPositions() const { return q_; }

  /// @brief Sets this context's current joint positions.
  void setJointPositions(const Eigen::VectorXd& q);

  /// @brief Converts partial joint positions to full ones, filling non-group joints from this
  /// context's current configuration.
  Eigen::VectorXd toFullJointPositions(const std::string& group_name,
                                       const Eigen::VectorXd& q) const;

  /// @brief The Scene this context was snapshotted from.
  /// @details Safe to use for the Scene's immutable queries (limits, group and frame lookups,
  /// distance, interpolation). Do not call the Scene's scratch-writing queries through it.
  const Scene& getScene() const { return scene_; }

  /// @brief The immutable model shared with the originating Scene.
  const pinocchio::Model& getModel() const { return model_; }

  /// @brief This context's private Data scratch (forward kinematics, joint Jacobians).
  pinocchio::Data& getData() const { return data_; }

  /// @brief The immutable collision geometry model shared with the originating Scene.
  const pinocchio::GeometryModel& getCollisionModel() const { return collision_model_; }

  /// @brief This context's private GeometryData scratch (geometry placements, distance results).
  pinocchio::GeometryData& getCollisionData() const { return geom_data_; }

  /// @brief Whether the originating Scene's collision geometry is still the one snapshotted here.
  bool isGeometryCurrent() const;

private:
  using BroadPhaseManager = pinocchio::BroadPhaseManagerTpl<coal::DynamicAABBTreeCollisionManager>;

  /// @brief Throws if the Scene's geometry changed since this context was built.
  /// @details Called at the top of every collision query. Without it, a context built before an
  /// addGeometry() would index its old, smaller distanceResults with the new pair count.
  void checkGeometryCurrent(const char* what) const;

  const Scene& scene_;                               ///< Borrowed; owned by the caller.
  const pinocchio::Model& model_;                    ///< Borrowed; immutable; owned by the Scene.
  const pinocchio::GeometryModel& collision_model_;  ///< Borrowed; immutable; owned by the Scene.
  mutable pinocchio::Data data_;                     ///< Owned scratch for forward kinematics.
  mutable pinocchio::GeometryData geom_data_;        ///< Owned scratch for geometry placements.
  mutable std::optional<BroadPhaseManager>
      manager_;  ///< Owned; bound to this context's geom_data_.

  /// @brief One coal collision object per geometry object, parallel to collision_model_'s
  /// geometryObjects, reused by computeDistances() to refresh world AABBs for broadphase culling.
  mutable std::vector<coal::CollisionObject> aabb_objects_;

  /// @brief This context's own random number generator.
  std::mt19937 rng_gen_;

  /// @brief This context's own current configuration (size model.nq).
  Eigen::VectorXd q_;

  /// @brief The Scene geometry version this context snapshotted.
  uint64_t geometry_version_;
};

}  // namespace roboplan
