#pragma once

#include <filesystem>
#include <iostream>
#include <map>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/geometry.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/collision/broadphase-manager.hpp>
#include <pinocchio/fwd.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>
#include <tl/expected.hpp>

#include <roboplan/core/geometry_wrappers.hpp>
#include <roboplan/core/types.hpp>

// The concrete broadphase manager (AABB tree). Mirror the coal/hpp-fcl include guard used in
// geometry_wrappers.hpp so the `coal` namespace resolves in both modern and legacy builds.
#if defined(__has_include) && __has_include(<coal/fwd.hh>)
#include <coal/broadphase/broadphase_dynamic_AABB_tree.h>
#else
#include <hpp/fcl/broadphase/broadphase_dynamic_AABB_tree.h>
namespace coal = hpp::fcl;
#endif

namespace roboplan {

/// @brief URDF robot description and SRDF planning configuration documents.
struct UrdfSceneDescription {
  std::string urdf_xml;
  std::string srdf_xml;
};

/// @brief Loads a URDF robot description and SRDF planning configuration from disk.
UrdfSceneDescription loadUrdfSceneDescription(const std::filesystem::path& urdf_path,
                                              const std::filesystem::path& srdf_path);

/// @brief Pinocchio model and collision geometry used to construct a Scene.
struct PinocchioSceneDescription {
  pinocchio::Model model;
  pinocchio::GeometryModel collision_model;
};

/// @brief Loads an MJCF model and its collision geometry from disk.
PinocchioSceneDescription loadMjcfModel(const std::filesystem::path& mjcf_path);

/// @brief Extended joint limits parsed from a URDF <limit> tag.
struct UrdfExtendedJointLimits {
  std::optional<double> acceleration;
  std::optional<double> jerk;
};

/// @brief Primary scene representation for planning and control.
///
/// @par Thread safety
/// A Scene splits into three kinds of state, and only the first is safe to share:
///
/// 1. The robot description (`getModel()`, the collision geometry, the frame / joint / group
///    lookups) is immutable once the constructor returns. Every query that reads only these --
///    `configurationDistance`, `interpolate`, `integrate`, `difference`, `isValidConfiguration`,
///    `clampToValidConfiguration`, `toFullJointPositions`, `getFrameId`, `getJointInfo`,
///    `getJointGroupInfo`, and the limit getters -- is safe to call concurrently on one Scene.
///
/// 2. Per-query scratch (`model_data_`, `collision_model_data_`, `broadphase_manager_`). The
///    methods that write it are marked `const` for convenience, but they are *not* safe to call
///    concurrently. Each thread should own a SceneContext, which holds private copies of that
///    scratch over this Scene's immutable description, and call the equivalent methods there.
///
/// 3. Mutable bookkeeping (`setJointPositions`, `setRngSeed`, and the geometry mutators). These
///    are for single-threaded setup and interactive use. Library code must not write them while
///    other threads are querying, and must not read `getCurrentJointPositions()` in the middle of
///    an algorithm.
///
/// For items 2. and 3., if you need thread safety, we recommend using a SceneContext.
class Scene {
public:
  /// @brief Builds a scene from typed URDF and SRDF documents.
  Scene(const std::string& name, const UrdfSceneDescription& description,
        const std::vector<std::filesystem::path>& package_paths =
            std::vector<std::filesystem::path>(),
        const std::filesystem::path& yaml_config_path = std::filesystem::path());

  /// @brief Builds a scene from a prebuilt Pinocchio model and collision geometry.
  Scene(const std::string& name, const PinocchioSceneDescription& description,
        const std::filesystem::path& yaml_config_path = std::filesystem::path());

  // Non-copyable and non-movable. `broadphase_manager_` caches raw pointers to this Scene's
  // `model_` and `collision_model_data_`, so a defaulted copy or move would silently produce a
  // Scene whose manager collides against the *original* Scene's data. Share a Scene with
  // `std::shared_ptr<Scene>` and give each thread its own SceneContext rather than cloning it.
  Scene(const Scene&) = delete;
  Scene& operator=(const Scene&) = delete;
  Scene(Scene&&) = delete;
  Scene& operator=(Scene&&) = delete;

  /// @brief Gets the scene's name.
  /// @return The scene name.
  const std::string& getName() const { return name_; };

  /// @brief Gets the scene's internal Pinocchio model.
  /// @return The Pinocchio model.
  const pinocchio::Model& getModel() const { return model_; };

  /// @brief Gets the scene's internal Pinocchio data (read-only).
  /// @return The Pinocchio data.
  const pinocchio::Data& getData() const { return model_data_; };

  /// @brief Gets the scene's internal Pinocchio collision model.
  /// @return The Pinocchio collision (geometry) model.
  const pinocchio::GeometryModel& getCollisionModel() const { return collision_model_; };

  /// @brief Gets the scene's internal Pinocchio collision (geometry) data.
  /// @details The data is shared with the scene; computations such as
  /// hasCollisions() and computeCollisionDistances() write into this data.
  /// @return The Pinocchio collision (geometry) data.
  const pinocchio::GeometryData& getCollisionData() const { return collision_model_data_; };

  /// @brief Updates geometry placements and computes distance results for all
  /// active collision pairs at the specified joint configuration.
  /// @details After this call, distances are available via
  /// getCollisionData().distanceResults.
  /// @note Writes the Scene's shared scratch; not safe to call concurrently. Use
  /// SceneContext::computeDistances instead.
  /// @param q The joint configuration at which to compute the distances.
  void computeCollisionDistances(const Eigen::VectorXd& q) const;

  /// @brief A counter that changes whenever the collision geometry is modified.
  /// @return The current geometry version.
  uint64_t getGeometryVersion() const { return geometry_version_; };

  /// @brief Gets the scene's actuated joint names (non-mimic joints only).
  /// @return A vector of joint names.
  const std::vector<std::string>& getJointNames() const { return actuated_joint_names_; };

  /// @brief Gets the scene's full joint names, including mimic joints.
  /// @return A vector of joint names.
  const std::vector<std::string>& getJointNamesWithMimics() const { return joint_names_; };

  /// @brief Gets the information for a specific joint.
  /// @param joint_name The name of the joint.
  /// @return The joint information struct if successful, else a string describing the error.
  tl::expected<JointInfo, std::string> getJointInfo(const std::string& joint_name) const;

  /// @brief Gets the distance between two joint configurations.
  /// @param q_start The starting joint positions.
  /// @param q_end The ending joint positions.
  /// @return The configuration-space distance between the two positions.
  double configurationDistance(const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_end) const;

  /// @brief Sets the seed for the random number generator (RNG).
  /// @param seed The seed to set.
  void setRngSeed(unsigned int seed);

  /// @brief Generates random positions for the robot model.
  /// @return The random positions.
  Eigen::VectorXd randomPositions();

  /// @brief Generates random positions using a caller-supplied random number generator.
  /// @details This draws from `rng` rather than the Scene's own generator, so it reads nothing
  /// mutable and any number of threads may call it at once as long as each passes its own.
  /// Prefer SceneContext, which owns an `rng` and calls this for you.
  /// @param rng The random number generator to draw from.
  /// @return The random positions.
  Eigen::VectorXd randomPositions(std::mt19937& rng) const;

  /// @brief Randomizes the positions of the specified joints in-place within a full configuration.
  /// @details Only the degrees of freedom belonging to `joint_names` are overwritten; all other
  /// entries of `q` are left untouched. This avoids allocating a full configuration and sampling
  /// joints outside of a planning group on every call (e.g. in the RRT sampling loop).
  /// @param joint_names The names of the joints to randomize.
  /// @param q The full configuration vector to modify in-place. Must be sized to the model's nq.
  void randomizeJointPositions(const std::vector<std::string>& joint_names, Eigen::VectorXd& q);

  /// @brief Randomizes the given joints in-place using a caller-supplied random number generator.
  /// @details The explicit-scratch overload of the above.
  /// @param rng The random number generator to draw from.
  /// @param joint_names The names of the joints to randomize.
  /// @param q The full configuration vector to modify in-place. Must be sized to the model's nq.
  void randomizeJointPositions(std::mt19937& rng, const std::vector<std::string>& joint_names,
                               Eigen::VectorXd& q) const;

  /// @brief Generates random collision-free positions for the robot model.
  /// @param max_tries The maximum number of samples to attempt.
  /// @return The random positions, if successful, else std::nullopt.
  std::optional<Eigen::VectorXd> randomCollisionFreePositions(size_t max_samples = 1000);

  /// @brief Checks collisions at specified joint positions.
  /// @param q The joint positions.
  /// @param debug If true, prints debug information and does not stop at first collision.
  /// This parameter is disabled by default.
  /// @return True if there are collisions, else false.
  bool hasCollisions(const Eigen::VectorXd& q, const bool debug = false) const;

  /// @brief Checks if the specified joint positions are valid with respect to joint limits.
  /// @param q The joint positions.
  /// @return True if the positions respect joint limits, else false.
  bool isValidConfiguration(const Eigen::VectorXd& q) const;

  /// @brief Clamps the specified joint positions to valid joint limits.
  /// @details Bounded joints are clamped to their position limits, while continuous and planar
  /// rotation representations are renormalized onto the unit circle.
  /// @param q The joint positions.
  /// @return A new vector of joint positions that respects joint limits.
  Eigen::VectorXd clampToValidConfiguration(const Eigen::VectorXd& q) const;

  /// @brief Converts partial joint positions to full joint positions.
  /// @details This includes adding new joints. The joints outside `group_name` are filled from the
  /// scene's current state; use the three-argument overload to supply them explicitly instead.
  /// @param group_name The name of the joint group.
  /// @param q The original (partial) joint positions.
  /// @return The full joint positions.
  Eigen::VectorXd toFullJointPositions(const std::string& group_name,
                                       const Eigen::VectorXd& q) const;

  /// @brief Converts partial joint positions to full joint positions against an explicit reference.
  /// @details Reads no Scene state beyond the immutable model, so it is safe to call concurrently.
  /// @param group_name The name of the joint group.
  /// @param q The original (partial) joint positions.
  /// @param q_reference The full configuration supplying the joints outside the group.
  /// @return The full joint positions.
  Eigen::VectorXd toFullJointPositions(const std::string& group_name, const Eigen::VectorXd& q,
                                       const Eigen::VectorXd& q_reference) const;

  /// @brief Interpolates between two joint configurations.
  /// @param q_start The starting joint configuration.
  /// @param q_end The ending joint configuration.
  /// @param fraction The interpolation coefficient, between 0 and 1.
  Eigen::VectorXd interpolate(const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_end,
                              const double fraction) const;

  /// @brief Integrates a velocity vector from a configuration using Lie group operations.
  /// @param q The starting joint configuration (size model.nq).
  /// @param v The velocity / displacement vector to integrate (size model.nv).
  /// @return The resulting joint configuration after integration.
  Eigen::VectorXd integrate(const Eigen::VectorXd& q, const Eigen::VectorXd& v) const;

  /// @brief Computes the difference between two configurations, using Lie group operations.
  /// @param q_start The starting joint configuration (size model.nq).
  /// @param q_end The ending joint configuration (size model.nq).
  /// @return The displacement from `q_start` to `q_end` (size model.nv).
  Eigen::VectorXd difference(const Eigen::VectorXd& q_start, const Eigen::VectorXd& q_end) const;

  /// @brief Calculates forward kinematics for a specific frame.
  /// @param q The joint configuration.
  /// @param frame_name The name of the frame for which to perform forward kinematics.
  /// @param base_frame Optional base frame. If empty, returns the world-frame pose.
  /// @return The 4x4 matrix denoting the transform of the specified frame.
  Eigen::Matrix4d forwardKinematics(const Eigen::VectorXd& q, const std::string& frame_name,
                                    const std::string& base_frame = "") const;

  /// @brief Calculates forward kinematics into a caller-supplied Pinocchio data.
  /// @details This writes `data` rather than the Scene's shared scratch, so it reads nothing
  /// mutable and any number of threads may call it at once as long as each passes its own.
  /// Prefer SceneContext, which owns a `data` and calls this for you.
  /// @param data The Pinocchio data to write.
  /// @param q The joint configuration.
  /// @param frame_name The name of the frame for which to perform forward kinematics.
  /// @param base_frame Optional base frame. If empty, returns the world-frame pose.
  /// @return The 4x4 matrix denoting the transform of the specified frame.
  Eigen::Matrix4d forwardKinematics(pinocchio::Data& data, const Eigen::VectorXd& q,
                                    const std::string& frame_name,
                                    const std::string& base_frame = "") const;

  /// @brief Computes the frame Jacobian for a specific frame expressed in world frame.
  /// @note Requires that forward kinematics and frame placements are up-to-date, or that
  /// this is the first kinematics call for the given q (the underlying Pinocchio call runs FK).
  /// @param q The joint configuration.
  /// @param frame_id The Pinocchio frame ID of the frame.
  /// @param reference_frame The reference frame for the Jacobian output (LOCAL, WORLD, or
  /// LOCAL_WORLD_ALIGNED).
  /// @param jacobian Output matrix to store the Jacobian (must be pre-allocated to 6 x nv).
  void computeFrameJacobian(const Eigen::VectorXd& q, pinocchio::FrameIndex frame_id,
                            pinocchio::ReferenceFrame reference_frame,
                            Eigen::Ref<Eigen::MatrixXd> jacobian) const;

  /// @brief Computes the frame Jacobian into a caller-supplied Pinocchio data.
  /// @details The explicit-scratch overload of the above.
  /// @param data The Pinocchio data to write.
  /// @param q The joint configuration.
  /// @param frame_id The Pinocchio frame ID of the frame.
  /// @param reference_frame The reference frame for the Jacobian output (LOCAL, WORLD, or
  /// LOCAL_WORLD_ALIGNED).
  /// @param jacobian Output matrix to store the Jacobian (must be pre-allocated to 6 x nv).
  void computeFrameJacobian(pinocchio::Data& data, const Eigen::VectorXd& q,
                            pinocchio::FrameIndex frame_id,
                            pinocchio::ReferenceFrame reference_frame,
                            Eigen::Ref<Eigen::MatrixXd> jacobian) const;

  /// @brief Computes the Jacobian of a frame's velocity relative to a (possibly moving) base frame.
  /// @details Computes the Jacobian of the EE frame velocity relative to the base frame, expressed
  /// in the reference frame of the relative transform T_rel = T_base^{-1} * T_ee.
  /// @note Requires that forward kinematics and frame placements are up-to-date, or that
  /// this is the first kinematics call for the given q (the underlying Pinocchio call runs FK).
  /// @param q The joint configuration.
  /// @param frame_id The Pinocchio frame ID of the end-effector frame.
  /// @param base_frame The name of the base frame (its ID is looked up internally).
  /// @param reference_frame The reference frame for the Jacobian output. LOCAL is expressed in the
  /// body frame of T_rel; LOCAL_WORLD_ALIGNED is at the T_rel origin with world orientation.
  /// @param jacobian Output matrix to store the Jacobian (must be pre-allocated to 6 x nv).
  void computeRelativeFrameJacobian(const Eigen::VectorXd& q, pinocchio::FrameIndex frame_id,
                                    const std::string& base_frame,
                                    pinocchio::ReferenceFrame reference_frame,
                                    Eigen::Ref<Eigen::MatrixXd> jacobian) const;

  /// @brief Computes the relative frame Jacobian into a caller-supplied Pinocchio data.
  /// @details The explicit-scratch overload of the above.
  /// @param data The Pinocchio data to write.
  /// @param q The joint configuration.
  /// @param frame_id The Pinocchio frame ID of the end-effector frame.
  /// @param base_frame The name of the base frame (its ID is looked up internally).
  /// @param reference_frame The reference frame for the Jacobian output. LOCAL is expressed in the
  /// body frame of T_rel; LOCAL_WORLD_ALIGNED is at the T_rel origin with world orientation.
  /// @param jacobian Output matrix to store the Jacobian (must be pre-allocated to 6 x nv).
  void computeRelativeFrameJacobian(pinocchio::Data& data, const Eigen::VectorXd& q,
                                    pinocchio::FrameIndex frame_id, const std::string& base_frame,
                                    pinocchio::ReferenceFrame reference_frame,
                                    Eigen::Ref<Eigen::MatrixXd> jacobian) const;

  /// @brief Computes the joint Jacobians for every joint at the given configuration.
  /// @details Populates the internal Pinocchio data so that pinocchio::getJointJacobian
  /// can be called for any joint after this. Also runs forward kinematics.
  /// @param q The joint configuration.
  void computeJointJacobians(const Eigen::VectorXd& q) const;

  /// @brief Computes the joint Jacobians into a caller-supplied Pinocchio data.
  /// @details The explicit-scratch overload of the above.
  /// @param data The Pinocchio data to write.
  /// @param q The joint configuration.
  void computeJointJacobians(pinocchio::Data& data, const Eigen::VectorXd& q) const;

  /// @brief Get the Pinocchio model ID of a frame by its name.
  /// @param name The name of the frame to look up.
  /// @return The Pinocchio frame ID if successful, else a string describing the error.
  tl::expected<pinocchio::FrameIndex, std::string> getFrameId(const std::string& name) const;

  /// @brief Get the joint group information of a scene by its name.
  /// @param name The name of the joint group to look up.
  /// @return The joint group information if successful, else a string describing the error.
  tl::expected<JointGroupInfo, std::string> getJointGroupInfo(const std::string& name) const;

  /// @brief Get the current Pinocchio configuration vector (model.nq).
  /// @details This is the internal planning layout (e.g. continuous joints as [cos, sin]).
  /// Joint count may differ from getJointNames().size().
  /// @return The current joint position vector.
  const Eigen::VectorXd& getCurrentJointPositions() const { return cur_state_.positions; }

  /// @brief Get current joint positions for all joints in getJointNamesWithMimics() order.
  /// @details Actuated joints copy values from the Pinocchio configuration; mimic joints use
  /// scaling * mimicked_position + offset per degree of freedom.
  /// @return The joint position vector aligned with getJointNamesWithMimics().
  Eigen::VectorXd getCurrentJointPositionsWithMimics() const;

  /// @brief Set the joint positions for the full robot state.
  /// @param positions The desired joint position vector (size model.nq).
  void setJointPositions(const Eigen::VectorXd& positions);

  /// @brief Get the joint position indices for a set of joint names.
  /// @param joint_names The joint names for which to look up position indices.
  /// @return The corresponding joint position indices.
  Eigen::VectorXi getJointPositionIndices(const std::vector<std::string>& joint_names) const;

  /// @brief Get the joint position limit vectors for a specified group.
  /// @param group_name The name of the group. Defaults to the complete robot model.
  /// @param collapsed If true, collapses limits for continuous rotation degrees of freedom into
  /// one value; else, leaves them expanded as two values for cos(theta) and sin(theta).
  /// @return A pair of vectors for the lower and upper joint position limits, if successful,
  /// or a string describing any errors.
  tl::expected<EigenVectorPair, std::string>
  getPositionLimitVectors(const std::string& group_name = "", const bool collapsed = false) const;

  /// @brief Get the joint velocity limit vectors for a specified group.
  /// @param group_name The name of the group. Defaults to the complete robot model.
  /// @return A pair of vectors for the lower and upper joint velocity limits, if successful,
  /// or a string describing any errors.
  tl::expected<EigenVectorPair, std::string>
  getVelocityLimitVectors(const std::string& group_name = "") const;

  /// @brief Get the joint acceleration limit vectors for a specified group.
  /// @param group_name The name of the group. Defaults to the complete robot model.
  /// @return A pair of vectors for the lower and upper joint acceleration limits, if successful,
  /// or a string describing any errors.
  tl::expected<EigenVectorPair, std::string>
  getAccelerationLimitVectors(const std::string& group_name = "") const;

  /// @brief Get the joint jerk limit vectors for a specified group.
  /// @param group_name The name of the group. Defaults to the complete robot model.
  /// @return A pair of vectors for the lower and upper joint jerk limits, if successful,
  /// or a string describing any errors.
  tl::expected<EigenVectorPair, std::string>
  getJerkLimitVectors(const std::string& group_name = "") const;

  /// @brief Adds a box geometry to the scene.
  /// @param name The name of the object to add.
  /// @param parent_frame The name of the parent frame to add the object to.
  /// @param box The box geometry instance to add.
  /// @param tform The transform between the parent frame and the geometry.
  /// @param color The color of the geometry, in RGBA vector format.
  /// @return Void if successful, else a string describing the error.
  tl::expected<void, std::string> addBoxGeometry(const std::string& name,
                                                 const std::string& parent_frame, const Box& box,
                                                 const Eigen::Matrix4d& tform,
                                                 const Eigen::Vector4d& color);

  /// @brief Adds a sphere geometry to the scene.
  /// @param name The name of the object to add.
  /// @param parent_frame The name of the parent frame to add the object to.
  /// @param sphere The sphere geometry instance to add.
  /// @param tform The transform between the parent frame and the geometry.
  /// @param color The color of the geometry, in RGBA vector format.
  /// @return Void if successful, else a string describing the error.
  tl::expected<void, std::string>
  addSphereGeometry(const std::string& name, const std::string& parent_frame, const Sphere& sphere,
                    const Eigen::Matrix4d& tform, const Eigen::Vector4d& color);

  /// @brief Adds a cylinder geometry to the scene.
  /// @param name The name of the object to add.
  /// @param parent_frame The name of the parent frame to add the object to.
  /// @param cylinder The cylinder geometry instance to add.
  /// @param tform The transform between the parent frame and the geometry.
  /// @param color The color of the geometry, in RGBA vector format.
  /// @return Void if successful, else a string describing the error.
  tl::expected<void, std::string> addCylinderGeometry(const std::string& name,
                                                      const std::string& parent_frame,
                                                      const Cylinder& cylinder,
                                                      const Eigen::Matrix4d& tform,
                                                      const Eigen::Vector4d& color);

  /// @brief Adds a triangle mesh geometry to the scene.
  /// @param name The name of the object to add.
  /// @param parent_frame The name of the parent frame to add the object to.
  /// @param mesh The mesh geometry instance to add.
  /// @param tform The transform between the parent frame and the geometry.
  /// @param color The color of the geometry, in RGBA vector format.
  /// @return Void if successful, else a string describing the error.
  tl::expected<void, std::string> addMeshGeometry(const std::string& name,
                                                  const std::string& parent_frame, const Mesh& mesh,
                                                  const Eigen::Matrix4d& tform,
                                                  const Eigen::Vector4d& color);

  /// @brief Adds a octree geometry to the scene.
  /// @param name The name of the object to add.
  /// @param parent_frame The name of the parent frame to add the object to.
  /// @param octree The octree geometry instance to add.
  /// @param tform The transform between the parent frame and the geometry.
  /// @param color The color of the geometry, in RGBA vector format.
  /// @return Void if successful, else a string describing the error.
  tl::expected<void, std::string>
  addOcTreeGeometry(const std::string& name, const std::string& parent_frame, const OcTree& octree,
                    const Eigen::Matrix4d& tform, const Eigen::Vector4d& color);

  /// @brief Adds a Pinocchio geometry object to the scene.
  /// @details This can be made the sole public entrypoint to add a geometry once
  /// Pinocchio and Coal have working nanobind bindings compatible with this library.
  /// @param geom_obj The geometry object instance to add.
  /// @return Void if successful, else a string describing the error.
  tl::expected<void, std::string> addGeometry(const pinocchio::GeometryObject& geom_obj);

  /// @brief Updates the placement of an object geometry in the scene.
  /// @param name The name of the object to update.
  /// @param parent_frame The parent frame of the transformation.
  /// @param tform The transform between the parent frame and the geometry.
  tl::expected<void, std::string> updateGeometryPlacement(const std::string& name,
                                                          const std::string& parent_frame,
                                                          Eigen::Matrix4d& tform);

  /// @brief Removes a geometry from the scene.
  /// @param name The name of the object to remove.
  tl::expected<void, std::string> removeGeometry(const std::string& name);

  /// @brief Gets a list of collision geometry IDs corresponding to a specified body.
  /// @details The body name can either be a model frame name or a collision model geometry name.
  /// @param body The name of the body.
  /// @return A std::vector of collision geometry indices for the body if successful,
  /// else a string describing the error.
  tl::expected<std::vector<pinocchio::GeomIndex>, std::string>
  getCollisionGeometryIds(const std::string& body);

  /// @brief Sets the allowable collisions for a pair of bodies in the model.
  /// @details The body names can either be model frame names or collision model geometry names.
  /// @param body1 The name of the first body.
  /// @param body2 The name of the second body.
  /// @param enable If true, enables the collision; if false, disables it.
  /// @return Void if successful, else a string describing the error.
  tl::expected<void, std::string> setCollisions(const std::string& body1, const std::string& body2,
                                                const bool enable);

  /// @brief Prints basic information about the scene.
  friend std::ostream& operator<<(std::ostream& os, const Scene& scene);

private:
  void
  initialize(const std::filesystem::path& yaml_config_path,
             const std::unordered_map<std::string, UrdfExtendedJointLimits>& urdf_extended_limits,
             const std::unordered_map<std::string, JointGroupInfo>& joint_group_info_map);

  /// @brief The name of the scene.
  std::string name_;

  /// @brief The Pinocchio model representing the robot and its environment.
  pinocchio::Model model_;

  /// @brief The default data structure for the underlying Pinocchio model.
  /// @details This won't be thread-safe unless each thread uses its own data.
  mutable pinocchio::Data model_data_;

  /// @brief The Pinocchio collision model representing the robot and its environment.
  pinocchio::GeometryModel collision_model_;

  /// @brief The default data structure for the underlying Pinocchio collision model.
  /// @details This won't be thread-safe unless each thread uses its own data.
  mutable pinocchio::GeometryData collision_model_data_;

  /// @brief Broadphase collision manager type, using a dynamic AABB tree to cull non-overlapping
  /// geometry pairs before narrow-phase collision checking.
  using BroadPhaseManager = pinocchio::BroadPhaseManagerTpl<coal::DynamicAABBTreeCollisionManager>;

  /// @brief Broadphase manager used to accelerate hasCollisions().
  /// @details Caches AABB-tree state and holds pointers into collision_model_ and
  /// collision_model_data_, so it must be rebuilt (see rebuildBroadphaseManager) whenever the
  /// collision geometry or its data is changed. Mutable for the same reason as
  /// collision_model_data_ (updated in place during a const collision query), and not thread-safe
  /// across shared Scenes.
  mutable std::optional<BroadPhaseManager> broadphase_manager_;

  /// @brief (Re)builds broadphase_manager_ from the current collision model and data.
  /// @details Must be called after collision_model_data_ is (re)assigned, since the manager caches
  /// pointers and geometry state derived from it.
  void rebuildBroadphaseManager();

  /// @brief The full list of joint names in the model (including mimic joints).
  std::vector<std::string> joint_names_;

  /// @brief Actuated (non-mimic) joint names in model order.
  std::vector<std::string> actuated_joint_names_;

  /// @brief Map from joint names to their corresponding information.
  std::unordered_map<std::string, JointInfo> joint_info_map_;

  /// @brief Map from joint group names to their corresponding information.
  std::unordered_map<std::string, JointGroupInfo> joint_group_info_map_;

  /// @brief A random number generator for the scene.
  std::mt19937 rng_gen_;

  /// @brief The current state of the model (used to fill in partial states).
  JointConfiguration cur_state_;

  /// @brief Maps each frame name to its respective Pinocchio frame ID.
  std::unordered_map<std::string, pinocchio::FrameIndex> frame_map_;

  /// @brief Maps each added collision geometry to its respective Pinocchio geometry ID.
  std::unordered_map<std::string, pinocchio::GeomIndex> collision_geometry_map_;

  /// @brief Incremented on every change to the collision geometry, so that SceneContexts
  /// snapshotted from an older version can detect that they are stale.
  uint64_t geometry_version_ = 0;
};

}  // namespace roboplan
