#pragma once

// INTERNAL helper — NOT public API. This header lives under src/, is not installed, and is
// not exposed in the Python bindings. It is built once inside the (future) TrajectoryOptimizer
// constructor and passed privately to the cost/constraint factories. Consumers of
// roboplan_aligator cannot include it.
//
// Why this exists (design doc §3.1): every other roboplan satellite does memoryless
// kinematics on the full Scene::getModel() and column-slices the group's DoF. Trajectory
// optimization instead integrates forward dynamics (aligator's MultibodyPhaseSpace /
// MultibodyFreeFwdDynamics integrate ALL DoF of whatever model they are handed), so non-group
// joints must be physically LOCKED into a reduced pinocchio::Model — an integrator cannot drop
// a DoF by column selection. When the group already spans the whole movable model, the
// reduction is a harmless no-op (no joints locked).
//
// Index maps and limits are NOT re-derived here: v_indices come from Scene::getJointGroupInfo,
// limit vectors from Scene::getPositionLimitVectors / getVelocityLimitVectors (read by the
// factories, exactly as OInK does). This helper's only unique contribution is the reduced
// Model + reduced GeometryModel and the frame remap into the reduced model.

#include <string>
#include <vector>

#include <Eigen/Core>
#include <pinocchio/multibody/fwd.hpp>       // pinocchio::FrameIndex
#include <pinocchio/multibody/geometry.hpp>  // pinocchio::GeometryModel
#include <pinocchio/multibody/model.hpp>     // pinocchio::Model
#include <tl/expected.hpp>

namespace roboplan {

class Scene;

/// @brief Remaps a full-model vector into reduced-model layout by joint name.
/// @details Order-robust (does not assume preserved joint ordering between the full and reduced
/// models). `use_tangent` selects the tangent layout (size reduced_model.nv, velocities / torques)
/// vs. the configuration layout (size reduced_model.nq). Shared by the ReducedGroupModel q0 remap
/// and the cost/constraint factories' limit-vector remaps.
Eigen::VectorXd remapFullToReduced(const Eigen::VectorXd& full, const pinocchio::Model& full_model,
                                   const pinocchio::Model& reduced_model, bool use_tangent);

/// @brief Reduced pinocchio model + geometry for a named joint group, with non-group joints
/// physically locked at the scene's current configuration.
/// @details Value-type snapshot whose shape mirrors roboplan::CollisionContext: it owns the
/// reduced Model/GeometryModel by value and borrows the full ones by const&. It is a snapshot
/// at construction time; if the scene model changes, discard it and build a new one.
/// Fixed-base groups only (aligator's MultibodyFreeFwdDynamics is free-space, no contact).
class ReducedGroupModel {
public:
  /// @brief Builds the reduced model for `group_name` from `scene`.
  /// @throws std::invalid_argument if `group_name` is empty, is not a known group, contains no
  /// movable joints, or spans a floating base (free-flyer / planar root — fixed-base only).
  /// Building the reduced model is a construction-time snapshot, so an invalid group is a setup
  /// invariant violation (numerics rule: exceptions for setup violations), not a recoverable
  /// per-call failure.
  ReducedGroupModel(const Scene& scene, const std::string& group_name);

  /// @brief The group this reduced model was built for.
  const std::string& groupName() const { return group_name_; }

  /// @brief The full (unreduced) model, borrowed from the originating Scene.
  const pinocchio::Model& fullModel() const { return full_model_; }
  /// @brief The reduced model (non-group joints locked), owned by value.
  const pinocchio::Model& reducedModel() const { return reduced_model_; }
  /// @brief The full collision geometry, borrowed from the originating Scene.
  const pinocchio::GeometryModel& fullCollisionModel() const { return full_collision_model_; }
  /// @brief The reduced collision geometry (matching the reduced model), owned by value.
  const pinocchio::GeometryModel& reducedCollisionModel() const { return reduced_collision_model_; }

  /// @brief Reduced-model configuration size (use for configs; never assume nq == nv).
  int nq() const { return reduced_model_.nq; }
  /// @brief Reduced-model tangent size (use for velocities / torques / Jacobian columns).
  int nv() const { return reduced_model_.nv; }

  /// @brief The scene's current group configuration on the reduced model (size nq()).
  const Eigen::VectorXd& q0() const { return q0_; }
  /// @brief Default reduced-model velocity: zero (design §3.1), size nv().
  const Eigen::VectorXd& v0() const { return v0_; }

  /// @brief Full-model velocity (Jacobian column) indices of the group's DoF, read verbatim
  /// from Scene::getJointGroupInfo (not re-derived). Size nv(). Used for collision-Jacobian
  /// column selection (§5): full-model columns -> reduced tangent/control indices.
  const Eigen::VectorXi& vIndices() const { return v_indices_; }

  /// @brief Names of the non-group joints locked into the reduced model, in ascending
  /// full-model joint-index order. Empty when the group spans the whole movable model (no-op
  /// reduction).
  const std::vector<std::string>& lockedJointNames() const { return locked_joint_names_; }

  /// @brief Resolves a frame name against the REDUCED model.
  /// @details buildReducedModel re-orders frames "in a hard to predict way" and turns locked
  /// joints into FIXED_JOINT frames, so a full-model pinocchio::FrameIndex does NOT carry over;
  /// frame names must be re-resolved here. Returns an error string if the frame is absent from
  /// the reduced model.
  tl::expected<pinocchio::FrameIndex, std::string> frameId(const std::string& frame_name) const;

  /// @brief All frame names present in the reduced model (for introspection / diagnostics).
  std::vector<std::string> frameNames() const;

private:
  std::string group_name_;
  const pinocchio::Model& full_model_;                    ///< Borrowed; owned by the Scene.
  const pinocchio::GeometryModel& full_collision_model_;  ///< Borrowed; owned by the Scene.
  pinocchio::Model reduced_model_;                        ///< Owned.
  pinocchio::GeometryModel reduced_collision_model_;      ///< Owned.
  Eigen::VectorXd q0_;         ///< Current group config on reduced model.
  Eigen::VectorXd v0_;         ///< Zero, size nv().
  Eigen::VectorXi v_indices_;  ///< From JointGroupInfo; size nv().
  std::vector<std::string> locked_joint_names_;
};

}  // namespace roboplan
