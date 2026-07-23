#include "reduced_group_model.hpp"

#include <cstddef>
#include <stdexcept>

#include <pinocchio/algorithm/model.hpp>  // pinocchio::buildReducedModel

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>  // roboplan::JointGroupInfo

namespace roboplan {

ReducedGroupModel::ReducedGroupModel(const Scene& scene, const std::string& group_name)
    : group_name_(group_name), full_model_(scene.getModel()),
      full_collision_model_(scene.getCollisionModel()) {
  if (group_name.empty()) {
    throw std::invalid_argument("ReducedGroupModel: group_name must not be empty.");
  }

  const auto group_info = scene.getJointGroupInfo(group_name);
  if (!group_info) {
    throw std::invalid_argument("ReducedGroupModel: unknown joint group '" + group_name +
                                "': " + group_info.error());
  }
  const JointGroupInfo& group = group_info.value();

  if (group.joint_indices.empty()) {
    throw std::invalid_argument("ReducedGroupModel: joint group '" + group_name +
                                "' contains no movable joints.");
  }

  // Fixed-base only: aligator's MultibodyFreeFwdDynamics is free-space (no contact), so a
  // floating base inside the group is unsupported. A floating base OUTSIDE the group is fine —
  // it gets locked into a rigid fixed base below.
  for (const auto joint_index : group.joint_indices) {
    const std::string shortname = full_model_.joints[joint_index].shortname();
    if (shortname == "JointModelFreeFlyer" || shortname == "JointModelPlanar") {
      throw std::invalid_argument("ReducedGroupModel: joint group '" + group_name +
                                  "' spans a floating base (joint '" +
                                  full_model_.names[joint_index] + "' is " + shortname +
                                  "); only fixed-base groups are supported.");
    }
  }

  // Lock every movable joint that is NOT in the group. Joint 0 is the universe root and is
  // never locked. When the group already spans every movable joint, this list is empty and the
  // reduction is a no-op.
  std::vector<bool> in_group(static_cast<std::size_t>(full_model_.njoints), false);
  for (const auto joint_index : group.joint_indices) {
    in_group[joint_index] = true;
  }
  std::vector<pinocchio::JointIndex> joints_to_lock;
  for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(full_model_.njoints);
       ++j) {
    if (!in_group[j]) {
      joints_to_lock.push_back(j);
      locked_joint_names_.push_back(full_model_.names[j]);
    }
  }

  // Reduce both the kinematic model and the collision geometry, locking the non-group joints at
  // the scene's current full configuration (model.nq layout). Locked joints become FIXED_JOINT
  // frames; joint/geometry order is preserved but frames are re-ordered (hence frameId() below
  // re-resolves by name). See API_NOTES.md.
  const Eigen::VectorXd& reference_configuration = scene.getCurrentJointPositions();
  pinocchio::buildReducedModel(full_model_, full_collision_model_, joints_to_lock,
                               reference_configuration, reduced_model_, reduced_collision_model_);

  // Reduced q0 by joint-name remap: copy each reduced joint's config segment out of the full
  // reference configuration. Order-robust (does not rely on preserved joint ordering). v0
  // defaults to zero (design §3.1).
  q0_ = Eigen::VectorXd::Zero(reduced_model_.nq);
  for (pinocchio::JointIndex j = 1; j < static_cast<pinocchio::JointIndex>(reduced_model_.njoints);
       ++j) {
    const std::string& name = reduced_model_.names[j];
    const pinocchio::JointIndex full_id = full_model_.getJointId(name);
    q0_.segment(reduced_model_.idx_qs[j], reduced_model_.nqs[j]) =
        reference_configuration.segment(full_model_.idx_qs[full_id], full_model_.nqs[full_id]);
  }
  v0_ = Eigen::VectorXd::Zero(reduced_model_.nv);

  // Group velocity-index map, read verbatim from core (not re-derived here).
  v_indices_ = group.v_indices;
}

tl::expected<pinocchio::FrameIndex, std::string>
ReducedGroupModel::frameId(const std::string& frame_name) const {
  // getFrameId returns frames.size() (a valid-looking index) when the frame is absent, so guard
  // with existFrame first (API_NOTES.md).
  if (!reduced_model_.existFrame(frame_name)) {
    return tl::make_unexpected("Frame '" + frame_name +
                               "' not found in the reduced model for group '" + group_name_ + "'.");
  }
  return reduced_model_.getFrameId(frame_name);
}

std::vector<std::string> ReducedGroupModel::frameNames() const {
  std::vector<std::string> names;
  names.reserve(reduced_model_.frames.size());
  for (const auto& frame : reduced_model_.frames) {
    names.push_back(frame.name);
  }
  return names;
}

}  // namespace roboplan
