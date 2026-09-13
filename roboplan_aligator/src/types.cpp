#include <roboplan_aligator/types.hpp>

#include <string>

#include <roboplan/core/scene.hpp>

namespace roboplan {

// --- TrajOptResult::toRoboplan ---------------------------------------------------------------

JointTrajectory TrajOptResult::toRoboplan(const Scene& scene, const std::string& group_name) const {
  JointTrajectory jt;
  // Full-model joint labels: toFullJointPositions returns a full-model configuration built on
  // the scene's current state, so the names match Scene::getJointNames(). This mirrors core's own
  // JointConfiguration convention, which pairs getJointNames() (actuated joints) with an nq-sized
  // position vector (Scene::cur_state_). CAVEAT: for a joint whose nq contribution is > 1 (e.g. a
  // continuous joint, stored as cos/sin), joint_names.size() and positions[k].size() differ, as
  // core documents (scene.hpp getModelJointCount note). Harmless for all currently supported
  // groups (fixed-base, single-DoF joints; floating bases are rejected upstream by
  // ReducedGroupModel), but revisit the label source if multi-DoF joints enter a planning group.
  jt.joint_names = scene.getJointNames();
  jt.times = trajectory.times;

  jt.positions.reserve(trajectory.positions.size());
  for (const auto& q_reduced : trajectory.positions) {
    // Reduced-group positions -> full-model layout. Throws (std::runtime_error) if group_name is
    // unknown or the reduced size does not match the group's nq.
    jt.positions.push_back(scene.toFullJointPositions(group_name, q_reduced));
  }

  jt.velocities.reserve(trajectory.velocities.size());
  for (const auto& v_reduced : trajectory.velocities) {
    // Reduced-group velocities -> full-model layout, non-group DoF zero (locked joints have no
    // meaningful velocity in a group-scoped solve; see Scene::toFullJointVelocities).
    jt.velocities.push_back(scene.toFullJointVelocities(group_name, v_reduced));
  }

  // Accelerations are intentionally left empty: not a ProxDDP output. Torques are likewise
  // dropped here — JointTrajectory has no torque field — and stay on TrajOptResult's `controls`.
  return jt;
}

}  // namespace roboplan
