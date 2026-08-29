#include <roboplan_aligator/types.hpp>

#include <stdexcept>
#include <string>

#include <roboplan/core/scene.hpp>

namespace roboplan {

// --- TrajOptResult::toRoboplan ---------------------------------------------------------------

JointTrajectory TrajOptResult::toRoboplan(const Scene& scene, const std::string& group_name) const {
  JointTrajectory jt;
  // Full-model joint labels: toFullJointPositions returns a full-model configuration built on
  // the scene's current state, so the names match Scene::getJointNames() (design §4.5). This
  // mirrors core's own JointConfiguration convention, which pairs getJointNames() (actuated
  // joints) with an nq-sized position vector (Scene::cur_state_). CAVEAT: for a joint whose
  // nq contribution is > 1 (e.g. a continuous joint, stored as cos/sin), joint_names.size()
  // and positions[k].size() differ, as core documents (scene.hpp getModelJointCount note).
  // Harmless for all currently supported groups (fixed-base, single-DoF joints; floating bases
  // are rejected upstream by ReducedGroupModel), but revisit the label source if multi-DoF
  // joints enter a planning group.
  jt.joint_names = scene.getJointNames();
  jt.times = trajectory.times;

  jt.positions.reserve(trajectory.positions.size());
  for (const auto& q_reduced : trajectory.positions) {
    // Reduced-group positions -> full-model layout (design §4.5). Throws (std::runtime_error)
    // if group_name is unknown or the reduced size does not match the group's nq.
    jt.positions.push_back(scene.toFullJointPositions(group_name, q_reduced));
  }

  jt.velocities.reserve(trajectory.velocities.size());
  for (const auto& v_reduced : trajectory.velocities) {
    // Reduced-group velocities -> full-model layout, non-group DoF zero (locked joints have no
    // meaningful velocity in a group-scoped solve; see Scene::toFullJointVelocities).
    jt.velocities.push_back(scene.toFullJointVelocities(group_name, v_reduced));
  }

  // Accelerations are intentionally left empty: not a ProxDDP output (design §4.5). Torques are
  // likewise dropped here — JointTrajectory has no torque field — and stay on TrajOptResult's
  // `controls`.
  return jt;
}

// --- StageWindow -----------------------------------------------------------------------------

StageWindow StageWindow::all() { return StageWindow(Kind::All, 0, 0); }

StageWindow StageWindow::range(int begin, int end) {
  if (begin < 0) {
    throw std::invalid_argument("StageWindow::range: begin must be >= 0, got " +
                                std::to_string(begin) + ".");
  }
  if (end <= begin) {
    // Half-open [begin, end): end == begin is empty, end < begin is inverted. Both are invalid.
    throw std::invalid_argument(
        "StageWindow::range: range is half-open [begin, end) and must be non-empty, so end must "
        "be > begin; got begin=" +
        std::to_string(begin) + ", end=" + std::to_string(end) + ".");
  }
  return StageWindow(Kind::Range, begin, end);
}

StageWindow StageWindow::terminal() { return StageWindow(Kind::Terminal, 0, 0); }

std::vector<int> StageWindow::resolveStages(int horizon) const {
  if (horizon <= 0) {
    throw std::invalid_argument("StageWindow::resolveStages: horizon (number of stages) must be "
                                "strictly positive, got " +
                                std::to_string(horizon) + ".");
  }

  switch (kind_) {
  case Kind::All: {
    std::vector<int> stages(static_cast<std::size_t>(horizon));
    for (int i = 0; i < horizon; ++i) {
      stages[static_cast<std::size_t>(i)] = i;
    }
    return stages;
  }
  case Kind::Terminal:
    // Attaches to the terminal node, not to any stage.
    return {};
  case Kind::Range: {
    if (end_ > horizon) {
      throw std::invalid_argument(
          "StageWindow::resolveStages: range end must be <= horizon (half-open [begin, end), "
          "end <= N); got end=" +
          std::to_string(end_) + ", horizon=" + std::to_string(horizon) + ".");
    }
    std::vector<int> stages(static_cast<std::size_t>(end_ - begin_));
    for (int i = begin_; i < end_; ++i) {
      stages[static_cast<std::size_t>(i - begin_)] = i;
    }
    return stages;
  }
  }
  // Unreachable: all Kind values are handled above.
  throw std::logic_error("StageWindow::resolveStages: unhandled StageWindow::Kind.");
}

}  // namespace roboplan
