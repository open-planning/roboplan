#include <roboplan_oink/barriers/position_barrier.hpp>

#include <array>
#include <limits>
#include <string_view>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

namespace roboplan {

namespace {
constexpr std::array<std::string_view, 3> kAxisNames = {"x", "y", "z"};
}  // namespace

PositionBarrier::PositionBarrier(const std::string& frame_name, const Eigen::Vector3d& p_min,
                                 const Eigen::Vector3d& p_max, int num_variables, double dt,
                                 const ConstraintAxisSelection& axis_selection, double gain,
                                 double safe_displacement_gain, double safety_margin)
    : Barrier(gain, dt, safe_displacement_gain, safety_margin), frame_name(frame_name),
      axis_selection(axis_selection), p_min(p_min), p_max(p_max) {
  // Validate that p_min < p_max for enabled axes with finite bounds
  const std::array<bool, 3> axes_enabled = {axis_selection.x, axis_selection.y, axis_selection.z};
  for (int i = 0; i < 3; ++i) {
    if (axes_enabled[i] && std::isfinite(p_min[i]) && std::isfinite(p_max[i])) {
      if (p_min[i] >= p_max[i]) {
        throw std::invalid_argument("PositionBarrier: p_min[" + std::string(kAxisNames[i]) +
                                    "] must be less than p_max[" + std::string(kAxisNames[i]) +
                                    "] (got " + std::to_string(p_min[i]) +
                                    " >= " + std::to_string(p_max[i]) + ")");
      }
    }
  }
  // Count active constraints (finite bounds and enabled axes)
  int num_barriers = 0;
  if (axis_selection.x) {
    if (std::isfinite(p_min[0]))
      ++num_barriers;
    if (std::isfinite(p_max[0]))
      ++num_barriers;
  }
  if (axis_selection.y) {
    if (std::isfinite(p_min[1]))
      ++num_barriers;
    if (std::isfinite(p_max[1]))
      ++num_barriers;
  }
  if (axis_selection.z) {
    if (std::isfinite(p_min[2]))
      ++num_barriers;
    if (std::isfinite(p_max[2]))
      ++num_barriers;
  }
  initializeStorage(num_barriers, num_variables);
  frame_jacobian = Eigen::MatrixXd::Zero(6, num_variables);
}

int PositionBarrier::getNumBarriers(const Scene& /*scene*/) const { return barrier_values.size(); }

tl::expected<void, std::string> PositionBarrier::computeBarrier(const Scene& scene) {
  // Cache frame ID on first call
  if (!frame_id_cached) {
    if (!scene.getModel().existFrame(frame_name)) {
      return tl::make_unexpected("Frame not found: " + frame_name);
    }
    frame_id = scene.getModel().getFrameId(frame_name);
    frame_id_cached = true;
  }

  // Get current frame position in world coordinates
  Eigen::Vector3d p = getFramePosition(scene);

  // Compute barrier values for each active constraint
  int idx = 0;

  // X axis
  if (axis_selection.x) {
    if (std::isfinite(p_min[0])) {
      barrier_values[idx] = p[0] - p_min[0];
      ++idx;
    }
    if (std::isfinite(p_max[0])) {
      barrier_values[idx] = p_max[0] - p[0];
      ++idx;
    }
  }

  // Y axis
  if (axis_selection.y) {
    if (std::isfinite(p_min[1])) {
      barrier_values[idx] = p[1] - p_min[1];
      ++idx;
    }
    if (std::isfinite(p_max[1])) {
      barrier_values[idx] = p_max[1] - p[1];
      ++idx;
    }
  }

  // Z axis
  if (axis_selection.z) {
    if (std::isfinite(p_min[2])) {
      barrier_values[idx] = p[2] - p_min[2];
      ++idx;
    }
    if (std::isfinite(p_max[2])) {
      barrier_values[idx] = p_max[2] - p[2];
      ++idx;
    }
  }

  return {};
}

tl::expected<void, std::string> PositionBarrier::computeJacobian(const Scene& scene) {
  // Compute frame Jacobian (6 x nv) in world frame
  // Using WORLD reference frame so no additional rotation is needed
  // since our position bounds are specified in world coordinates
  const Eigen::VectorXd& q = scene.getCurrentJointPositions();
  scene.computeFrameJacobian(q, frame_id, pinocchio::ReferenceFrame::WORLD, frame_jacobian);

  // Pinocchio frame Jacobian layout with WORLD reference frame:
  //   Rows 0-2: linear velocity (dp_world/dq) - this is what we need
  //   Rows 3-5: angular velocity (d_omega_world/dq)
  // Note: With LOCAL or LOCAL_WORLD_ALIGNED, the ordering may differ

  // Build barrier Jacobians from the linear velocity rows
  int idx = 0;

  // X axis
  if (axis_selection.x) {
    if (std::isfinite(p_min[0])) {
      jacobian_container.row(idx) = frame_jacobian.row(0);
      ++idx;
    }
    if (std::isfinite(p_max[0])) {
      jacobian_container.row(idx) = -frame_jacobian.row(0);
      ++idx;
    }
  }

  // Y axis
  if (axis_selection.y) {
    if (std::isfinite(p_min[1])) {
      jacobian_container.row(idx) = frame_jacobian.row(1);
      ++idx;
    }
    if (std::isfinite(p_max[1])) {
      jacobian_container.row(idx) = -frame_jacobian.row(1);
      ++idx;
    }
  }

  // Z axis
  if (axis_selection.z) {
    if (std::isfinite(p_min[2])) {
      jacobian_container.row(idx) = frame_jacobian.row(2);
      ++idx;
    }
    if (std::isfinite(p_max[2])) {
      jacobian_container.row(idx) = -frame_jacobian.row(2);
      ++idx;
    }
  }

  return {};
}

Eigen::Vector3d PositionBarrier::getFramePosition(const Scene& scene) const {
  return scene.getData().oMf[frame_id].translation();
}

tl::expected<double, std::string>
PositionBarrier::evaluateAtConfiguration(const pinocchio::Model& model, pinocchio::Data& data,
                                         const Eigen::VectorXd& q) const {
  // Cache frame ID on first call (same mechanism as computeBarrier)
  if (!frame_id_cached) {
    if (!model.existFrame(frame_name)) {
      return tl::make_unexpected("Frame not found: " + frame_name);
    }
    frame_id = model.getFrameId(frame_name);
    frame_id_cached = true;
  }

  // Compute FK for the candidate configuration
  pinocchio::forwardKinematics(model, data, q);

  // Update frame placement
  pinocchio::updateFramePlacement(model, data, frame_id);

  // Get frame position
  const Eigen::Vector3d pos = data.oMf[frame_id].translation();

  // Compute minimum barrier value across all enabled constraints
  double min_h = std::numeric_limits<double>::infinity();

  if (axis_selection.x) {
    if (std::isfinite(p_min[0])) {
      min_h = std::min(min_h, pos[0] - p_min[0]);
    }
    if (std::isfinite(p_max[0])) {
      min_h = std::min(min_h, p_max[0] - pos[0]);
    }
  }

  if (axis_selection.y) {
    if (std::isfinite(p_min[1])) {
      min_h = std::min(min_h, pos[1] - p_min[1]);
    }
    if (std::isfinite(p_max[1])) {
      min_h = std::min(min_h, p_max[1] - pos[1]);
    }
  }

  if (axis_selection.z) {
    if (std::isfinite(p_min[2])) {
      min_h = std::min(min_h, pos[2] - p_min[2]);
    }
    if (std::isfinite(p_max[2])) {
      min_h = std::min(min_h, p_max[2] - pos[2]);
    }
  }

  return min_h;
}

}  // namespace roboplan
