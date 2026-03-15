#include <roboplan_oink/barriers/position_barrier.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

namespace roboplan {

PositionBarrier::PositionBarrier(const std::string& frame_name_, const Eigen::Vector3d& p_min_,
                                 const Eigen::Vector3d& p_max_, int num_variables_, double gain,
                                 double dt, double safe_displacement_gain, double safety_margin)
    : Barrier(gain, dt, safe_displacement_gain, safety_margin), frame_name(frame_name_),
      indices({0, 1, 2}), p_min(p_min_), p_max(p_max_) {
  // Count active constraints (finite bounds)
  int num_barriers = 0;
  for (int i = 0; i < 3; ++i) {
    if (std::isfinite(p_min_[i])) {
      num_barriers++;
    }
    if (std::isfinite(p_max_[i])) {
      num_barriers++;
    }
  }
  initializeStorage(num_barriers, num_variables_);
  frame_jacobian = Eigen::MatrixXd::Zero(6, num_variables_);
}

PositionBarrier::PositionBarrier(const std::string& frame_name_, const std::vector<int>& indices_,
                                 const Eigen::VectorXd& p_min_, const Eigen::VectorXd& p_max_,
                                 int num_variables_, double gain, double dt,
                                 double safe_displacement_gain, double safety_margin)
    : Barrier(gain, dt, safe_displacement_gain, safety_margin), frame_name(frame_name_),
      indices(indices_), p_min(p_min_), p_max(p_max_) {
  // Validate indices
  for (int idx : indices_) {
    if (idx < 0 || idx > 2) {
      throw std::invalid_argument("Axis index must be 0, 1, or 2 (x, y, z)");
    }
  }

  // Validate sizes match
  if (static_cast<int>(indices_.size()) != p_min_.size() ||
      static_cast<int>(indices_.size()) != p_max_.size()) {
    throw std::invalid_argument("p_min and p_max size must match indices size");
  }

  // Count active constraints (finite bounds)
  int num_barriers = 0;
  for (int i = 0; i < static_cast<int>(indices_.size()); ++i) {
    if (std::isfinite(p_min_[i])) {
      num_barriers++;
    }
    if (std::isfinite(p_max_[i])) {
      num_barriers++;
    }
  }
  initializeStorage(num_barriers, num_variables_);
  frame_jacobian = Eigen::MatrixXd::Zero(6, num_variables_);
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
  // Note: p_min[i] and p_max[i] correspond to indices[i], not to axis i directly
  // For full constructor with indices={0,1,2}, p_min/p_max are Vector3d so this works
  // For selective constructor, p_min/p_max have size indices.size()
  int idx = 0;
  for (size_t i = 0; i < indices.size(); ++i) {
    int axis = indices[i];
    double p_i = p[axis];  // Position along this axis in world frame

    // Lower bound barrier: h = p - p_min >= 0 when p >= p_min (safe)
    if (std::isfinite(p_min[i])) {
      barrier_values[idx++] = p_i - p_min[i];
    }

    // Upper bound barrier: h = p_max - p >= 0 when p <= p_max (safe)
    if (std::isfinite(p_max[i])) {
      barrier_values[idx++] = p_max[i] - p_i;
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
  for (size_t i = 0; i < indices.size(); ++i) {
    int axis = indices[i];  // 0=x, 1=y, 2=z

    // Lower bound: h = p - p_min, so J_h = dp/dq = J_p[axis, :]
    if (std::isfinite(p_min[i])) {
      jacobian_container.row(idx++) = frame_jacobian.row(axis);
    }

    // Upper bound: h = p_max - p, so J_h = -dp/dq = -J_p[axis, :]
    if (std::isfinite(p_max[i])) {
      jacobian_container.row(idx++) = -frame_jacobian.row(axis);
    }
  }

  return {};
}

Eigen::Vector3d PositionBarrier::getFramePosition(const Scene& scene) const {
  return scene.getData().oMf[frame_id].translation();
}

}  // namespace roboplan
