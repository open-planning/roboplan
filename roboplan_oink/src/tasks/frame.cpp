#include <roboplan_oink/tasks/frame.hpp>

#include <cmath>
#include <stdexcept>

#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/spatial/explog.hpp>

namespace {
// Position subspace dimension (x, y, z)
constexpr int kPositionDimension = 3;
// Orientation subspace dimension (roll, pitch, yaw)
constexpr int kOrientationDimension = 3;
// Minimum norm threshold to avoid division by zero in error saturation
constexpr double kMinNormForSaturation = 1e-9;
}  // namespace

namespace roboplan {

FrameTask::FrameTask(const Oink& oink, const Scene& scene,
                     const CartesianConfiguration& target_pose, const FrameTaskOptions& options)
    : Task(createWeightMatrix(options.position_cost, options.orientation_cost), options.task_gain,
           options.lm_damping),
      frame_name(target_pose.tip_frame), target_pose(target_pose),
      max_position_error(options.max_position_error),
      max_rotation_error(options.max_rotation_error) {
  const auto maybe_frame_id = scene.getFrameId(frame_name);
  if (!maybe_frame_id) {
    throw std::runtime_error("Frame '" + frame_name + "' not found: " + maybe_frame_id.error());
  }
  frame_id = maybe_frame_id.value();

  v_indices = oink.v_indices;

  // Pre-allocate storage: 6 rows (SE(3) task) × group velocity DOFs columns
  initializeStorage(kSpatialDimension, v_indices.size());
  // Pre-allocate full Jacobian buffer (must be 6 x model.nv for computeFrameJacobian)
  full_jacobian = Eigen::MatrixXd::Zero(kSpatialDimension, scene.getModel().nv);
}

tl::expected<void, std::string> FrameTask::computeError(const Scene& scene) {
  // Get data from scene (assumes kinematics are already up-to-date)
  auto& data = scene.getData();

  // Get current frame pose in world frame
  const pinocchio::SE3& transform_world_to_frame = data.oMf.at(frame_id);

  // Get target pose as SE3
  const pinocchio::SE3 transform_world_to_target(target_pose.tform);

  // Compute linear and angular errors from target to frame, in the world frame.
  Eigen::Vector3d e_pos =
      transform_world_to_target.translation() - transform_world_to_frame.translation();
  Eigen::Matrix3d R_err =
      transform_world_to_frame.rotation().transpose() * transform_world_to_target.rotation();
  Eigen::Vector3d e_rot = transform_world_to_frame.rotation() * pinocchio::log3(R_err);
  error_container.head<3>() = e_pos;
  error_container.tail<3>() = e_rot;

  // Soft saturation of position error using tanh for smooth gradients
  // This prevents large jumps that can invalidate CBF linearization while maintaining
  // smooth error dynamics. Uses saturate(e) = e_max * tanh(||e|| / e_max) * (e / ||e||)
  if (std::isfinite(max_position_error)) {
    Eigen::Vector3d pos_error = error_container.head<kPositionDimension>();
    const double pos_norm = pos_error.norm();
    if (pos_norm > kMinNormForSaturation) {
      const double scale = max_position_error * std::tanh(pos_norm / max_position_error) / pos_norm;
      error_container.head<kPositionDimension>() = pos_error * scale;
    }
  }

  // Soft saturation of rotation error using tanh for smooth gradients
  if (std::isfinite(max_rotation_error)) {
    Eigen::Vector3d rot_error = error_container.tail<kOrientationDimension>();
    const double rot_norm = rot_error.norm();
    if (rot_norm > kMinNormForSaturation) {
      const double scale = max_rotation_error * std::tanh(rot_norm / max_rotation_error) / rot_norm;
      error_container.tail<kOrientationDimension>() = rot_error * scale;
    }
  }

  return {};
}

tl::expected<void, std::string> FrameTask::computeJacobian(const Scene& scene) {
  // Get current joint configuration
  const Eigen::VectorXd& q = scene.getCurrentJointPositions();

  // Compute full-robot frame Jacobian, then select the group's velocity columns
  full_jacobian.setZero();
  scene.computeFrameJacobian(q, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                             full_jacobian);

  // The negative sign ensures that with the QP formulation (min ||J*dq + gain*e||^2),
  // the solution dq = -gain * J^{-1} * e moves toward the target.
  jacobian_container.noalias() = -full_jacobian(Eigen::placeholders::all, v_indices);

  return {};
}

Eigen::MatrixXd FrameTask::createWeightMatrix(double position_cost, double orientation_cost) {
  Eigen::MatrixXd W = Eigen::MatrixXd::Identity(kSpatialDimension, kSpatialDimension);
  W.block<kPositionDimension, kPositionDimension>(0, 0) *= std::sqrt(position_cost);
  W.block<kOrientationDimension, kOrientationDimension>(kPositionDimension, kPositionDimension) *=
      std::sqrt(orientation_cost);
  return W;
}

}  // namespace roboplan
