#include <roboplan_oink/tasks/frame.hpp>

#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/spatial/explog.hpp>

namespace {
// Position subspace dimension (x, y, z)
constexpr int kPositionDimension = 3;
// Orientation subspace dimension (roll, pitch, yaw)
constexpr int kOrientationDimension = 3;
}  // namespace

namespace roboplan {

tl::expected<void, std::string> FrameTask::computeError(const Scene& scene) {
  // Get the frame ID
  const auto maybe_frame_id = scene.getFrameId(frame_name);
  if (!maybe_frame_id) {
    return tl::make_unexpected("Frame '" + frame_name + "' not found: " + maybe_frame_id.error());
  }
  const auto frame_id = maybe_frame_id.value();

  // Get data from scene (assumes kinematics are already up-to-date)
  auto& data = scene.getData();

  // Get current frame pose in world frame
  const pinocchio::SE3& transform_frame_to_world = data.oMf.at(frame_id);

  // Get target pose as SE3
  const pinocchio::SE3 transform_target_to_world(target_pose.tform);

  // Compute transform from target to frame
  // T_target_to_frame = T_frame_to_world^{-1} * T_target_to_world
  const pinocchio::SE3 transform_target_to_frame =
      transform_frame_to_world.actInv(transform_target_to_world);

  // Compute error as SE3 logarithm and store in error_container
  // Error is the tangent vector from current frame to target (toward goal)
  const pinocchio::Motion error_motion = pinocchio::log6(transform_target_to_frame);
  error_container = error_motion.toVector();

  return {};
}

tl::expected<void, std::string> FrameTask::computeJacobian(const Scene& scene) {
  // Get the frame ID
  const auto maybe_frame_id = scene.getFrameId(frame_name);
  if (!maybe_frame_id) {
    return tl::make_unexpected("Frame '" + frame_name + "' not found: " + maybe_frame_id.error());
  }
  const auto frame_id = maybe_frame_id.value();

  // Get current joint configuration
  const Eigen::VectorXd& q = scene.getCurrentJointPositions();

  // Get current frame pose in world frame (assumes kinematics are already up-to-date)
  const auto& data = scene.getData();
  const pinocchio::SE3& transform_frame_to_world = data.oMf.at(frame_id);

  // Get target pose as SE3
  const pinocchio::SE3 transform_target_to_world(target_pose.tform);

  // Compute transform from target to frame
  const pinocchio::SE3 transform_target_to_frame =
      transform_frame_to_world.actInv(transform_target_to_world);

  // Compute frame Jacobian into jacobian_container
  scene.computeFrameJacobian(q, frame_id, pinocchio::ReferenceFrame::LOCAL, jacobian_container);

  // Compute logarithmic Jacobian
  pinocchio::Jlog6(transform_target_to_frame, Jlog);

  // Combine: J(q) = -Jlog6 * J_frame (in-place)
  // The negative sign ensures that with the QP formulation (min ||J*dq + gain*e||^2),
  // the solution dq = -gain * J^{-1} * e moves toward the target.
  jacobian_container.applyOnTheLeft(-Jlog);

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
