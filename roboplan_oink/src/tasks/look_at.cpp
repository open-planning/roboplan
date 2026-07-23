#include <roboplan_oink/tasks/look_at.hpp>

#include <cmath>
#include <stdexcept>

#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/spatial/skew.hpp>

namespace {
// Alignment subspace dimension (the a × u error)
constexpr int kAlignmentDimension = 3;
// SE(3) spatial dimension of the frame Jacobian (3 linear + 3 angular)
constexpr int kFrameJacobianRows = 6;
// Minimum frame-to-target distance below which the look direction is undefined
constexpr double kMinDistance = 1e-6;
}  // namespace

namespace roboplan {

LookAtTask::LookAtTask(const Oink& oink, const Scene& scene, const std::string& frame_name,
                       const Eigen::Vector3d& target_point, double target_distance,
                       const LookAtTaskOptions& options)
    : Task(options.priority, createWeightMatrix(options.orientation_cost, options.distance_cost),
           options.task_gain, options.lm_damping),
      frame_name(frame_name), target_point(target_point), target_distance(target_distance),
      look_axis(options.look_axis), max_distance_error(options.max_distance_error) {
  if (target_distance < 0.0) {
    throw std::invalid_argument("LookAtTask: target_distance must be non-negative, got " +
                                std::to_string(target_distance));
  }
  const double axis_norm = look_axis.norm();
  if (axis_norm < kMinDistance) {
    throw std::invalid_argument("LookAtTask: look_axis must be nonzero");
  }
  look_axis /= axis_norm;

  const auto maybe_frame_id = scene.getFrameId(frame_name);
  if (!maybe_frame_id) {
    throw std::runtime_error("Frame '" + frame_name + "' not found: " + maybe_frame_id.error());
  }
  frame_id = maybe_frame_id.value();

  v_indices = oink.v_indices;

  // Pre-allocate storage: 4 rows (3 alignment + 1 distance) × group velocity DOFs columns
  initializeStorage(kLookAtDimension, v_indices.size());
  // Pre-allocate full Jacobian buffers (must span model.nv columns for computeFrameJacobian)
  full_jacobian = Eigen::MatrixXd::Zero(kFrameJacobianRows, scene.getModel().nv);
  task_jacobian_full = Eigen::MatrixXd::Zero(kLookAtDimension, scene.getModel().nv);
}

void LookAtTask::setTargetDistance(double distance) {
  if (distance < 0.0) {
    throw std::invalid_argument("LookAtTask: target_distance must be non-negative, got " +
                                std::to_string(distance));
  }
  target_distance = distance;
}

tl::expected<void, std::string> LookAtTask::computeError(const Scene& scene) {
  // Get data from scene (assumes kinematics are already up-to-date)
  auto& data = scene.getData();

  // Get current frame pose in world frame
  const pinocchio::SE3& transform_world_to_frame = data.oMf.at(frame_id);

  // Look axis in world coordinates and vector from the frame origin to the target
  const Eigen::Vector3d a = transform_world_to_frame.rotation() * look_axis;
  const Eigen::Vector3d d = target_point - transform_world_to_frame.translation();
  const double r = d.norm();

  // The look direction is undefined when the frame origin sits on the target point;
  // deactivate the task until the two separate.
  if (r < kMinDistance) {
    error_container.setZero();
    return {};
  }
  const Eigen::Vector3d u = d / r;

  // Alignment error: vanishes when the look axis faces the target, and leaves
  // rotation about the look axis free.
  error_container.head<kAlignmentDimension>() = a.cross(u);

  // Distance error: positive when farther than the desired standoff distance.
  double distance_error = r - target_distance;

  // Soft saturation of the distance error using tanh for smooth gradients, matching
  // the FrameTask position error saturation (tanh is odd, so the sign is preserved).
  if (std::isfinite(max_distance_error)) {
    distance_error = max_distance_error * std::tanh(distance_error / max_distance_error);
  }
  error_container(kAlignmentDimension) = distance_error;

  return {};
}

tl::expected<void, std::string> LookAtTask::computeJacobian(const Scene& scene) {
  // Get current joint configuration and frame pose
  const Eigen::VectorXd& q = scene.getCurrentJointPositions();
  auto& data = scene.getData();
  const pinocchio::SE3& transform_world_to_frame = data.oMf.at(frame_id);

  const Eigen::Vector3d a = transform_world_to_frame.rotation() * look_axis;
  const Eigen::Vector3d d = target_point - transform_world_to_frame.translation();
  const double r = d.norm();

  // Degenerate case: no meaningful gradient when the frame origin sits on the target.
  if (r < kMinDistance) {
    jacobian_container.setZero();
    return {};
  }
  const Eigen::Vector3d u = d / r;

  // Frame Jacobian in LOCAL_WORLD_ALIGNED: rows 0-2 are the frame origin's linear
  // velocity and rows 3-5 the angular velocity, both in world axes.
  full_jacobian.setZero();
  scene.computeFrameJacobian(q, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED,
                             full_jacobian);
  const auto J_v = full_jacobian.topRows<kAlignmentDimension>();
  const auto J_w = full_jacobian.bottomRows<kAlignmentDimension>();

  // Time derivative of e_align = a × u with a static target:
  //     ė_align = S(u)·S(a)·ω − S(a)·(I − u·uᵀ)/r · ṗ
  // where ω rotates the look axis and ṗ moves the frame origin (which changes u).
  const Eigen::Matrix3d S_a = pinocchio::skew(a);
  const Eigen::Matrix3d angular_part = pinocchio::skew(u) * S_a;
  const Eigen::Matrix3d linear_part = S_a * (Eigen::Matrix3d::Identity() - u * u.transpose()) / r;
  task_jacobian_full.topRows<kAlignmentDimension>().noalias() = angular_part * J_w;
  task_jacobian_full.topRows<kAlignmentDimension>().noalias() -= linear_part * J_v;

  // Time derivative of e_dist = r − target_distance: ṙ = −uᵀ·ṗ
  task_jacobian_full.row(kAlignmentDimension).noalias() = -u.transpose() * J_v;

  // Select the group's velocity columns. No extra negation: task_jacobian_full is
  // already de/dq, so the QP residual ||J·Δq + α·e||² drives the error to zero.
  jacobian_container = task_jacobian_full(Eigen::placeholders::all, v_indices);

  return {};
}

Eigen::MatrixXd LookAtTask::createWeightMatrix(double orientation_cost, double distance_cost) {
  Eigen::MatrixXd W = Eigen::MatrixXd::Identity(kLookAtDimension, kLookAtDimension);
  W.block<kAlignmentDimension, kAlignmentDimension>(0, 0) *= std::sqrt(orientation_cost);
  W(kAlignmentDimension, kAlignmentDimension) = std::sqrt(distance_cost);
  return W;
}

}  // namespace roboplan
