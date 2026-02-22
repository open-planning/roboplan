#include <roboplan_oink/teleop/teleop_target.hpp>

#include <pinocchio/spatial/explog.hpp>

namespace {
constexpr double kMinTwistNorm = 1e-12;
}  // namespace

namespace roboplan {

TeleopTarget::TeleopTarget(const Eigen::Matrix4d& initial_pose, const WorkspaceBounds& bounds,
                           TeleopFrame frame)
    : target_(initial_pose), bounds_(bounds), frame_(frame) {
  clampWorkspace();
}

void TeleopTarget::update(const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& twist, double dt) {
  update(twist, dt, frame_);
}

void TeleopTarget::update(const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& twist, double dt,
                          TeleopFrame frame) {
  if (dt <= 0.0) {
    return;
  }

  const Eigen::Matrix<double, 6, 1> scaled_twist = twist * dt;
  if (scaled_twist.norm() <= kMinTwistNorm) {
    return;
  }

  const Eigen::Matrix4d delta =
      pinocchio::exp6(pinocchio::Motion(scaled_twist)).toHomogeneousMatrix();
  if (frame == TeleopFrame::EndEffector) {
    target_ = target_ * delta;
  } else {
    target_ = delta * target_;
  }

  clampWorkspace();
}

void TeleopTarget::resetTo(const Eigen::Matrix4d& pose) {
  target_ = pose;
  clampWorkspace();
}

Eigen::Matrix4d TeleopTarget::getTarget() const { return target_; }

void TeleopTarget::setFrame(TeleopFrame frame) { frame_ = frame; }

TeleopFrame TeleopTarget::getFrame() const { return frame_; }

void TeleopTarget::setBounds(const WorkspaceBounds& bounds) {
  bounds_ = bounds;
  clampWorkspace();
}

const WorkspaceBounds& TeleopTarget::getBounds() const { return bounds_; }

void TeleopTarget::clampWorkspace() {
  target_.block<3, 1>(0, 3) =
      target_.block<3, 1>(0, 3).cwiseMax(bounds_.min_xyz).cwiseMin(bounds_.max_xyz);
}

}  // namespace roboplan
