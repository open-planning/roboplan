#include <cmath>

#include <roboplan/filters/se3_low_pass_filter.hpp>

namespace roboplan {

SE3LowPassFilter::SE3LowPassFilter(double tau)
    : tau_{tau}, initialized_{false}, filtered_position_{Eigen::Vector3d::Zero()},
      filtered_quaternion_(Eigen::Quaterniond::Identity()) {}

double SE3LowPassFilter::tau() const { return tau_; }

void SE3LowPassFilter::setTau(double tau) { tau_ = tau; }

bool SE3LowPassFilter::isInitialized() const { return initialized_; }

void SE3LowPassFilter::reset(const pinocchio::SE3& pose) {
  filtered_position_ = pose.translation();
  filtered_quaternion_ = Eigen::Quaterniond(pose.rotation());
  filtered_quaternion_.normalize();
  initialized_ = true;
}

pinocchio::SE3 SE3LowPassFilter::update(const pinocchio::SE3& target_pose, double dt) {
  const Eigen::Vector3d target_position = target_pose.translation();
  Eigen::Quaterniond target_quaternion(target_pose.rotation());
  target_quaternion.normalize();

  if (!initialized_) {
    reset(target_pose);
    return target_pose;
  }

  const double alpha = 1.0 - std::exp(-dt / tau_);

  filtered_position_ = filtered_position_ + alpha * (target_position - filtered_position_);
  filtered_quaternion_ = filtered_quaternion_.slerp(alpha, target_quaternion);

  return pinocchio::SE3(filtered_quaternion_.toRotationMatrix(), filtered_position_);
}

}  // namespace roboplan
