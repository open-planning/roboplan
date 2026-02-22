#include <roboplan_oink/teleop/command_filter.hpp>

#include <algorithm>
#include <cmath>

namespace roboplan {

CommandFilter::CommandFilter(const CommandFilterConfig& config) : config_(config) {}

void CommandFilter::applyDeadzone(Eigen::Ref<Eigen::Vector3d> values, double threshold) {
  for (int i = 0; i < values.size(); ++i) {
    if (std::abs(values[i]) < threshold) {
      values[i] = 0.0;
    }
  }
}

Eigen::Matrix<double, 6, 1>
CommandFilter::filter(const Eigen::Ref<const Eigen::Matrix<double, 6, 1>>& raw_twist) {
  Eigen::Vector3d linear = raw_twist.head<3>();
  Eigen::Vector3d angular = raw_twist.tail<3>();

  applyDeadzone(linear, config_.linear_deadzone);
  applyDeadzone(angular, config_.angular_deadzone);

  linear = linear.cwiseMax(-1.0).cwiseMin(1.0) * config_.linear_sensitivity;
  angular = angular.cwiseMax(-1.0).cwiseMin(1.0) * config_.angular_sensitivity;

  linear = linear.cwiseMax(-config_.max_linear_speed).cwiseMin(config_.max_linear_speed);
  angular = angular.cwiseMax(-config_.max_angular_speed).cwiseMin(config_.max_angular_speed);

  const double alpha = std::clamp(config_.smoothing_alpha, 0.0, 1.0);
  linear = alpha * linear + (1.0 - alpha) * prev_linear_;
  angular = alpha * angular + (1.0 - alpha) * prev_angular_;

  prev_linear_ = linear;
  prev_angular_ = angular;

  Eigen::Matrix<double, 6, 1> filtered;
  filtered << linear, angular;
  return filtered;
}

void CommandFilter::reset() {
  prev_linear_.setZero();
  prev_angular_.setZero();
}

void CommandFilter::setConfig(const CommandFilterConfig& config) { config_ = config; }

const CommandFilterConfig& CommandFilter::getConfig() const { return config_; }

}  // namespace roboplan
