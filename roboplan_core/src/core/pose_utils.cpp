#include <roboplan/core/pose_utils.hpp>

#include <Eigen/Geometry>

namespace roboplan {

std::pair<double, double> poseError(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b) {
  const double position_error = (a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).norm();
  const Eigen::Matrix3d relative_rotation = a.block<3, 3>(0, 0).transpose() * b.block<3, 3>(0, 0);
  const double orientation_error = Eigen::AngleAxisd(relative_rotation).angle();
  return {position_error, orientation_error};
}

Eigen::Matrix4d interpolatePose(const Eigen::Matrix4d& start, const Eigen::Matrix4d& end,
                                double fraction) {
  // Linear interpolation for position, SLERP for orientation.
  const Eigen::Vector3d position =
      start.block<3, 1>(0, 3) + fraction * (end.block<3, 1>(0, 3) - start.block<3, 1>(0, 3));
  Eigen::Quaterniond q_start(start.block<3, 3>(0, 0));
  Eigen::Quaterniond q_end(end.block<3, 3>(0, 0));
  q_start.normalize();
  q_end.normalize();
  const Eigen::Quaterniond q_interp = q_start.slerp(fraction, q_end);

  Eigen::Matrix4d out = Eigen::Matrix4d::Identity();
  out.block<3, 3>(0, 0) = q_interp.toRotationMatrix();
  out.block<3, 1>(0, 3) = position;
  return out;
}

}  // namespace roboplan
