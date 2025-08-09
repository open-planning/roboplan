#include <chrono>
#include <stdexcept>

#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/geometric_path/piecewise_poly_path.hpp>
#include <toppra/parametrizer/const_accel.hpp>

#include <roboplan/core/path_utils.hpp>
#include <roboplan_toppra/toppra.hpp>

namespace roboplan {

PathParameterizerTOPPRA::PathParameterizerTOPPRA(const std::shared_ptr<Scene> scene) {

  // Extract joint velocity + acceleration limits from scene.
  // TODO: Extract only for specified joint group.
  const auto num_joints = scene->getModel().nq;
  vel_lower_limits_ = Eigen::VectorXd::Zero(num_joints);
  vel_upper_limits_ = Eigen::VectorXd::Zero(num_joints);
  acc_lower_limits_ = Eigen::VectorXd::Zero(num_joints);
  acc_upper_limits_ = Eigen::VectorXd::Zero(num_joints);

  joint_names_ = scene->getJointNames();
  for (size_t idx = 0; idx < joint_names_.size(); ++idx) {
    const auto& joint_name = joint_names_.at(idx);
    const auto& joint_info = scene->getJointInfo(joint_name);
    switch (joint_info.type) {
    case JointType::FLOATING:
    case JointType::PLANAR:
      throw std::runtime_error("Multi-DOF joints not yet supported by TOPP-RA.");
    case JointType::CONTINUOUS:
      throw std::runtime_error("Continuous joints not yet supported by TOPP-RA.");
    default:  // Prismatic, revolute, or continuous, which are single-DOF.
      if (joint_info.limits.max_velocity.size() == 0) {
        throw std::runtime_error("Velocity limit must be defined for joint '" + joint_name + "'.");
      }
      if (joint_info.limits.max_acceleration.size() == 0) {
        throw std::runtime_error("Acceleration limit must be defined for joint '" + joint_name +
                                 "'.");
      }
      const auto& max_vel = joint_info.limits.max_velocity[0];
      vel_lower_limits_(idx) = -max_vel;
      vel_upper_limits_(idx) = max_vel;
      const auto& max_acc = joint_info.limits.max_acceleration[0];
      acc_lower_limits_(idx) = -max_acc;
      acc_upper_limits_(idx) = max_acc;
    }
  }
}

tl::expected<JointTrajectory, std::string>
PathParameterizerTOPPRA::generate(const JointPath& path, const double dt,
                                  const double velocity_scale, const double acceleration_scale) {
  const auto num_pts = path.positions.size();
  if (num_pts < 2) {
    return tl::make_unexpected("Path must have at least 2 points.");
  }
  if (dt <= 0.0) {
    return tl::make_unexpected("dt must be strictly positive.");
  }
  if ((velocity_scale <= 0.0) || (velocity_scale > 1.0)) {
    return tl::make_unexpected("Velocity scale must be greater than 0.0 and less than 1.0.");
  }
  if ((acceleration_scale <= 0.0) || (acceleration_scale > 1.0)) {
    return tl::make_unexpected("Acceleration scale must be greater than 0.0 and less than 1.0.");
  }

  // // TODO: Check this based on the joint group.
  if ((joint_names_.size() != path.joint_names.size()) ||
      !std::equal(joint_names_.begin(), joint_names_.end(), path.joint_names.begin())) {
    return tl::make_unexpected("Path joint names do not match the scene joint names.");
  }

  // Create scaled velocity and acceleration constraints.
  toppra::LinearConstraintPtr vel_constraint, acc_constraint;
  vel_constraint = std::make_shared<toppra::constraint::LinearJointVelocity>(
      vel_lower_limits_ * velocity_scale, vel_upper_limits_ * velocity_scale);
  acc_constraint = std::make_shared<toppra::constraint::LinearJointAcceleration>(
      acc_lower_limits_ * acceleration_scale, acc_upper_limits_ * acceleration_scale);
  acc_constraint->discretizationType(toppra::DiscretizationType::Interpolation);
  toppra::LinearConstraintPtrs constraints = {vel_constraint, acc_constraint};

  // Create initial cubic spline with path and random times.
  toppra::Vectors path_pos_vecs, path_vel_vecs;
  path_pos_vecs.reserve(num_pts);
  path_vel_vecs.reserve(num_pts);
  std::vector<double> steps;
  steps.reserve(num_pts);
  double s = 0.0;
  for (const auto& pos : path.positions) {
    path_pos_vecs.push_back(pos);
    // TODO: Support nonzero endpoint velocities?
    path_vel_vecs.push_back(Eigen::VectorXd::Zero(pos.size()));
    steps.push_back(s);
    s += 1.0;
  }
  const auto spline =
      toppra::PiecewisePolyPath::CubicHermiteSpline(path_pos_vecs, path_vel_vecs, steps);
  const auto geom_path = std::make_shared<toppra::PiecewisePolyPath>(spline);

  // Solve TOPP-RA problem.
  toppra::PathParametrizationAlgorithmPtr algo =
      std::make_shared<toppra::algorithm::TOPPRA>(constraints, geom_path);
  const auto rc = algo->computePathParametrization();
  if (rc != toppra::ReturnCode::OK) {
    return tl::make_unexpected("TOPPRA failed with return code " +
                               std::to_string(static_cast<int>(rc)));
  }

  // Evaluate the parameterized path at the specified times.
  const auto param_data = algo->getParameterizationData();
  const auto const_acc = std::make_shared<toppra::parametrizer::ConstAccel>(
      geom_path, param_data.gridpoints, param_data.parametrization);

  JointTrajectory traj;
  traj.joint_names = path.joint_names;

  const auto t_final = const_acc->pathInterval()[1];
  const auto num_traj_pts = static_cast<size_t>(std::ceil(t_final / dt)) + 1;
  traj.times.reserve(num_traj_pts);
  traj.positions.reserve(num_traj_pts);
  traj.velocities.reserve(num_traj_pts);
  traj.accelerations.reserve(num_traj_pts);
  for (size_t i = 0; i < num_traj_pts; ++i) {
    const auto t = std::min(static_cast<double>(i) * dt, t_final);
    traj.times.push_back(t);
  }
  Eigen::Map<Eigen::VectorXd> times_vec(traj.times.data(), traj.times.size());
  for (const auto& pos : const_acc->eval(times_vec, 0)) {
    traj.positions.push_back(pos);
  }
  for (const auto& vel : const_acc->eval(times_vec, 1)) {
    traj.velocities.push_back(vel);
  }
  for (const auto& acc : const_acc->eval(times_vec, 2)) {
    traj.accelerations.push_back(acc);
  }

  return traj;
}

}  // namespace roboplan
