#include <chrono>
#include <stdexcept>

#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/parametrizer/const_accel.hpp>

#include <roboplan/core/path_utils.hpp>
#include <roboplan/core/scene_utils.hpp>
#include <roboplan_toppra/toppra.hpp>

namespace roboplan {

PathParameterizerTOPPRA::PathParameterizerTOPPRA(const std::shared_ptr<Scene> scene,
                                                 const std::string& group_name)
    : scene_{scene}, group_name_{group_name} {
  // Extract joint velocity + acceleration limits from the scene.
  const auto maybe_joint_velocity_limits = scene_->getVelocityLimitVectors(group_name_);
  if (!maybe_joint_velocity_limits) {
    throw std::runtime_error("Could not initialize TOPP-RA path parameterizer: " +
                             maybe_joint_velocity_limits.error());
  }
  vel_lower_limits_ = maybe_joint_velocity_limits->first;
  vel_upper_limits_ = maybe_joint_velocity_limits->second;

  const auto maybe_joint_acceleration_limits = scene_->getAccelerationLimitVectors(group_name_);
  if (!maybe_joint_acceleration_limits) {
    throw std::runtime_error("Could not initialize TOPP-RA path parameterizer: " +
                             maybe_joint_acceleration_limits.error());
  }
  acc_lower_limits_ = maybe_joint_acceleration_limits->first;
  acc_upper_limits_ = maybe_joint_acceleration_limits->second;

  // Get the continuous joint indices for unwrapping positions.
  const auto maybe_joint_group_info = scene_->getJointGroupInfo(group_name_);
  if (!maybe_joint_group_info) {
    throw std::runtime_error("Could not initialize TOPP-RA path parameterizer: " +
                             maybe_joint_group_info.error());
  }
  const auto& joint_group_info = maybe_joint_group_info.value();
  joint_names_ = joint_group_info.joint_names;

  for (size_t j_idx = 0; j_idx < joint_group_info.joint_names.size(); ++j_idx) {
    const auto& joint_name = joint_group_info.joint_names.at(j_idx);
    const auto maybe_joint_info = scene_->getJointInfo(joint_name);
    if (!maybe_joint_info) {
      throw std::runtime_error("Failed to instantiate TOPP-RA: " + maybe_joint_info.error());
    }
    if (maybe_joint_info->type == JointType::CONTINUOUS) {
      continuous_joint_indices_.push_back(j_idx);
    }
  }
}

tl::expected<toppra::Vectors, std::string>
PathParameterizerTOPPRA::getPathPositionVectors(const JointPath& path) {
  const auto num_pts = path.positions.size();
  toppra::Vectors path_pos_vecs;
  path_pos_vecs.reserve(num_pts);
  for (size_t idx = 0; idx < path.positions.size(); ++idx) {
    const auto& pos = path.positions.at(idx);
    auto maybe_collapsed_pos = collapseContinuousJointPositions(*scene_, group_name_, pos);
    if (!maybe_collapsed_pos) {
      return tl::make_unexpected(maybe_collapsed_pos.error());
    }
    auto curr_collapsed = maybe_collapsed_pos.value();

    // For continuous joints we have to ensure that we take "the short way around" in the spline.
    // If the distance to the preview point is greater than PI, then we either add or subtract
    // 2*PI to this point to ensure that we don't travel further than we need to.
    if (idx > 0) {
      const auto& prev_collapsed = path_pos_vecs.at(idx - 1);
      for (auto j_idx : continuous_joint_indices_) {
        const auto diff = curr_collapsed(j_idx) - prev_collapsed(j_idx);
        if (diff > M_PI) {
          curr_collapsed(j_idx) -= 2.0 * M_PI;
        } else if (diff < -M_PI) {
          curr_collapsed(j_idx) += 2.0 * M_PI;
        }
      }
    }
    path_pos_vecs.push_back(curr_collapsed);
  }
  return path_pos_vecs;
}

std::shared_ptr<toppra::PiecewisePolyPath>
PathParameterizerTOPPRA::generateCubicSpline(const toppra::Vectors& path_pos_vecs) {
  const auto num_pts = path_pos_vecs.size();
  Eigen::VectorXd times(num_pts);
  double s = 0.0;
  for (size_t idx = 0; idx < num_pts; ++idx) {
    times(idx) = s;
    s += 1.0;
  }

  // Set boundary conditions to zero velocity and acceleration at both endpoints.
  toppra::BoundaryCond bc{2, Eigen::VectorXd::Zero(path_pos_vecs.at(0).size())};
  toppra::BoundaryCondFull bc_full{bc, bc};

  const auto spline = toppra::PiecewisePolyPath::CubicSpline(path_pos_vecs, times, bc_full);
  return std::make_shared<toppra::PiecewisePolyPath>(spline);
}

std::shared_ptr<toppra::PiecewisePolyPath>
PathParameterizerTOPPRA::generateCubicHermiteSpline(const toppra::Vectors& path_pos_vecs) {
  const auto num_pts = path_pos_vecs.size();
  toppra::Vectors path_vel_vecs;
  path_vel_vecs.reserve(num_pts);
  std::vector<double> steps;
  steps.reserve(num_pts);
  double s = 0.0;
  for (size_t idx = 0; idx < num_pts; ++idx) {
    path_vel_vecs.push_back(Eigen::VectorXd::Zero(path_pos_vecs.at(0).size()));
    steps.push_back(s);
    s += 1.0;
  }
  const auto spline =
      toppra::PiecewisePolyPath::CubicHermiteSpline(path_pos_vecs, path_vel_vecs, steps);
  return std::make_shared<toppra::PiecewisePolyPath>(spline);
}

tl::expected<JointTrajectory, std::string> PathParameterizerTOPPRA::generate(
    const JointPath& path, const double dt, const SplineFittingMode mode,
    const double velocity_scale, const double acceleration_scale, const int max_adaptive_iterations,
    const double max_adaptive_step_size) {
  if (path.positions.size() < 2) {
    return tl::make_unexpected("Path must have at least 2 points.");
  }
  if ((joint_names_.size() != path.joint_names.size()) ||
      !std::equal(joint_names_.begin(), joint_names_.end(), path.joint_names.begin())) {
    return tl::make_unexpected("Path joint names do not match the scene joint names.");
  }
  if (dt <= 0.0) {
    return tl::make_unexpected("dt must be strictly positive.");
  }
  if ((velocity_scale <= 0.0) || (velocity_scale > 1.0)) {
    return tl::make_unexpected(
        "Velocity scale must be greater than 0.0 and less than or equal to 1.0.");
  }
  if ((acceleration_scale <= 0.0) || (acceleration_scale > 1.0)) {
    return tl::make_unexpected(
        "Acceleration scale must be greater than 0.0 and less than or equal to 1.0.");
  }

  // Create scaled velocity and acceleration constraints.
  toppra::LinearConstraintPtr vel_constraint, acc_constraint;
  vel_constraint = std::make_shared<toppra::constraint::LinearJointVelocity>(
      vel_lower_limits_ * velocity_scale, vel_upper_limits_ * velocity_scale);
  acc_constraint = std::make_shared<toppra::constraint::LinearJointAcceleration>(
      acc_lower_limits_ * acceleration_scale, acc_upper_limits_ * acceleration_scale);
  acc_constraint->discretizationType(toppra::DiscretizationType::Interpolation);
  toppra::LinearConstraintPtrs constraints = {vel_constraint, acc_constraint};

  auto maybe_path_pos_vecs = getPathPositionVectors(path);
  if (!maybe_path_pos_vecs) {
    return tl::make_unexpected("Failed to extract position vectors from path: " +
                               maybe_path_pos_vecs.error());
  }
  auto path_pos_vecs = maybe_path_pos_vecs.value();

  // Parse the spline fitting mode and set the options accordingly.
  // The basic rules are:
  // - If Hermite mode is enabled, we don't need to iterate or check collisions.
  // - If cubic mode is enabled, we just do one iteration with collision checking.
  // - If adaptive mode is enabled, we do need to iterate by checking collisisions.
  int max_collision_iterations;
  switch (mode) {
  case SplineFittingMode::Hermite:
    max_collision_iterations = 0;
    break;
  case SplineFittingMode::Cubic:
    max_collision_iterations = 1;
    break;
  case SplineFittingMode::Adaptive:
    max_collision_iterations = max_adaptive_iterations;
    break;
  }

  bool found_collision_free_path = false;
  std::shared_ptr<toppra::PiecewisePolyPath> geom_path;
  for (int idx = 0; idx < max_collision_iterations; ++idx) {
    // Create the cubic spline.
    geom_path = generateCubicSpline(path_pos_vecs);

    // Collision check the spline.
    // This assumes the initial path has time indices for each point at exactly increments of 1.0
    // (which is the case). These time values will later be modified by the final TOPP-RA algorithm.
    int last_collision_index = -1;
    size_t points_added = 0;
    const auto time_points = geom_path->proposeGridpoints(
        /* max_segment_error */ 1.0e-4, /* max_iteration */ 100, max_adaptive_step_size);
    for (const auto t : time_points) {
      // If the current point has already been added, can skip to the next time point.
      const auto t_idx = static_cast<int>(t);
      if (last_collision_index == t_idx) {
        continue;
      }

      const auto q = geom_path->eval_single(t, 0);
      const auto maybe_q_expanded = expandContinuousJointPositions(*scene_, group_name_, q);
      if (!maybe_q_expanded) {
        return tl::make_unexpected("Failed to collision check geometric path: " +
                                   maybe_q_expanded.error());
      }
      const auto q_full = scene_->toFullJointPositions(group_name_, maybe_q_expanded.value());

      // If a collision is found, add a waypoint in the middle of the current and next point.
      // Don't add points in the final iteration, as it is not needed.
      if (scene_->hasCollisions(q_full)) {
        last_collision_index = t_idx;
        if (idx < max_collision_iterations - 1) {
          const auto& q_prev = path_pos_vecs.at(t_idx + points_added);
          const auto& q_next = path_pos_vecs.at(t_idx + points_added + 1);
          const auto q_interp = 0.5 * (q_prev + q_next);
          path_pos_vecs.insert(path_pos_vecs.begin() + t_idx + points_added + 1, q_interp);
          ++points_added;
        }
      }
    }

    if (last_collision_index == -1) {
      found_collision_free_path = true;
      break;
    }
  }

  // If necessary, fall back to a Hermite cubic spline using the original path.
  // This happens with Hermite mode or if we didn't find a collision-free path with other modes.
  if (!found_collision_free_path) {
    geom_path = generateCubicHermiteSpline(getPathPositionVectors(path).value());
  }

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
    const auto maybe_expanded_pos = expandContinuousJointPositions(*scene_, group_name_, pos);
    if (!maybe_expanded_pos) {
      return tl::make_unexpected("Failed to compute path parameterization: " +
                                 maybe_expanded_pos.error());
    }
    traj.positions.push_back(maybe_expanded_pos.value());
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
