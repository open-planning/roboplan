#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include <toppra/constraint/linear_joint_acceleration.hpp>
#include <toppra/constraint/linear_joint_velocity.hpp>
#include <toppra/parametrizer/const_accel.hpp>

#include <roboplan/core/path_utils.hpp>
#include <roboplan/core/scene_utils.hpp>
#include <roboplan_toppra/toppra.hpp>

namespace {
/// @brief Floor on a path segment's normalized length, to keep the normalized coordinate strictly
/// increasing (and avoid division by zero) when two waypoints coincide for a non-standard joint.
constexpr double kMinSegmentLength = 1.0e-9;

/// @brief Step (in path-fraction units) used to finite-difference geodesics when reconstructing
/// translational velocities and accelerations for non-standard joints.
constexpr double kFractionEpsilon = 1.0e-6;

/// @brief Wraps an angle to the interval (-pi, pi].
double wrapAngle(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }
}  // namespace

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

  // Get the continuous joint position indices for unwrapping positions.
  const auto maybe_joint_group_info = scene_->getJointGroupInfo(group_name_);
  if (!maybe_joint_group_info) {
    throw std::runtime_error("Could not initialize TOPP-RA path parameterizer: " +
                             maybe_joint_group_info.error());
  }
  const auto& joint_group_info = maybe_joint_group_info.value();
  joint_names_ = joint_group_info.joint_names;
  q_indices_ = joint_group_info.q_indices;

  // Determine the dimension of the normalized representation and which joints are non-standard.
  // Standard joints contribute their velocity DOFs directly; non-standard (Lie-group) joints each
  // contribute a single normalized arc-length coordinate.
  for (const auto& joint_name : joint_group_info.joint_names) {
    const auto maybe_joint_info = scene_->getJointInfo(joint_name);
    if (!maybe_joint_info) {
      throw std::runtime_error("Failed to instantiate TOPP-RA: " + maybe_joint_info.error());
    }
    const auto& joint_info = maybe_joint_info.value();
    if (joint_info.mimic_info) {
      continue;  // Mimic joints occupy no degrees of freedom.
    }

    if (isNonstandardJoint(joint_info.type)) {
      has_nonstandard_joints_ = true;
      norm_dim_ += 1;
    } else {
      norm_dim_ += joint_info.num_velocity_dofs;
    }
  }
}

bool PathParameterizerTOPPRA::isNonstandardJoint(JointType type) {
  return type == JointType::CONTINUOUS || type == JointType::PLANAR || type == JointType::FLOATING;
}

std::pair<size_t, double>
PathParameterizerTOPPRA::locateSegment(const std::vector<double>& cumulative, double value) {
  const size_t num_segments = cumulative.size() - 1;
  // Find the segment [k, k+1) such that cumulative[k] <= value < cumulative[k+1].
  const auto it = std::upper_bound(cumulative.begin(), cumulative.end(), value);
  size_t seg = (it == cumulative.begin())
                   ? 0
                   : static_cast<size_t>(std::distance(cumulative.begin(), it)) - 1;
  seg = std::min(seg, num_segments - 1);
  const double length = cumulative[seg + 1] - cumulative[seg];
  const double fraction =
      (length > 0.0) ? std::clamp((value - cumulative[seg]) / length, 0.0, 1.0) : 0.0;
  return {seg, fraction};
}

tl::expected<toppra::Vectors, std::string>
PathParameterizerTOPPRA::getPathPositionVectors(const JointPath& path) {
  // Standard code path: with no non-standard joints, the collapsed representation is already a
  // valid Euclidean parameterization, so we simply collapse each waypoint.
  if (!has_nonstandard_joints_) {
    toppra::Vectors path_pos_vecs;
    path_pos_vecs.reserve(path.positions.size());
    for (const auto& pos : path.positions) {
      auto maybe_collapsed_pos = collapseContinuousJointPositions(*scene_, group_name_, pos);
      if (!maybe_collapsed_pos) {
        return tl::make_unexpected(maybe_collapsed_pos.error());
      }
      path_pos_vecs.push_back(maybe_collapsed_pos.value());
    }
    return path_pos_vecs;
  }

  // Normalized code path: represent each non-standard joint by a single normalized arc-length
  // coordinate so the spline tracks the Lie-group geodesic exactly without dense resampling.
  const size_t num_waypoints = path.positions.size();
  const auto& model = scene_->getModel();

  // Precompute the full-model and collapsed configurations at each waypoint.
  full_waypoints_.clear();
  full_waypoints_.reserve(num_waypoints);
  std::vector<Eigen::VectorXd> collapsed(num_waypoints);
  for (size_t k = 0; k < num_waypoints; ++k) {
    full_waypoints_.push_back(scene_->toFullJointPositions(group_name_, path.positions.at(k)));
    auto maybe_collapsed =
        collapseContinuousJointPositions(*scene_, group_name_, path.positions.at(k));
    if (!maybe_collapsed) {
      return tl::make_unexpected(maybe_collapsed.error());
    }
    collapsed.at(k) = maybe_collapsed.value();
  }

  nonstandard_cumulative_.clear();
  norm_vel_lower_limits_ = Eigen::VectorXd::Zero(norm_dim_);
  norm_vel_upper_limits_ = Eigen::VectorXd::Zero(norm_dim_);
  norm_acc_lower_limits_ = Eigen::VectorXd::Zero(norm_dim_);
  norm_acc_upper_limits_ = Eigen::VectorXd::Zero(norm_dim_);

  toppra::Vectors path_pos_vecs(num_waypoints, Eigen::VectorXd::Zero(norm_dim_));

  // Computes the geodesic distance between two waypoints restricted to a subset of full-config
  // position indices, by copying waypoint k and overwriting only those indices with waypoint k+1.
  const auto masked_distance = [&](size_t k, const std::vector<Eigen::Index>& indices) {
    Eigen::VectorXd q_masked = full_waypoints_.at(k);
    for (const auto idx : indices) {
      q_masked(idx) = full_waypoints_.at(k + 1)(idx);
    }
    return scene_->configurationDistance(full_waypoints_.at(k), q_masked);
  };

  size_t norm_col = 0;       // Column in the normalized representation.
  size_t collapsed_col = 0;  // Column in the collapsed representation.
  size_t vel_col = 0;        // Column in the velocity/acceleration representation.
  for (const auto& joint_name : joint_names_) {
    const auto joint_info = scene_->getJointInfo(joint_name).value();
    if (joint_info.mimic_info) {
      continue;
    }

    if (!isNonstandardJoint(joint_info.type)) {
      // Copy the joint's collapsed positions and limits straight through.
      for (size_t d = 0; d < joint_info.num_velocity_dofs; ++d) {
        for (size_t k = 0; k < num_waypoints; ++k) {
          path_pos_vecs.at(k)(norm_col + d) = collapsed.at(k)(collapsed_col + d);
        }
        norm_vel_lower_limits_(norm_col + d) = vel_lower_limits_(vel_col + d);
        norm_vel_upper_limits_(norm_col + d) = vel_upper_limits_(vel_col + d);
        norm_acc_lower_limits_(norm_col + d) = acc_lower_limits_(vel_col + d);
        norm_acc_upper_limits_(norm_col + d) = acc_upper_limits_(vel_col + d);
      }
      norm_col += joint_info.num_velocity_dofs;
      collapsed_col += joint_info.num_position_dofs;
      vel_col += joint_info.num_velocity_dofs;
      continue;
    }

    // Identify the joint's rotational and full position-index blocks, and the linear/angular
    // limits. Splitting the geodesic this way recovers the true body-frame linear distance (the arc
    // length), since the SE(2)/SE(3) log decomposes orthogonally: L^2 = d_linear^2 + d_angular^2.
    // The world chord between waypoints would underestimate the arc on curving (translating +
    // rotating) segments, so we use d_linear = sqrt(L^2 - d_angular^2) instead.
    const auto q_off = static_cast<Eigen::Index>(model.idx_qs[model.getJointId(joint_name)]);
    std::vector<Eigen::Index> all_indices, rotation_indices;
    double max_linear_velocity = std::numeric_limits<double>::infinity();
    double max_linear_acceleration = std::numeric_limits<double>::infinity();
    double max_angular_velocity = std::numeric_limits<double>::infinity();
    double max_angular_acceleration = std::numeric_limits<double>::infinity();
    switch (joint_info.type) {
    case JointType::CONTINUOUS:
      all_indices = {q_off, q_off + 1};
      rotation_indices = all_indices;
      max_angular_velocity = joint_info.limits.max_velocity(0);
      max_angular_acceleration = joint_info.limits.max_acceleration(0);
      break;
    case JointType::PLANAR:
      all_indices = {q_off, q_off + 1, q_off + 2, q_off + 3};
      rotation_indices = {q_off + 2, q_off + 3};
      max_linear_velocity =
          std::min(joint_info.limits.max_velocity(0), joint_info.limits.max_velocity(1));
      max_linear_acceleration =
          std::min(joint_info.limits.max_acceleration(0), joint_info.limits.max_acceleration(1));
      max_angular_velocity = joint_info.limits.max_velocity(2);
      max_angular_acceleration = joint_info.limits.max_acceleration(2);
      break;
    default:
      return tl::make_unexpected("Floating joints are not yet supported by TOPP-RA.");
    }

    // Accumulate the normalized coordinate as the per-segment minimum traversal time, and derive a
    // single (conservative) acceleration limit for the normalized coordinate.
    std::vector<double> cumulative(num_waypoints, 0.0);
    double accel_limit = std::numeric_limits<double>::infinity();
    for (size_t k = 0; k + 1 < num_waypoints; ++k) {
      const double total = masked_distance(k, all_indices);
      const double angular = masked_distance(k, rotation_indices);
      const double linear = std::sqrt(std::max(0.0, total * total - angular * angular));

      double segment_length = 0.0;
      if (std::isfinite(max_linear_velocity) && max_linear_velocity > 0.0) {
        segment_length = std::max(segment_length, linear / max_linear_velocity);
      }
      if (std::isfinite(max_angular_velocity) && max_angular_velocity > 0.0) {
        segment_length = std::max(segment_length, angular / max_angular_velocity);
      }
      segment_length = std::max(segment_length, kMinSegmentLength);
      cumulative[k + 1] = cumulative[k] + segment_length;

      if (linear > kMinSegmentLength && std::isfinite(max_linear_acceleration) &&
          max_linear_acceleration > 0.0) {
        accel_limit = std::min(accel_limit, max_linear_acceleration * segment_length / linear);
      }
      if (angular > kMinSegmentLength && std::isfinite(max_angular_acceleration) &&
          max_angular_acceleration > 0.0) {
        accel_limit = std::min(accel_limit, max_angular_acceleration * segment_length / angular);
      }
    }

    for (size_t k = 0; k < num_waypoints; ++k) {
      path_pos_vecs.at(k)(norm_col) = cumulative[k];
    }
    norm_vel_lower_limits_(norm_col) = -1.0;
    norm_vel_upper_limits_(norm_col) = 1.0;
    norm_acc_lower_limits_(norm_col) = -accel_limit;
    norm_acc_upper_limits_(norm_col) = accel_limit;
    nonstandard_cumulative_.push_back(std::move(cumulative));

    norm_col += 1;
    collapsed_col += static_cast<size_t>(joint_info.type == JointType::PLANAR ? 3 : 1);
    vel_col += joint_info.num_velocity_dofs;
  }

  return path_pos_vecs;
}

Eigen::VectorXd
PathParameterizerTOPPRA::normalizedToGroupPositions(const Eigen::VectorXd& q_norm) const {
  Eigen::VectorXd group_pos(q_indices_.size());
  const auto& model = scene_->getModel();

  size_t norm_col = 0;
  Eigen::Index expanded_col = 0;
  int nonstandard_idx = 0;
  for (const auto& joint_name : joint_names_) {
    const auto joint_info = scene_->getJointInfo(joint_name).value();
    if (joint_info.mimic_info) {
      continue;
    }

    if (!isNonstandardJoint(joint_info.type)) {
      for (size_t d = 0; d < joint_info.num_velocity_dofs; ++d) {
        group_pos(expanded_col + static_cast<Eigen::Index>(d)) = q_norm(norm_col + d);
      }
      norm_col += joint_info.num_velocity_dofs;
      expanded_col += static_cast<Eigen::Index>(joint_info.num_position_dofs);
      continue;
    }

    const auto& cumulative = nonstandard_cumulative_.at(nonstandard_idx);
    const auto [segment, fraction] = locateSegment(cumulative, q_norm(norm_col));
    const Eigen::VectorXd interpolated =
        scene_->interpolate(full_waypoints_.at(segment), full_waypoints_.at(segment + 1), fraction);
    const auto q_off = static_cast<Eigen::Index>(model.idx_qs[model.getJointId(joint_name)]);
    for (size_t d = 0; d < joint_info.num_position_dofs; ++d) {
      group_pos(expanded_col + static_cast<Eigen::Index>(d)) =
          interpolated(q_off + static_cast<Eigen::Index>(d));
    }
    norm_col += 1;
    expanded_col += static_cast<Eigen::Index>(joint_info.num_position_dofs);
    ++nonstandard_idx;
  }

  return group_pos;
}

void PathParameterizerTOPPRA::normalizedToVelocitiesAndAccelerations(
    const Eigen::VectorXd& q_norm, const Eigen::VectorXd& dq_norm, const Eigen::VectorXd& ddq_norm,
    Eigen::VectorXd& velocities, Eigen::VectorXd& accelerations) const {
  velocities = Eigen::VectorXd::Zero(vel_lower_limits_.size());
  accelerations = Eigen::VectorXd::Zero(vel_lower_limits_.size());
  const auto& model = scene_->getModel();

  size_t norm_col = 0;
  size_t vel_col = 0;
  int nonstandard_idx = 0;
  for (const auto& joint_name : joint_names_) {
    const auto joint_info = scene_->getJointInfo(joint_name).value();
    if (joint_info.mimic_info) {
      continue;
    }

    if (!isNonstandardJoint(joint_info.type)) {
      for (size_t d = 0; d < joint_info.num_velocity_dofs; ++d) {
        velocities(vel_col + d) = dq_norm(norm_col + d);
        accelerations(vel_col + d) = ddq_norm(norm_col + d);
      }
      norm_col += joint_info.num_velocity_dofs;
      vel_col += joint_info.num_velocity_dofs;
      continue;
    }

    // Reconstruct body-relative velocities/accelerations on the segment geodesic. The normalized
    // coordinate maps to a fraction f along the segment, with df/dt = norm_velocity /
    // segment_length and d2f/dt2 = norm_acceleration / segment_length. Translational components are
    // obtained by finite-differencing the geodesic; rotational components vary linearly with f, so
    // their slope is the (constant) wrapped angular difference and their curvature is zero.
    const auto& cumulative = nonstandard_cumulative_.at(nonstandard_idx);
    const auto [segment, fraction] = locateSegment(cumulative, q_norm(norm_col));
    const double segment_length = cumulative[segment + 1] - cumulative[segment];
    const double df_dt = dq_norm(norm_col) / segment_length;
    const double d2f_dt2 = ddq_norm(norm_col) / segment_length;
    const auto q_off = static_cast<Eigen::Index>(model.idx_qs[model.getJointId(joint_name)]);

    const auto& q_start = full_waypoints_.at(segment);
    const auto& q_end = full_waypoints_.at(segment + 1);

    // Indices of the rotation representation within the joint's full-config block.
    const Eigen::Index cos_idx = q_off + (joint_info.type == JointType::PLANAR ? 2 : 0);
    const double theta_start = std::atan2(q_start(cos_idx + 1), q_start(cos_idx));
    const double theta_end = std::atan2(q_end(cos_idx + 1), q_end(cos_idx));
    const double dtheta_df = wrapAngle(theta_end - theta_start);

    if (joint_info.type == JointType::PLANAR) {
      const Eigen::VectorXd interp_minus =
          scene_->interpolate(q_start, q_end, fraction - kFractionEpsilon);
      const Eigen::VectorXd interp_center = scene_->interpolate(q_start, q_end, fraction);
      const Eigen::VectorXd interp_plus =
          scene_->interpolate(q_start, q_end, fraction + kFractionEpsilon);
      for (Eigen::Index t = 0; t < 2; ++t) {  // Translational x and y.
        const double minus = interp_minus(q_off + t);
        const double center = interp_center(q_off + t);
        const double plus = interp_plus(q_off + t);
        const double dq_df = (plus - minus) / (2.0 * kFractionEpsilon);
        const double d2q_df2 =
            (plus - 2.0 * center + minus) / (kFractionEpsilon * kFractionEpsilon);
        velocities(vel_col + static_cast<size_t>(t)) = dq_df * df_dt;
        accelerations(vel_col + static_cast<size_t>(t)) = d2q_df2 * df_dt * df_dt + dq_df * d2f_dt2;
      }
      velocities(vel_col + 2) = dtheta_df * df_dt;
      accelerations(vel_col + 2) = dtheta_df * d2f_dt2;
    } else {  // Continuous: a single rotational component.
      velocities(vel_col) = dtheta_df * df_dt;
      accelerations(vel_col) = dtheta_df * d2f_dt2;
    }

    norm_col += 1;
    vel_col += joint_info.num_velocity_dofs;
    ++nonstandard_idx;
  }
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

  // Build the position vectors first, since for non-standard joints this also computes the
  // normalized velocity/acceleration limits used by the constraints below.
  auto maybe_path_pos_vecs = getPathPositionVectors(path);
  if (!maybe_path_pos_vecs) {
    return tl::make_unexpected("Failed to extract position vectors from path: " +
                               maybe_path_pos_vecs.error());
  }
  auto path_pos_vecs = maybe_path_pos_vecs.value();

  // Create scaled velocity and acceleration constraints. With non-standard joints the limits are
  // expressed in the normalized representation (planar/continuous/floating joints collapse to a
  // single coordinate with a unit velocity limit and a derived acceleration limit).
  const auto& vel_lower = has_nonstandard_joints_ ? norm_vel_lower_limits_ : vel_lower_limits_;
  const auto& vel_upper = has_nonstandard_joints_ ? norm_vel_upper_limits_ : vel_upper_limits_;
  const auto& acc_lower = has_nonstandard_joints_ ? norm_acc_lower_limits_ : acc_lower_limits_;
  const auto& acc_upper = has_nonstandard_joints_ ? norm_acc_upper_limits_ : acc_upper_limits_;
  toppra::LinearConstraintPtr vel_constraint, acc_constraint;
  vel_constraint = std::make_shared<toppra::constraint::LinearJointVelocity>(
      vel_lower * velocity_scale, vel_upper * velocity_scale);
  acc_constraint = std::make_shared<toppra::constraint::LinearJointAcceleration>(
      acc_lower * acceleration_scale, acc_upper * acceleration_scale);
  acc_constraint->discretizationType(toppra::DiscretizationType::Interpolation);
  toppra::LinearConstraintPtrs constraints = {vel_constraint, acc_constraint};

  // Parse the spline fitting mode and set the options accordingly.
  // The basic rules are:
  // - If Hermite mode is enabled, we don't need to iterate or check collisions.
  // - If cubic mode is enabled, we just do one iteration with collision checking.
  // - If adaptive mode is enabled, we do need to iterate by checking collisisions.
  int max_collision_iterations = 0;
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
      Eigen::VectorXd q_full;
      if (has_nonstandard_joints_) {
        q_full = scene_->toFullJointPositions(group_name_, normalizedToGroupPositions(q));
      } else {
        const auto maybe_q_expanded = expandContinuousJointPositions(*scene_, group_name_, q);
        if (!maybe_q_expanded) {
          return tl::make_unexpected("Failed to collision check geometric path: " +
                                     maybe_q_expanded.error());
        }
        q_full = scene_->toFullJointPositions(group_name_, maybe_q_expanded.value());
      }

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
  const auto positions = const_acc->eval(times_vec, 0);
  const auto velocities = const_acc->eval(times_vec, 1);
  const auto accelerations = const_acc->eval(times_vec, 2);
  for (size_t i = 0; i < positions.size(); ++i) {
    if (has_nonstandard_joints_) {
      // Reconstruct the expanded positions and the collapsed velocities/accelerations from the
      // normalized solution.
      traj.positions.push_back(normalizedToGroupPositions(positions.at(i)));
      Eigen::VectorXd vel, acc;
      normalizedToVelocitiesAndAccelerations(positions.at(i), velocities.at(i), accelerations.at(i),
                                             vel, acc);
      traj.velocities.push_back(std::move(vel));
      traj.accelerations.push_back(std::move(acc));
    } else {
      const auto maybe_expanded_pos =
          expandContinuousJointPositions(*scene_, group_name_, positions.at(i));
      if (!maybe_expanded_pos) {
        return tl::make_unexpected("Failed to compute path parameterization: " +
                                   maybe_expanded_pos.error());
      }
      traj.positions.push_back(maybe_expanded_pos.value());
      traj.velocities.push_back(velocities.at(i));
      traj.accelerations.push_back(accelerations.at(i));
    }
  }

  return traj;
}

}  // namespace roboplan
