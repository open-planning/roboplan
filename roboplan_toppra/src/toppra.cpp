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

  // Determine the dimension of the normalized representation and which joints are non-standard, and
  // precompute each joint's placement across every representation. Standard joints contribute their
  // velocity DOFs directly; non-standard (Lie-group) joints each contribute a single normalized
  // arc-length coordinate. The collapsed representation replaces each non-standard joint's rotation
  // (cos, sin) with a single angle, so it occupies one fewer DOF than the expanded representation.
  const auto& model = scene_->getModel();
  size_t normalized_col = 0, collapsed_col = 0, velocity_col = 0, expanded_col = 0;
  int nonstandard_count = 0;
  for (const auto& joint_name : joint_group_info.joint_names) {
    const auto maybe_joint_info = scene_->getJointInfo(joint_name);
    if (!maybe_joint_info) {
      throw std::runtime_error("Failed to instantiate TOPP-RA: " + maybe_joint_info.error());
    }
    const auto& joint_info = maybe_joint_info.value();
    if (joint_info.mimic_info) {
      continue;  // Mimic joints occupy no degrees of freedom.
    }

    const bool is_nonstandard = isNonstandardJoint(joint_info.type);
    const auto joint_id = model.getJointId(joint_name);
    JointLayout layout;
    layout.joint_name = joint_name;
    layout.type = joint_info.type;
    layout.is_nonstandard = is_nonstandard;
    layout.num_position_dofs = joint_info.num_position_dofs;
    layout.num_velocity_dofs = joint_info.num_velocity_dofs;
    layout.normalized_col = normalized_col;
    layout.collapsed_col = collapsed_col;
    layout.velocity_col = velocity_col;
    layout.expanded_col = expanded_col;
    layout.q_offset = static_cast<Eigen::Index>(model.idx_qs[joint_id]);
    layout.v_offset = static_cast<Eigen::Index>(model.idx_vs[joint_id]);
    layout.nonstandard_idx = is_nonstandard ? nonstandard_count : -1;
    joint_layouts_.push_back(layout);

    if (is_nonstandard) {
      has_nonstandard_joints_ = true;
      ++nonstandard_count;
      norm_dim_ += 1;
      normalized_col += 1;
      collapsed_col += joint_info.num_position_dofs - 1;
    } else {
      norm_dim_ += joint_info.num_velocity_dofs;
      normalized_col += joint_info.num_velocity_dofs;
      collapsed_col += joint_info.num_position_dofs;
    }
    velocity_col += joint_info.num_velocity_dofs;
    expanded_col += joint_info.num_position_dofs;
  }
}

bool PathParameterizerTOPPRA::isNonstandardJoint(JointType type) {
  return type == JointType::CONTINUOUS || type == JointType::PLANAR || type == JointType::FLOATING;
}

std::pair<size_t, double>
PathParameterizerTOPPRA::locateSegment(const std::vector<double>& cumulative_coordinates,
                                       double value) {
  const size_t num_segments = cumulative_coordinates.size() - 1;
  // Find the segment [k, k+1) such that cumulative_coordinates[k] <= value <
  // cumulative_coordinates[k+1].
  const auto it =
      std::upper_bound(cumulative_coordinates.begin(), cumulative_coordinates.end(), value);
  size_t seg = (it == cumulative_coordinates.begin())
                   ? 0
                   : static_cast<size_t>(std::distance(cumulative_coordinates.begin(), it)) - 1;
  seg = std::min(seg, num_segments - 1);
  const double length = cumulative_coordinates.at(seg + 1) - cumulative_coordinates.at(seg);
  const double fraction =
      (length > 0.0) ? std::clamp((value - cumulative_coordinates.at(seg)) / length, 0.0, 1.0)
                     : 0.0;
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

  nonstandard_joint_cumulative_coordinates_.clear();
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

  for (const auto& layout : joint_layouts_) {
    if (!layout.is_nonstandard) {
      // Copy the joint's collapsed positions and limits straight through.
      for (size_t d = 0; d < layout.num_velocity_dofs; ++d) {
        for (size_t k = 0; k < num_waypoints; ++k) {
          path_pos_vecs.at(k)(layout.normalized_col + d) =
              collapsed.at(k)(layout.collapsed_col + d);
        }
        norm_vel_lower_limits_(layout.normalized_col + d) =
            vel_lower_limits_(layout.velocity_col + d);
        norm_vel_upper_limits_(layout.normalized_col + d) =
            vel_upper_limits_(layout.velocity_col + d);
        norm_acc_lower_limits_(layout.normalized_col + d) =
            acc_lower_limits_(layout.velocity_col + d);
        norm_acc_upper_limits_(layout.normalized_col + d) =
            acc_upper_limits_(layout.velocity_col + d);
      }
      continue;
    }

    // Identify the joint's rotational and full position-index blocks, and the linear/angular
    // limits. Splitting the geodesic this way recovers the true body-frame linear distance (the arc
    // length), since the SE(2)/SE(3) log decomposes orthogonally: L^2 = d_linear^2 + d_angular^2.
    // The world chord between waypoints would underestimate the arc on curving (translating +
    // rotating) segments, so we use d_linear = sqrt(L^2 - d_angular^2) instead.
    const auto joint_info = scene_->getJointInfo(layout.joint_name).value();
    const Eigen::Index q_off = layout.q_offset;
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
    std::vector<double> cumulative_coordinates(num_waypoints, 0.0);
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
      cumulative_coordinates[k + 1] = cumulative_coordinates[k] + segment_length;

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
      path_pos_vecs.at(k)(layout.normalized_col) = cumulative_coordinates[k];
    }
    norm_vel_lower_limits_(layout.normalized_col) = -1.0;
    norm_vel_upper_limits_(layout.normalized_col) = 1.0;
    norm_acc_lower_limits_(layout.normalized_col) = -accel_limit;
    norm_acc_upper_limits_(layout.normalized_col) = accel_limit;
    nonstandard_joint_cumulative_coordinates_.push_back(std::move(cumulative_coordinates));
  }

  return path_pos_vecs;
}

Eigen::VectorXd
PathParameterizerTOPPRA::normalizedToGroupPositions(const Eigen::VectorXd& q_norm) const {
  Eigen::VectorXd group_pos(q_indices_.size());

  for (const auto& layout : joint_layouts_) {
    const auto expanded_col = static_cast<Eigen::Index>(layout.expanded_col);
    if (!layout.is_nonstandard) {
      for (size_t dof = 0; dof < layout.num_velocity_dofs; ++dof) {
        group_pos(expanded_col + static_cast<Eigen::Index>(dof)) =
            q_norm(layout.normalized_col + dof);
      }
      continue;
    }

    const auto& cumulative_coordinates =
        nonstandard_joint_cumulative_coordinates_.at(layout.nonstandard_idx);
    const auto [segment, fraction] =
        locateSegment(cumulative_coordinates, q_norm(layout.normalized_col));
    const Eigen::VectorXd interpolated =
        scene_->interpolate(full_waypoints_.at(segment), full_waypoints_.at(segment + 1), fraction);
    for (size_t dof = 0; dof < layout.num_position_dofs; ++dof) {
      group_pos(expanded_col + static_cast<Eigen::Index>(dof)) =
          interpolated(layout.q_offset + static_cast<Eigen::Index>(dof));
    }
  }

  return group_pos;
}

void PathParameterizerTOPPRA::normalizedToVelocitiesAndAccelerations(
    const Eigen::VectorXd& q_norm, const Eigen::VectorXd& dq_norm, const Eigen::VectorXd& ddq_norm,
    Eigen::VectorXd& velocities, Eigen::VectorXd& accelerations) const {
  velocities = Eigen::VectorXd::Zero(vel_lower_limits_.size());
  accelerations = Eigen::VectorXd::Zero(vel_lower_limits_.size());

  for (const auto& layout : joint_layouts_) {
    if (!layout.is_nonstandard) {
      for (size_t dof = 0; dof < layout.num_velocity_dofs; ++dof) {
        velocities(layout.velocity_col + dof) = dq_norm(layout.normalized_col + dof);
        accelerations(layout.velocity_col + dof) = ddq_norm(layout.normalized_col + dof);
      }
      continue;
    }

    // The non-standard joint follows the scene geodesic, whose body-frame tangent (the Lie group
    // log) is constant along the segment. The normalized coordinate maps to a fraction f with
    // df/dt = norm_velocity / segment_length and d2f/dt2 = norm_acceleration / segment_length, so
    // by the chain rule the body velocity is twist * df/dt and the body acceleration is twist *
    // d2f/dt2 (the geodesic is "straight" in the Lie algebra, so the df/dt^2 curvature term
    // vanishes).
    const auto& cumulative_coordinates =
        nonstandard_joint_cumulative_coordinates_.at(layout.nonstandard_idx);
    const size_t segment =
        locateSegment(cumulative_coordinates, q_norm(layout.normalized_col)).first;
    const double segment_length =
        cumulative_coordinates[segment + 1] - cumulative_coordinates[segment];
    const double df_dt = dq_norm(layout.normalized_col) / segment_length;
    const double d2f_dt2 = ddq_norm(layout.normalized_col) / segment_length;

    const Eigen::VectorXd twist =
        scene_->difference(full_waypoints_.at(segment), full_waypoints_.at(segment + 1));
    for (size_t dof = 0; dof < layout.num_velocity_dofs; ++dof) {
      const double tangent = twist(layout.v_offset + static_cast<Eigen::Index>(dof));
      velocities(layout.velocity_col + dof) = tangent * df_dt;
      accelerations(layout.velocity_col + dof) = tangent * d2f_dt2;
    }
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
