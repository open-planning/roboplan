#include <roboplan_cartesian_planning/cartesian_path_planner.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>

#include <roboplan_oink/constraints/position_limit.hpp>
#include <roboplan_oink/constraints/velocity_limit.hpp>
#include <roboplan_oink/optimal_ik.hpp>
#include <roboplan_oink/tasks/configuration.hpp>
#include <roboplan_oink/tasks/frame.hpp>
#include <roboplan_toppra/toppra.hpp>

namespace roboplan {

namespace {

/// @brief Maximum number of differential-IK steps used to drive the robot onto the
/// first waypoint before the timed trace begins.
constexpr int kMaxConvergenceIters = 500;

/// @brief Hard cap on the number of waypoints handed to TOPP-RA. Normally the full dense diff-IK
/// trace is used as-is (the LinearBlend geometry is density-robust, so decimating only discards
/// path detail); this cap only kicks in for pathologically long paths, decimating to bound the
/// time-parameterization problem size (and hence planning time).
constexpr size_t kMaxToppraWaypoints = 10000;

/// @brief Resamples a dense sequence of joint positions to `count` waypoints spaced
/// uniformly in joint-space arc length (endpoints preserved).
/// @details TOPP-RA parameterizes its cubic spline by waypoint index, so evenly spaced
/// knots are essential: unevenly spaced waypoints (e.g. clustered where the tracker
/// throttled at corners) leave large gaps that the spline overshoots, deviating from
/// the path. Linear interpolation between the closely spaced dense samples is accurate.
std::vector<Eigen::VectorXd> resampleUniform(const std::vector<Eigen::VectorXd>& positions,
                                             size_t count) {
  const size_t n = positions.size();
  if (n <= 2 || count < 2) {
    return positions;
  }

  // Cumulative joint-space arc length along the dense path.
  std::vector<double> cumulative(n, 0.0);
  for (size_t i = 1; i < n; ++i) {
    cumulative.at(i) = cumulative.at(i - 1) + (positions.at(i) - positions.at(i - 1)).norm();
  }
  const double total_length = cumulative.back();
  if (total_length <= 0.0) {
    return {positions.front(), positions.back()};
  }

  const size_t num_points = std::min(count, n);
  std::vector<Eigen::VectorXd> result;
  result.reserve(num_points);
  size_t segment = 0;
  for (size_t k = 0; k < num_points; ++k) {
    const double target =
        total_length * static_cast<double>(k) / static_cast<double>(num_points - 1);
    while (segment + 1 < n && cumulative.at(segment + 1) < target) {
      ++segment;
    }
    if (segment + 1 >= n) {
      result.push_back(positions.back());
      continue;
    }
    const double span = cumulative.at(segment + 1) - cumulative.at(segment);
    const double fraction = span > 1e-12 ? (target - cumulative.at(segment)) / span : 0.0;
    result.push_back(
        (positions.at(segment) + fraction * (positions.at(segment + 1) - positions.at(segment)))
            .eval());
  }
  // Pin the exact endpoints.
  result.front() = positions.front();
  result.back() = positions.back();
  return result;
}

/// @brief Computes the position (meters) and orientation (radians) error between two
/// SE(3) transforms expressed in the same frame.
void poseError(const Eigen::Matrix4d& a, const Eigen::Matrix4d& b, double& position_error,
               double& orientation_error) {
  position_error = (a.block<3, 1>(0, 3) - b.block<3, 1>(0, 3)).norm();
  const Eigen::Matrix3d relative_rotation = a.block<3, 3>(0, 0).transpose() * b.block<3, 3>(0, 0);
  orientation_error = Eigen::AngleAxisd(relative_rotation).angle();
}

}  // namespace

Eigen::Matrix4d CartesianPathPlanner::Reference::eval(double s) const {
  if (waypoints.empty()) {
    return Eigen::Matrix4d::Identity();
  }
  if (waypoints.size() == 1 || total_time <= 0.0) {
    return waypoints.back();
  }

  s = std::clamp(s, 0.0, total_time);

  // Find the segment [i, i+1] containing reference time s.
  size_t i = 0;
  while (i + 2 < cumulative_times.size() && cumulative_times.at(i + 1) < s) {
    ++i;
  }
  const double t0 = cumulative_times.at(i);
  const double t1 = cumulative_times.at(i + 1);
  const double segment_duration = t1 - t0;
  const double fraction = segment_duration > 0.0 ? (s - t0) / segment_duration : 0.0;

  const Eigen::Matrix4d& start = waypoints.at(i);
  const Eigen::Matrix4d& end = waypoints.at(i + 1);

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

CartesianPathPlanner::Reference
CartesianPathPlanner::buildReference(const std::vector<Eigen::Matrix4d>& waypoints,
                                     double linear_speed, double angular_speed) const {
  Reference reference;
  reference.waypoints = waypoints;
  reference.cumulative_times.assign(waypoints.size(), 0.0);
  for (size_t i = 1; i < waypoints.size(); ++i) {
    const double linear_distance =
        (waypoints.at(i).block<3, 1>(0, 3) - waypoints.at(i - 1).block<3, 1>(0, 3)).norm();
    const Eigen::Matrix3d relative_rotation =
        waypoints.at(i - 1).block<3, 3>(0, 0).transpose() * waypoints.at(i).block<3, 3>(0, 0);
    const double angular_distance = Eigen::AngleAxisd(relative_rotation).angle();

    const double linear_time = linear_speed > 0.0 ? linear_distance / linear_speed : 0.0;
    const double angular_time = angular_speed > 0.0 ? angular_distance / angular_speed : 0.0;
    reference.cumulative_times.at(i) =
        reference.cumulative_times.at(i - 1) + std::max(linear_time, angular_time);
  }
  reference.total_time = reference.cumulative_times.back();
  return reference;
}

CartesianPathPlanner::CartesianPathPlanner(const std::shared_ptr<Scene> scene,
                                           const CartesianPlannerOptions& options)
    : scene_{scene}, options_{options} {
  const auto maybe_joint_group_info = scene_->getJointGroupInfo(options_.group_name);
  if (!maybe_joint_group_info) {
    throw std::runtime_error("Could not initialize Cartesian path planner: " +
                             maybe_joint_group_info.error());
  }
  joint_group_info_ = maybe_joint_group_info.value();
}

tl::expected<CartesianPlanResult, std::string>
CartesianPathPlanner::plan(const CartesianPath& path, const JointConfiguration& q_start) {
  // Validate options.
  if (options_.dt <= 0.0) {
    return tl::make_unexpected("dt must be strictly positive.");
  }
  if (options_.linear_speed <= 0.0 || options_.angular_speed <= 0.0) {
    return tl::make_unexpected("linear_speed and angular_speed must be strictly positive.");
  }
  if (options_.max_position_error <= 0.0 || options_.max_orientation_error <= 0.0) {
    return tl::make_unexpected(
        "max_position_error and max_orientation_error must be strictly positive.");
  }
  if (options_.velocity_scale <= 0.0 || options_.velocity_scale > 1.0) {
    return tl::make_unexpected("velocity_scale must be in the interval (0, 1].");
  }

  // Validate the path (v1 supports a single end-effector frame).
  if (path.base_frames.size() != 1 || path.tip_frames.size() != 1 || path.tforms.size() != 1) {
    return tl::make_unexpected(
        "CartesianPathPlanner v1 supports exactly one end-effector frame (the path must contain "
        "exactly one base frame, tip frame, and transform list).");
  }
  if (path.tforms.at(0).size() < 1) {
    return tl::make_unexpected("The Cartesian path must contain at least one waypoint.");
  }

  // Validate the seed configuration.
  const auto& model = scene_->getModel();
  if (q_start.positions.size() != model.nq) {
    return tl::make_unexpected("q_start must be a full model configuration of size model.nq (" +
                               std::to_string(model.nq) + "), got " +
                               std::to_string(q_start.positions.size()) + ".");
  }

  switch (options_.speed_mode) {
  case CartesianSpeedMode::ConstantCartesianSpeed:
    return planConstantSpeed(path, q_start.positions);
  case CartesianSpeedMode::TimeOptimalToppra:
    return planToppra(path, q_start.positions);
  }
  return tl::make_unexpected("Unknown CartesianSpeedMode.");
}

tl::expected<CartesianPathPlanner::TrackResult, std::string>
CartesianPathPlanner::trackReference(const CartesianPath& path, const Eigen::VectorXd& q_start_full,
                                     double linear_speed, double angular_speed) {
  const auto& model = scene_->getModel();
  const std::string& base_frame = path.base_frames.at(0);
  const std::string& tip_frame = path.tip_frames.at(0);
  const Reference reference = buildReference(path.tforms.at(0), linear_speed, angular_speed);

  // The Oink FrameTask expects its target expressed in the world frame, while the
  // CartesianPath waypoints are given relative to `base_frame`. The base frame is
  // fixed relative to the world for a fixed-base robot, so compute world_T_base once
  // and use it to map each base-relative reference pose into the world frame.
  Eigen::Matrix4d world_T_base;
  try {
    world_T_base = scene_->forwardKinematics(q_start_full, base_frame);
  } catch (const std::exception& e) {
    return tl::make_unexpected(std::string("Could not resolve base frame '") + base_frame +
                               "': " + e.what());
  }
  const auto to_world = [&world_T_base](const Eigen::Matrix4d& base_T_tip) -> Eigen::Matrix4d {
    return world_T_base * base_T_tip;
  };

  // Build the Oink solver and its tasks/constraints. Construction can throw if a
  // frame or group cannot be resolved, so convert any exception to an error.
  try {
    Oink oink(*scene_, options_.group_name);
    const int num_variables = oink.num_variables;

    // Priority-1 frame task: track the moving reference pose.
    CartesianConfiguration target;
    target.base_frame = "";  // FrameTask interprets the target tform in the world frame.
    target.tip_frame = tip_frame;
    target.tform = to_world(reference.eval(0.0));
    FrameTaskOptions frame_options;
    frame_options.position_cost = options_.position_cost;
    frame_options.orientation_cost = options_.orientation_cost;
    frame_options.task_gain = options_.task_gain;
    frame_options.lm_damping = options_.lm_damping;
    frame_options.priority = 1;
    auto frame_task = std::make_shared<FrameTask>(oink, *scene_, target, frame_options);

    // Priority-2 configuration task: gently regularize redundant joints toward the
    // seed, using only the nullspace the frame task leaves free.
    const Eigen::VectorXd joint_weights =
        Eigen::VectorXd::Constant(num_variables, options_.config_task_weight);
    ConfigurationTaskOptions config_options;
    config_options.priority = 2;
    const Eigen::VectorXd target_q = q_start_full(oink.q_indices);
    auto config_task =
        std::make_shared<ConfigurationTask>(oink, target_q, joint_weights, config_options);

    std::vector<std::shared_ptr<Task>> tasks = {frame_task, config_task};

    // Velocity-limit constraint: bounds each differential-IK step to dt * v_max.
    const auto maybe_velocity_limits = scene_->getVelocityLimitVectors(options_.group_name);
    if (!maybe_velocity_limits) {
      return tl::make_unexpected("Could not get joint velocity limits: " +
                                 maybe_velocity_limits.error());
    }
    Eigen::VectorXd v_max = maybe_velocity_limits->second.cwiseAbs() * options_.velocity_scale;
    if (v_max.size() != num_variables) {
      return tl::make_unexpected("Velocity limit vector size (" + std::to_string(v_max.size()) +
                                 ") does not match the group velocity DOF count (" +
                                 std::to_string(num_variables) + ").");
    }

    // Enforce both joint velocity and joint position limits inside the QP:
    //   - VelocityLimit bounds each step to dt * v_max (so |delta_q|/dt <= v_max).
    //   - PositionLimit restricts each step so the integrated configuration stays
    //     within the joint position limits.
    std::vector<std::shared_ptr<Constraints>> constraints = {
        std::make_shared<VelocityLimit>(oink, options_.dt, v_max),
        std::make_shared<PositionLimit>(oink, options_.position_limit_gain)};

    // Per-step scratch.
    Eigen::VectorXd delta_q(num_variables);
    Eigen::VectorXd delta_q_full(model.nv);
    Eigen::VectorXd q = q_start_full;

    // Runs one differential-IK step toward `world_tform` (a world-frame target pose)
    // from the committed configuration `q`, writing the candidate configuration and its
    // world-frame FK pose error. Does not commit.
    auto solve_step = [&](const Eigen::Matrix4d& world_tform, Eigen::VectorXd& q_candidate,
                          double& position_error,
                          double& orientation_error) -> tl::expected<void, std::string> {
      // Refresh the scene state and FK cache to the committed configuration so the
      // Oink tasks read the correct current pose.
      scene_->setJointPositions(q);
      scene_->forwardKinematics(q, tip_frame);
      frame_task->setTargetFrameTransform(world_tform);
      delta_q.setZero();
      const auto result =
          oink.solveIk(*scene_, tasks, constraints, delta_q, options_.regularization);
      if (!result) {
        return tl::make_unexpected(result.error());
      }
      delta_q_full.setZero();
      delta_q_full(oink.v_indices) = delta_q;
      q_candidate = scene_->integrate(q, delta_q_full);
      const Eigen::Matrix4d fk = scene_->forwardKinematics(q_candidate, tip_frame);
      poseError(fk, world_tform, position_error, orientation_error);
      return {};
    };

    // Convergence phase: drive the robot onto the first waypoint within tolerance.
    const Eigen::Matrix4d start_pose = to_world(reference.eval(0.0));
    double position_error = 0.0;
    double orientation_error = 0.0;
    bool converged = false;
    for (int i = 0; i < kMaxConvergenceIters; ++i) {
      Eigen::VectorXd q_candidate;
      const auto step = solve_step(start_pose, q_candidate, position_error, orientation_error);
      if (!step) {
        return tl::make_unexpected("Oink solve failed while converging to the first waypoint: " +
                                   step.error());
      }
      q = q_candidate;

      if (position_error <= options_.max_position_error &&
          orientation_error <= options_.max_orientation_error) {
        converged = true;
        break;
      }
    }
    if (!converged) {
      return tl::make_unexpected(
          "Could not converge to the first waypoint within tolerance (position error " +
          std::to_string(position_error) + " m, orientation error " +
          std::to_string(orientation_error) +
          " rad). Ensure q_start places the tool at or near the first waypoint.");
    }

    // Initialize the trace at the converged start.
    TrackResult result;
    result.group_positions.push_back(q(oink.q_indices).eval());
    result.group_velocities.push_back(Eigen::VectorXd::Zero(num_variables));
    result.times.push_back(0.0);

    Eigen::Vector3d previous_position = scene_->forwardKinematics(q, tip_frame).block<3, 1>(0, 3);

    // Timed servo loop with feedrate throttling for tolerance enforcement.
    double s = 0.0;
    int total_steps = 0;
    int throttled_steps = 0;
    const double eps = 1e-9;
    const int hard_cap = static_cast<int>(std::ceil(reference.total_time / options_.dt)) *
                             (options_.max_attempts_per_step + 2) +
                         1000;

    while (s < reference.total_time - eps) {
      bool committed = false;
      double alpha = 1.0;
      Eigen::VectorXd q_candidate;
      double committed_s = s;
      for (int attempt = 0; attempt < options_.max_attempts_per_step; ++attempt) {
        const double s_try = std::min(s + alpha * options_.dt, reference.total_time);
        const Eigen::Matrix4d tform = to_world(reference.eval(s_try));
        const auto step = solve_step(tform, q_candidate, position_error, orientation_error);
        if (!step) {
          return tl::make_unexpected("Oink solve failed at reference time " + std::to_string(s) +
                                     "s: " + step.error());
        }
        if (position_error <= options_.max_position_error &&
            orientation_error <= options_.max_orientation_error) {
          committed = true;
          committed_s = s_try;
          if (alpha < 1.0 - eps) {
            ++throttled_steps;
          }
          break;
        }
        alpha *= 0.5;
      }

      if (!committed) {
        std::string hint = " The path may be unreachable or pass through a singularity.";
        return tl::make_unexpected(
            "Cartesian planner stalled at reference time " + std::to_string(s) +
            "s: cannot stay within tolerance (position error " + std::to_string(position_error) +
            " m, orientation error " + std::to_string(orientation_error) + " rad)." + hint);
      }

      // Commit the step.
      s = committed_s;
      q = q_candidate;

      // The VelocityLimit and PositionLimit constraints keep each step within the
      // joint limits inside the QP; verify the committed step actually does, to guard
      // against solver constraint relaxation or numerical drift. A small relative
      // tolerance absorbs the QP's constraint-satisfaction tolerance.
      constexpr double kLimitRelTolerance = 1e-2;
      const Eigen::ArrayXd velocity_bound =
          options_.dt * v_max.array() * (1.0 + kLimitRelTolerance) + 1e-9;
      if ((delta_q.array().abs() > velocity_bound).any()) {
        return tl::make_unexpected(
            "Joint velocity limit exceeded at reference time " + std::to_string(s) +
            "s (peak |q_dot| = " +
            std::to_string((delta_q.array().abs() / options_.dt).maxCoeff()) + " vs. limit " +
            std::to_string(v_max.maxCoeff()) + ").");
      }
      if (!scene_->isValidConfiguration(q)) {
        return tl::make_unexpected("Joint position limit exceeded at reference time " +
                                   std::to_string(s) + "s.");
      }

      ++total_steps;
      result.times.push_back(total_steps * options_.dt);
      result.group_positions.push_back(q(oink.q_indices).eval());
      result.group_velocities.push_back((delta_q / options_.dt).eval());

      const Eigen::Vector3d current_position =
          scene_->forwardKinematics(q, tip_frame).block<3, 1>(0, 3);
      result.achieved_path_length += (current_position - previous_position).norm();
      previous_position = current_position;

      if (total_steps > hard_cap) {
        return tl::make_unexpected(
            "Cartesian planner exceeded the maximum number of control steps (" +
            std::to_string(hard_cap) + "). The path may be infeasible at the requested tolerance.");
      }
    }

    result.feedrate_efficiency =
        total_steps > 0
            ? static_cast<double>(total_steps - throttled_steps) / static_cast<double>(total_steps)
            : 1.0;
    return result;
  } catch (const std::exception& e) {
    return tl::make_unexpected(std::string("Cartesian planner setup failed: ") + e.what());
  }
}

void CartesianPathPlanner::computePeakLimitRatios(const JointTrajectory& trajectory,
                                                  double& velocity_ratio,
                                                  double& acceleration_ratio) const {
  velocity_ratio = 0.0;
  acceleration_ratio = 0.0;

  const auto velocity_limits = scene_->getVelocityLimitVectors(options_.group_name);
  const auto acceleration_limits = scene_->getAccelerationLimitVectors(options_.group_name);

  const auto peak_ratio = [](const std::vector<Eigen::VectorXd>& values,
                             const Eigen::VectorXd& limit) -> double {
    double ratio = 0.0;
    for (const auto& value : values) {
      if (value.size() != limit.size()) {
        continue;
      }
      for (Eigen::Index i = 0; i < value.size(); ++i) {
        // Skip joints with negligible limits to avoid divide-by-zero.
        if (std::abs(limit(i)) > 1e-9) {
          ratio = std::max(ratio, std::abs(value(i)) / std::abs(limit(i)));
        }
      }
    }
    return ratio;
  };

  if (velocity_limits) {
    velocity_ratio = peak_ratio(trajectory.velocities, velocity_limits->second.cwiseAbs());
  }
  if (acceleration_limits) {
    acceleration_ratio =
        peak_ratio(trajectory.accelerations, acceleration_limits->second.cwiseAbs());
  }
}

tl::expected<CartesianPlanResult, std::string>
CartesianPathPlanner::planConstantSpeed(const CartesianPath& path,
                                        const Eigen::VectorXd& q_start_full) {
  auto tracked = trackReference(path, q_start_full, options_.linear_speed, options_.angular_speed);
  if (!tracked) {
    return tl::make_unexpected(tracked.error());
  }

  CartesianPlanResult plan_result;
  JointTrajectory& trajectory = plan_result.trajectory;
  trajectory.joint_names = joint_group_info_.joint_names;
  trajectory.times = std::move(tracked->times);
  trajectory.positions = std::move(tracked->group_positions);
  trajectory.velocities = std::move(tracked->group_velocities);

  // Fill accelerations by backward finite difference of the velocities. The
  // ConstantCartesianSpeed mode is velocity-level, so these can exceed the joint
  // acceleration limits; the peak ratios below make that explicit.
  const int num_variables =
      trajectory.velocities.empty() ? 0 : static_cast<int>(trajectory.velocities.front().size());
  trajectory.accelerations.assign(trajectory.velocities.size(),
                                  Eigen::VectorXd::Zero(num_variables));
  for (size_t i = 1; i < trajectory.velocities.size(); ++i) {
    trajectory.accelerations.at(i) =
        (trajectory.velocities.at(i) - trajectory.velocities.at(i - 1)) / options_.dt;
  }

  plan_result.achieved_path_length = tracked->achieved_path_length;
  plan_result.feedrate_efficiency = tracked->feedrate_efficiency;
  computePeakLimitRatios(trajectory, plan_result.peak_velocity_ratio,
                         plan_result.peak_acceleration_ratio);
  return plan_result;
}

tl::expected<CartesianPlanResult, std::string>
CartesianPathPlanner::planToppra(const CartesianPath& path, const Eigen::VectorXd& q_start_full) {
  // Resolve a dense geometric joint path that hugs the Cartesian path. The resolution
  // speed only sets sampling density and tracking tightness here (TOPP-RA assigns the
  // final timing), so use a deliberately low speed that essentially any robot can follow
  // without lagging: a faster resolution speed makes the robot trail the reference by up
  // to the tolerance band, and TOPP-RA would then faithfully reproduce that lag.
  constexpr double kResolutionLinearSpeed = 0.05;   // m/s
  constexpr double kResolutionAngularSpeed = 0.25;  // rad/s

  // Geometric resolution is velocity-level only; TOPP-RA enforces the acceleration limits
  // in the re-timing stage.
  auto tracked =
      trackReference(path, q_start_full, kResolutionLinearSpeed, kResolutionAngularSpeed);
  if (!tracked) {
    return tl::make_unexpected(tracked.error());
  }

  // Hand the dense diff-IK trace straight to TOPP-RA: the LinearBlend geometry is density-robust
  // (straight segments have zero curvature), so decimating only throws away path detail. Decimate
  // solely as a safety cap on the problem size for very long paths.
  JointPath joint_path;
  joint_path.joint_names = joint_group_info_.joint_names;
  joint_path.positions = tracked->group_positions.size() > kMaxToppraWaypoints
                             ? resampleUniform(tracked->group_positions, kMaxToppraWaypoints)
                             : tracked->group_positions;
  if (joint_path.positions.size() < 2) {
    return tl::make_unexpected(
        "Resolved joint path has fewer than 2 waypoints; the Cartesian path may be degenerate "
        "or too short to time-parameterize.");
  }

  // Time-parameterize the joint path with TOPP-RA so the result respects joint velocity and
  // acceleration limits. Use the LinearBlend geometry (straight segments with circular corner
  // blends): its straight segments have zero curvature, so the dense, slightly jittery diff-IK
  // trace no longer inflates the acceleration constraint and crawls the trajectory the way an
  // interpolating cubic spline would. Corners are rounded within toppra_blend_deviation.
  try {
    PathParameterizerTOPPRA parameterizer(scene_, options_.group_name);
    auto maybe_trajectory = parameterizer.generate(
        joint_path, options_.dt, SplineFittingMode::LinearBlend, options_.velocity_scale,
        options_.acceleration_scale, /*max_adaptive_iterations=*/10,
        /*max_adaptive_step_size=*/0.05, options_.toppra_blend_deviation);
    if (!maybe_trajectory) {
      return tl::make_unexpected("TOPP-RA time parameterization failed: " +
                                 maybe_trajectory.error());
    }

    CartesianPlanResult plan_result;
    plan_result.trajectory = std::move(maybe_trajectory.value());
    // Re-timing does not change the geometric path, so reuse the resolved length.
    plan_result.achieved_path_length = tracked->achieved_path_length;
    plan_result.feedrate_efficiency = tracked->feedrate_efficiency;
    computePeakLimitRatios(plan_result.trajectory, plan_result.peak_velocity_ratio,
                           plan_result.peak_acceleration_ratio);
    return plan_result;
  } catch (const std::exception& e) {
    return tl::make_unexpected(std::string("TOPP-RA setup failed: ") + e.what());
  }
}

}  // namespace roboplan
