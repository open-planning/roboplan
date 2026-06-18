#include <roboplan_cartesian_planning/cartesian_path_planner.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <tuple>

#include <Eigen/Geometry>

#include <roboplan_oink/constraints/position_limit.hpp>
#include <roboplan_oink/constraints/velocity_limit.hpp>
#include <roboplan_oink/optimal_ik.hpp>
#include <roboplan_oink/tasks/configuration.hpp>
#include <roboplan_oink/tasks/frame.hpp>
#include <roboplan_toppra/toppra.hpp>

namespace roboplan {

namespace {

/// @brief Small epsilon for distance/angle/limit comparisons and strict-inequality slack.
constexpr double kEps = 1e-9;

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
    const double fraction = span > kEps ? (target - cumulative.at(segment)) / span : 0.0;
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
    : scene_{scene}, options_{options}, oink_{std::make_shared<Oink>(*scene, options.group_name)},
      toppra_{scene, options.group_name} {
  const auto maybe_joint_group_info = scene_->getJointGroupInfo(options_.group_name);
  if (!maybe_joint_group_info) {
    throw std::runtime_error("Could not initialize Cartesian path planner: " +
                             maybe_joint_group_info.error());
  }
  joint_group_info_ = maybe_joint_group_info.value();
  buildStaticSolverComponents();
}

CartesianPathPlanner::CartesianPathPlanner(const std::shared_ptr<Scene> scene,
                                           const CartesianPlannerOptions& options,
                                           const CartesianPlannerComponents& components)
    : scene_{scene}, options_{options}, oink_{components.oink}, components_{components},
      toppra_{scene, options.group_name} {
  if (!components.oink) {
    throw std::runtime_error(
        "Could not initialize Cartesian path planner: components.oink must not be null.");
  }
  if (components.tracking_tasks.empty()) {
    throw std::runtime_error("Could not initialize Cartesian path planner: "
                             "components.tracking_tasks must not be empty.");
  }
  for (const auto& tracking_task : components.tracking_tasks) {
    if (!tracking_task) {
      throw std::runtime_error("Could not initialize Cartesian path planner: "
                               "components.tracking_tasks must not contain a null entry.");
    }
  }
  const auto maybe_joint_group_info = scene_->getJointGroupInfo(options_.group_name);
  if (!maybe_joint_group_info) {
    throw std::runtime_error("Could not initialize Cartesian path planner: " +
                             maybe_joint_group_info.error());
  }
  joint_group_info_ = maybe_joint_group_info.value();
  buildStaticSolverComponents();
}

void CartesianPathPlanner::buildStaticSolverComponents() {
  const int num_variables = oink_->num_variables;

  if (components_) {
    // Caller-supplied setup: the solver objectives are fixed, so assemble them once. The
    // tracking tasks are prepended so they are always solved; everything else passes through.
    tasks_.clear();
    tasks_.reserve(components_->tracking_tasks.size() + components_->extra_tasks.size());
    tasks_.insert(tasks_.end(), components_->tracking_tasks.begin(),
                  components_->tracking_tasks.end());
    tasks_.insert(tasks_.end(), components_->extra_tasks.begin(), components_->extra_tasks.end());
    constraints_ = components_->constraints;
    barriers_ = components_->barriers;

    // Verify against the scene's physical velocity limits (unscaled) if available; the caller
    // owns whatever velocity constraint they supplied, so this is only a hard-limit sanity net.
    const auto maybe_velocity_limits = scene_->getVelocityLimitVectors(options_.group_name);
    if (maybe_velocity_limits && maybe_velocity_limits->second.size() == num_variables) {
      verify_v_max_ = maybe_velocity_limits->second.cwiseAbs();
      has_velocity_check_ = true;
    }
    return;
  }

  // Default setup: the velocity/position-limit constraints and the verification limits do not
  // depend on the path or seed, so build them once here. The per-end-effector FrameTasks and the
  // nullspace ConfigurationTask are (re)built in trackReference() because they do.
  const auto maybe_velocity_limits = scene_->getVelocityLimitVectors(options_.group_name);
  if (!maybe_velocity_limits) {
    throw std::runtime_error("Could not initialize Cartesian path planner: could not get joint "
                             "velocity limits: " +
                             maybe_velocity_limits.error());
  }
  Eigen::VectorXd v_max = maybe_velocity_limits->second.cwiseAbs() * options_.velocity_scale;
  if (v_max.size() != num_variables) {
    throw std::runtime_error(
        "Could not initialize Cartesian path planner: velocity limit vector size (" +
        std::to_string(v_max.size()) + ") does not match the group velocity DOF count (" +
        std::to_string(num_variables) + ").");
  }

  // Enforce both joint velocity and joint position limits inside the QP:
  //   - VelocityLimit bounds each step to dt * v_max (so |delta_q|/dt <= v_max).
  //   - PositionLimit restricts each step so the integrated configuration stays within limits.
  constraints_ = {std::make_shared<VelocityLimit>(*oink_, options_.dt, v_max),
                  std::make_shared<PositionLimit>(*oink_, options_.position_limit_gain)};
  verify_v_max_ = std::move(v_max);
  has_velocity_check_ = true;
}

tl::expected<CartesianPlanResult, std::string>
CartesianPathPlanner::plan(const CartesianPath& path, const JointConfiguration& q_start) {
  // Validate options.
  if (options_.dt <= 0.0) {
    return tl::make_unexpected("dt must be strictly positive.");
  }
  if (options_.max_position_error <= 0.0 || options_.max_orientation_error <= 0.0) {
    return tl::make_unexpected(
        "max_position_error and max_orientation_error must be strictly positive.");
  }
  if (options_.velocity_scale <= 0.0 || options_.velocity_scale > 1.0) {
    return tl::make_unexpected("velocity_scale must be in the interval (0, 1].");
  }
  if (options_.acceleration_scale <= 0.0 || options_.acceleration_scale > 1.0) {
    return tl::make_unexpected("acceleration_scale must be in the interval (0, 1].");
  }
  if (options_.limit_ratio_tolerance < 1.0) {
    return tl::make_unexpected("limit_ratio_tolerance must be >= 1.0.");
  }

  // Validate the path: one or more end-effector frames, each with a matching base frame, tip
  // frame, and (non-empty) transform list.
  const size_t num_frames = path.tforms.size();
  if (num_frames < 1) {
    return tl::make_unexpected("The Cartesian path must contain at least one end-effector frame.");
  }
  if (path.base_frames.size() != num_frames || path.tip_frames.size() != num_frames) {
    return tl::make_unexpected(
        "The Cartesian path must contain the same number of base frames, tip frames, and "
        "transform lists (one per end-effector).");
  }
  for (size_t f = 0; f < num_frames; ++f) {
    if (path.tforms.at(f).size() < 1) {
      return tl::make_unexpected("Each Cartesian path frame must contain at least one waypoint (" +
                                 path.tip_frames.at(f) + " has none).");
    }
  }
  // In the custom-components mode there must be exactly one tracking task per end-effector.
  if (components_ && components_->tracking_tasks.size() != num_frames) {
    return tl::make_unexpected("The number of tracking tasks (" +
                               std::to_string(components_->tracking_tasks.size()) +
                               ") must match the number of end-effector frames in the path (" +
                               std::to_string(num_frames) + ").");
  }

  // Validate the seed configuration.
  const auto& model = scene_->getModel();
  if (q_start.positions.size() != model.nq) {
    return tl::make_unexpected("q_start must be a full model configuration of size model.nq (" +
                               std::to_string(model.nq) + "), got " +
                               std::to_string(q_start.positions.size()) + ".");
  }

  switch (options_.speed_mode) {
  case CartesianSpeedMode::Bounded:
    return planBounded(path, q_start.positions);
  case CartesianSpeedMode::TimeOptimal:
    return planTimeOptimal(path, q_start.positions);
  }
  return tl::make_unexpected("Unknown CartesianSpeedMode.");
}

tl::expected<CartesianPathPlanner::TrackResult, std::string>
CartesianPathPlanner::trackReference(const CartesianPath& path, const Eigen::VectorXd& q_start_full,
                                     double linear_speed, double angular_speed,
                                     double linear_acceleration, double angular_acceleration) {
  const auto& model = scene_->getModel();
  const size_t num_frames = path.tforms.size();

  // Per-end-effector tracking state, kept as parallel vectors indexed by frame: the arc-length
  // reference, the fixed world<-base transform, and the FrameTask whose target the planner
  // retargets each step. The tip frame to trace is path.tip_frames.at(f).
  std::vector<Reference> references(num_frames);
  std::vector<Eigen::Matrix4d> world_T_base(num_frames);
  std::vector<std::shared_ptr<FrameTask>> frame_tasks(num_frames);

  // The Oink FrameTask expects its target expressed in the world frame, while the CartesianPath
  // waypoints are given relative to each frame's base. The base frame is fixed relative to the
  // world for a fixed-base robot, so compute world_T_base once per frame and use it to map each
  // base-relative reference pose into the world frame.
  for (size_t f = 0; f < num_frames; ++f) {
    references.at(f) = buildReference(path.tforms.at(f), linear_speed, angular_speed);
    // Resolve the tip frame up front so the FrameTask construction below cannot throw.
    if (const auto maybe_tip_id = scene_->getFrameId(path.tip_frames.at(f)); !maybe_tip_id) {
      return tl::make_unexpected("Could not resolve tip frame '" + path.tip_frames.at(f) +
                                 "': " + maybe_tip_id.error());
    }
    try {
      world_T_base.at(f) = scene_->forwardKinematics(q_start_full, path.base_frames.at(f));
    } catch (const std::exception& e) {
      return tl::make_unexpected(std::string("Could not resolve base frame '") +
                                 path.base_frames.at(f) + "': " + e.what());
    }
  }

  // Common timeline: a single reference time  `s` advances all frames together; each reference
  // clamps `s` to its own duration, so a shorter motion simply holds at its final waypoint. The
  // reference time is the nominal full-speed traversal time: advancing `s` at a rate (feedrate) of
  // 1.0 traces the path at exactly the commanded max linear OR angular speed (whichever binds) on
  // each segment, by construction of buildReference().
  double total_time = 0.0;
  for (const auto& reference : references) {
    total_time = std::max(total_time, reference.total_time);
  }

  // Trapezoidal feedrate profile setup. We bound the feedrate acceleration s_ddot (units 1/s) so
  // the Cartesian acceleration stays within the commanded maxima. On a straight segment the
  // Cartesian linear acceleration is s_ddot * v_nominal, where v_nominal <= linear_speed is the
  // segment's full-speed linear speed; bounding s_ddot <= linear_acceleration / linear_speed
  // therefore keeps it within linear_acceleration (and likewise for rotation). We take the binding
  // (smaller) of the two limits, ignoring a modality the path has no motion in. A non-positive
  // acceleration disables the profile, recovering the constant-feedrate trace used by TimeOptimal.
  bool use_accel_profile = linear_acceleration > 0.0 && angular_acceleration > 0.0;
  double s_ddot_max = 0.0;
  // Feedrate ceiling grid (sampled uniformly in reference time): the maximum feedrate at each `s`
  // such that the motion can still decelerate to every upcoming corner speed cap and stop at the
  // end, all within s_ddot_max. Empty when the profile is disabled.
  std::vector<double> feedrate_ceiling;
  double ceiling_ds = 0.0;
  size_t num_corner_knots = 0;
  if (use_accel_profile) {
    double total_linear_distance = 0.0;
    double total_angular_distance = 0.0;
    for (const auto& reference : references) {
      for (size_t i = 1; i < reference.waypoints.size(); ++i) {
        total_linear_distance += (reference.waypoints.at(i).block<3, 1>(0, 3) -
                                  reference.waypoints.at(i - 1).block<3, 1>(0, 3))
                                     .norm();
        const Eigen::Matrix3d relative_rotation =
            reference.waypoints.at(i - 1).block<3, 3>(0, 0).transpose() *
            reference.waypoints.at(i).block<3, 3>(0, 0);
        total_angular_distance += Eigen::AngleAxisd(relative_rotation).angle();
      }
    }
    s_ddot_max = std::numeric_limits<double>::infinity();
    if (total_linear_distance > kEps && linear_speed > 0.0) {
      s_ddot_max = std::min(s_ddot_max, linear_acceleration / linear_speed);
    }
    if (total_angular_distance > kEps && angular_speed > 0.0) {
      s_ddot_max = std::min(s_ddot_max, angular_acceleration / angular_speed);
    }
    if (!std::isfinite(s_ddot_max) || s_ddot_max <= 0.0 || total_time <= 0.0) {
      use_accel_profile = false;  // Degenerate (zero-length path): fall back to constant feedrate.
    }
  }

  if (use_accel_profile) {
    // Corner speed caps. The reference is piecewise-linear (linear position interpolation, SLERP
    // orientation), so at a waypoint where the path direction turns by an angle theta the commanded
    // velocity direction flips abruptly. Crossing that corner at Cartesian speed v changes the
    // velocity by |dv| = 2 v sin(theta/2) over roughly one control period dt, implying an
    // acceleration |dv|/dt. Capping that at the commanded maximum gives the cornering speed
    // v <= a * dt / (2 sin(theta/2)); converting to a feedrate cap (v / commanded speed) and taking
    // the binding of position and orientation yields the maximum feedrate through the corner. A
    // near-reversal drives the cap toward zero (handled by the feedrate floor in the servo loop).
    constexpr double kCornerAngleMin = 1.0 * M_PI / 180.0;  // Treat smaller turns as straight.
    const auto corner_feedrate_cap = [&](const Eigen::Matrix4d& prev, const Eigen::Matrix4d& cur,
                                         const Eigen::Matrix4d& next) -> double {
      double cap = 1.0;
      const auto cap_from_turn = [&](double deflection, double accel, double speed) {
        if (deflection <= kCornerAngleMin || speed <= 0.0) {
          return;
        }
        const double velocity_change = 2.0 * std::sin(0.5 * deflection);
        cap = std::min(cap, accel * options_.dt / (speed * velocity_change));
      };
      // Position turn.
      const Eigen::Vector3d in_pos = cur.block<3, 1>(0, 3) - prev.block<3, 1>(0, 3);
      const Eigen::Vector3d out_pos = next.block<3, 1>(0, 3) - cur.block<3, 1>(0, 3);
      if (in_pos.norm() > kEps && out_pos.norm() > kEps) {
        const double cos_angle =
            std::clamp(in_pos.normalized().dot(out_pos.normalized()), -1.0, 1.0);
        cap_from_turn(std::acos(cos_angle), linear_acceleration, linear_speed);
      }
      // Orientation turn: change of rotation axis between the incoming and outgoing relative
      // rotations (only meaningful when both actually rotate).
      const Eigen::AngleAxisd in_rot(prev.block<3, 3>(0, 0).transpose() * cur.block<3, 3>(0, 0));
      const Eigen::AngleAxisd out_rot(cur.block<3, 3>(0, 0).transpose() * next.block<3, 3>(0, 0));
      if (in_rot.angle() > kEps && out_rot.angle() > kEps) {
        const double cos_angle = std::clamp(in_rot.axis().dot(out_rot.axis()), -1.0, 1.0);
        cap_from_turn(std::acos(cos_angle), angular_acceleration, angular_speed);
      }
      return std::clamp(cap, 0.0, 1.0);
    };

    // Build the ceiling on a uniform grid in reference time (one knot per control period).
    const size_t num_intervals =
        std::max<size_t>(1, static_cast<size_t>(std::ceil(total_time / options_.dt)));
    ceiling_ds = total_time / static_cast<double>(num_intervals);
    feedrate_ceiling.assign(num_intervals + 1, 1.0);
    feedrate_ceiling.back() = 0.0;  // Stop at the path end.
    for (const auto& reference : references) {
      if (reference.waypoints.size() < 3) {
        continue;
      }
      for (size_t i = 1; i + 1 < reference.waypoints.size(); ++i) {
        const double cap =
            corner_feedrate_cap(reference.waypoints.at(i - 1), reference.waypoints.at(i),
                                reference.waypoints.at(i + 1));
        const size_t k = std::min(
            num_intervals,
            static_cast<size_t>(std::llround(reference.cumulative_times.at(i) / ceiling_ds)));
        feedrate_ceiling.at(k) = std::min(feedrate_ceiling.at(k), cap);
        ++num_corner_knots;
      }
    }
    // Backward pass: enforce that the feedrate at each knot can still decelerate to the cap at the
    // next knot within s_ddot_max (v^2 <= v_next^2 + 2 * a * ds). This turns the per-corner caps
    // into a continuous deceleration envelope the online generator below ramps up against.
    for (size_t k = feedrate_ceiling.size() - 1; k-- > 0;) {
      const double feasible = std::sqrt(feedrate_ceiling.at(k + 1) * feedrate_ceiling.at(k + 1) +
                                        2.0 * s_ddot_max * ceiling_ds);
      feedrate_ceiling.at(k) = std::min(feedrate_ceiling.at(k), feasible);
    }
  }

  // Samples the feedrate ceiling at reference time `s` (linear interpolation between grid knots).
  const auto sample_ceiling = [&](double s_query) -> double {
    if (feedrate_ceiling.empty()) {
      return 1.0;
    }
    const double x =
        std::clamp(s_query / ceiling_ds, 0.0, static_cast<double>(feedrate_ceiling.size() - 1));
    const size_t k = std::min(feedrate_ceiling.size() - 2, static_cast<size_t>(x));
    const double frac = x - static_cast<double>(k);
    return feedrate_ceiling.at(k) + frac * (feedrate_ceiling.at(k + 1) - feedrate_ceiling.at(k));
  };

  // World-frame target pose of frame `f` at reference time `s`.
  const auto frame_target = [&world_T_base, &references](size_t f, double s) -> Eigen::Matrix4d {
    return world_T_base.at(f) * references.at(f).eval(s);
  };

  // Wire up the per-end-effector tracking tasks. The constraints, barriers, and velocity-limit
  // verification were assembled once at construction (see buildStaticSolverComponents); in the
  // custom-components mode the task list is fixed too, while the default mode rebuilds the
  // per-end-effector FrameTasks and nullspace ConfigurationTask here because they depend on the
  // path's frames and the seed configuration. The tip frames were validated above, so the
  // FrameTask construction below cannot throw.
  Oink& oink = *oink_;
  const int num_variables = oink.num_variables;

  if (components_) {
    // Caller-supplied setup: map each pre-assembled tracking task to its path frame (matched by
    // order) and validate the ordering. The solver task list (tasks_) is already built.
    for (size_t f = 0; f < num_frames; ++f) {
      const auto& tracking_task = components_->tracking_tasks.at(f);
      if (tracking_task->frame_name != path.tip_frames.at(f)) {
        return tl::make_unexpected(
            "Tracking task " + std::to_string(f) + " tracks frame '" + tracking_task->frame_name +
            "' but path end-effector " + std::to_string(f) + " is '" + path.tip_frames.at(f) +
            "'. Tracking tasks must be ordered to match the path's tip frames.");
      }
      frame_tasks.at(f) = tracking_task;
    }
  } else {
    // Default setup: (re)build one priority-1 frame task per end-effector tracking its reference
    // pose, plus a priority-2 configuration task that gently regularizes redundant joints toward
    // the seed using only the nullspace the frame tasks leave free. Reuse the tasks_ buffer.
    tasks_.clear();
    tasks_.reserve(num_frames + 1);
    for (size_t f = 0; f < num_frames; ++f) {
      CartesianConfiguration target;
      target.base_frame = "";  // FrameTask interprets the target tform in the world frame.
      target.tip_frame = path.tip_frames.at(f);
      target.tform = frame_target(f, 0.0);
      FrameTaskOptions frame_options;
      frame_options.position_cost = options_.position_cost;
      frame_options.orientation_cost = options_.orientation_cost;
      frame_options.task_gain = options_.task_gain;
      frame_options.lm_damping = options_.lm_damping;
      frame_options.priority = 1;
      frame_tasks.at(f) = std::make_shared<FrameTask>(oink, *scene_, target, frame_options);
      tasks_.push_back(frame_tasks.at(f));
    }

    const Eigen::VectorXd joint_weights =
        Eigen::VectorXd::Constant(num_variables, options_.config_task_weight);
    ConfigurationTaskOptions config_options;
    config_options.priority = 2;
    const Eigen::VectorXd target_q = q_start_full(oink.q_indices);
    tasks_.push_back(
        std::make_shared<ConfigurationTask>(oink, target_q, joint_weights, config_options));
  }

  // Per-step scratch.
  Eigen::VectorXd delta_q(num_variables);
  Eigen::VectorXd delta_q_full(model.nv);
  Eigen::VectorXd q = q_start_full;

  // Runs one differential-IK step that retargets every frame to its pose at reference time
  // `s` from the committed configuration `q`, writing the candidate configuration and the
  // worst-case (max over frames) world-frame FK pose error. Does not commit.
  auto solve_step = [&](double s, Eigen::VectorXd& q_candidate, double& position_error,
                        double& orientation_error) -> tl::expected<void, std::string> {
    // Refresh the scene state to the committed configuration so the Oink tasks read the
    // correct current pose, and retarget every tracking task.
    scene_->setJointPositions(q);
    for (size_t f = 0; f < num_frames; ++f) {
      frame_tasks.at(f)->setTargetFrameTransform(frame_target(f, s));
    }
    delta_q.setZero();
    const auto result =
        oink.solveIk(*scene_, tasks_, constraints_, barriers_, delta_q, options_.regularization);
    if (!result) {
      return tl::make_unexpected(result.error());
    }
    delta_q_full.setZero();
    delta_q_full(oink.v_indices) = delta_q;
    q_candidate = scene_->integrate(q, delta_q_full);

    // Worst-case pose error across all tracked frames drives the tolerance/throttling logic.
    position_error = 0.0;
    orientation_error = 0.0;
    for (size_t f = 0; f < num_frames; ++f) {
      const Eigen::Matrix4d fk = scene_->forwardKinematics(q_candidate, path.tip_frames.at(f));
      double frame_position_error = 0.0;
      double frame_orientation_error = 0.0;
      poseError(fk, frame_target(f, s), frame_position_error, frame_orientation_error);
      position_error = std::max(position_error, frame_position_error);
      orientation_error = std::max(orientation_error, frame_orientation_error);
    }
    return {};
  };

  // Convergence phase: drive every frame onto its first waypoint within tolerance.
  double position_error = 0.0;
  double orientation_error = 0.0;
  bool converged = false;
  for (int i = 0; i < kMaxConvergenceIters; ++i) {
    Eigen::VectorXd q_candidate;
    const auto step = solve_step(0.0, q_candidate, position_error, orientation_error);
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
        " rad). Ensure q_start places the tool(s) at or near the first waypoint.");
  }

  // Initialize the trace at the converged start.
  TrackResult result;
  result.group_positions.push_back(q(oink.q_indices).eval());
  result.group_velocities.push_back(Eigen::VectorXd::Zero(num_variables));
  result.times.push_back(0.0);

  // Per-frame previous tip position, used to accumulate the achieved path length (summed
  // across all end-effectors).
  std::vector<Eigen::Vector3d> previous_positions(num_frames);
  for (size_t f = 0; f < num_frames; ++f) {
    previous_positions.at(f) =
        scene_->forwardKinematics(q, path.tip_frames.at(f)).block<3, 1>(0, 3);
  }

  // Timed servo loop. Each step advances the reference by the commanded feedrate, throttling it
  // further wherever the robot would leave the path tolerance or exceed its joint limits.
  double s = 0.0;
  double feedrate = 0.0;  // Current feedrate ds/dt in [0, 1]; ramped under the trapezoidal profile.
  int total_steps = 0;
  int throttled_steps = 0;
  // The acceleration ramps (start, end, and every corner where the feedrate dips to a stop) extend
  // the motion past the nominal full-speed duration, so size the safety cap on an estimate that
  // adds a full ramp-up/ramp-down per corner knot plus the endpoints.
  const double profile_duration =
      use_accel_profile
          ? total_time + (static_cast<double>(num_corner_knots) + 2.0) * 2.0 / s_ddot_max
          : total_time;
  const int hard_cap = static_cast<int>(std::ceil(profile_duration / options_.dt)) *
                           (options_.max_attempts_per_step + 2) +
                       1000;

  while (s < total_time - kEps) {
    // Commanded feedrate for this step. Without the profile this is simply full speed; with it,
    // ramp toward full speed at s_ddot_max but never above the precomputed deceleration envelope,
    // which forces the feedrate down before each corner cap and to a stop at the path end. This
    // bounds the Cartesian acceleration along straights and through corners alike.
    double feedrate_cmd = 1.0;
    if (use_accel_profile) {
      feedrate_cmd = std::min({1.0, sample_ceiling(s), feedrate + s_ddot_max * options_.dt});
      // Floor the feedrate at one ramp step so a corner whose cap collapses to ~0 (a near-reversal)
      // is crawled through rather than deadlocking the reference. At this speed the corner is
      // traversed in ~one step, keeping its acceleration close to the commanded maximum.
      feedrate_cmd = std::max(feedrate_cmd, s_ddot_max * options_.dt);
    }

    bool committed = false;
    double feedrate_eff = feedrate_cmd;
    Eigen::VectorXd q_candidate;
    double committed_s = s;
    for (int attempt = 0; attempt < options_.max_attempts_per_step; ++attempt) {
      const double s_try = std::min(s + feedrate_eff * options_.dt, total_time);
      const auto step = solve_step(s_try, q_candidate, position_error, orientation_error);
      if (!step) {
        return tl::make_unexpected("Oink solve failed at reference time " + std::to_string(s) +
                                   "s: " + step.error());
      }
      if (position_error <= options_.max_position_error &&
          orientation_error <= options_.max_orientation_error) {
        committed = true;
        committed_s = s_try;
        if (feedrate_eff < feedrate_cmd - kEps) {
          ++throttled_steps;
        }
        break;
      }
      feedrate_eff *= 0.5;
    }

    if (!committed) {
      return tl::make_unexpected(
          "Cartesian planner stalled at reference time " + std::to_string(s) +
          "s: cannot stay within tolerance (position error " + std::to_string(position_error) +
          " m, orientation error " + std::to_string(orientation_error) +
          " rad). The path may be unreachable or pass through a singularity.");
    }

    // Commit the step. Carry the achieved feedrate forward so the next ramp continues from the
    // real speed (a throttled step naturally slows the profile).
    s = committed_s;
    feedrate = feedrate_eff;
    q = q_candidate;

    // The velocity constraint keeps each step within the joint velocity limits inside the
    // QP; verify the committed step actually does, to guard against solver constraint
    // relaxation or numerical drift. A small relative tolerance absorbs the QP's
    // constraint-satisfaction tolerance. Skipped when no velocity limits are available.
    if (has_velocity_check_) {
      constexpr double kLimitRelTolerance = 1e-2;
      const Eigen::ArrayXd velocity_bound =
          options_.dt * verify_v_max_.array() * (1.0 + kLimitRelTolerance) + kEps;
      if ((delta_q.array().abs() > velocity_bound).any()) {
        return tl::make_unexpected(
            "Joint velocity limit exceeded at reference time " + std::to_string(s) +
            "s (peak |q_dot| = " +
            std::to_string((delta_q.array().abs() / options_.dt).maxCoeff()) + " vs. limit " +
            std::to_string(verify_v_max_.maxCoeff()) + ").");
      }
    }
    if (!scene_->isValidConfiguration(q)) {
      return tl::make_unexpected("Joint position limit exceeded at reference time " +
                                 std::to_string(s) + "s.");
    }

    ++total_steps;
    result.times.push_back(total_steps * options_.dt);
    result.group_positions.push_back(q(oink.q_indices).eval());
    result.group_velocities.push_back((delta_q / options_.dt).eval());

    for (size_t f = 0; f < num_frames; ++f) {
      const Eigen::Vector3d current_position =
          scene_->forwardKinematics(q, path.tip_frames.at(f)).block<3, 1>(0, 3);
      result.achieved_path_length += (current_position - previous_positions.at(f)).norm();
      previous_positions.at(f) = current_position;
    }

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
}

std::pair<double, double>
CartesianPathPlanner::computePeakLimitRatios(const JointTrajectory& trajectory) const {
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
        if (std::abs(limit(i)) > kEps) {
          ratio = std::max(ratio, std::abs(value(i)) / std::abs(limit(i)));
        }
      }
    }
    return ratio;
  };

  double velocity_ratio = 0.0;
  double acceleration_ratio = 0.0;
  if (velocity_limits) {
    velocity_ratio = peak_ratio(trajectory.velocities, velocity_limits->second.cwiseAbs());
  }
  if (acceleration_limits) {
    acceleration_ratio =
        peak_ratio(trajectory.accelerations, acceleration_limits->second.cwiseAbs());
  }
  return {velocity_ratio, acceleration_ratio};
}

tl::expected<CartesianPlanResult, std::string>
CartesianPathPlanner::planBounded(const CartesianPath& path, const Eigen::VectorXd& q_start_full) {
  if (options_.max_linear_speed <= 0.0 || options_.max_angular_speed <= 0.0) {
    return tl::make_unexpected("max_linear_speed and max_angular_speed must be strictly positive.");
  }
  if (options_.max_linear_acceleration <= 0.0 || options_.max_angular_acceleration <= 0.0) {
    return tl::make_unexpected(
        "max_linear_acceleration and max_angular_acceleration must be strictly positive.");
  }

  // The Cartesian feedrate profile bounds the *Cartesian* acceleration, but where the Jacobian
  // varies quickly (corners, near-singular poses) the implied *joint* acceleration can still
  // exceed the robot's limits. There is no single-step feedrate that fixes this (slowing one step
  // only trades a high acceleration for a high deceleration), so instead we re-time the whole
  // motion slower: traversing the same path in time-scale m multiplies joint velocities by 1/m and
  // joint accelerations by 1/m^2, so we scale the commanded speeds by 1/m and accelerations by
  // 1/m^2 and retry until the (scaled) joint limits are met. The commanded values therefore act as
  // maxima that are relaxed only as much as the kinematics require.
  constexpr int kMaxSlowdownIters = 6;
  // Target ratios: the velocity constraint is already scaled by velocity_scale inside the QP, and
  // we want the peak acceleration to land within acceleration_scale of the unscaled limit.
  const double velocity_target = options_.velocity_scale;
  const double acceleration_target = options_.acceleration_scale;
  // Accept a small overshoot before slowing further (user-tunable; >= 1.0, default 1.05).
  const double ratio_tolerance = options_.limit_ratio_tolerance;

  double speed_scale = 1.0;
  CartesianPlanResult plan_result;
  for (int iter = 0; iter < kMaxSlowdownIters; ++iter) {
    auto tracked = trackReference(path, q_start_full, options_.max_linear_speed * speed_scale,
                                  options_.max_angular_speed * speed_scale,
                                  options_.max_linear_acceleration * speed_scale * speed_scale,
                                  options_.max_angular_acceleration * speed_scale * speed_scale);
    if (!tracked) {
      // A failure (e.g. a stall) may be speed-induced; halve the speed and retry while we can.
      if (iter + 1 < kMaxSlowdownIters) {
        speed_scale *= 0.5;
        continue;
      }
      return tl::make_unexpected(tracked.error());
    }

    plan_result = CartesianPlanResult{};
    JointTrajectory& trajectory = plan_result.trajectory;
    trajectory.joint_names = joint_group_info_.joint_names;
    trajectory.times = std::move(tracked->times);
    trajectory.positions = std::move(tracked->group_positions);
    trajectory.velocities = std::move(tracked->group_velocities);

    // Fill accelerations by backward finite difference of the velocities.
    const int num_variables =
        trajectory.velocities.empty() ? 0 : static_cast<int>(trajectory.velocities.front().size());
    trajectory.accelerations.assign(trajectory.velocities.size(),
                                    Eigen::VectorXd::Zero(num_variables));
    for (size_t i = 1; i < trajectory.velocities.size(); ++i) {
      trajectory.accelerations.at(i) =
          (trajectory.velocities.at(i) - trajectory.velocities.at(i - 1)) / options_.dt;
    }
    // The motion starts and ends at rest, so the boundary accelerations are zero. The final
    // servo step lands exactly on the path end as a sub-dt fragment, which makes its
    // finite-difference acceleration spuriously large; zero both boundaries so this sampling
    // artifact does not pollute the reported peak or trigger an unnecessary slow-down.
    if (!trajectory.accelerations.empty()) {
      trajectory.accelerations.front().setZero();
      trajectory.accelerations.back().setZero();
    }

    plan_result.achieved_path_length = tracked->achieved_path_length;
    plan_result.feedrate_efficiency = tracked->feedrate_efficiency;
    std::tie(plan_result.peak_velocity_ratio, plan_result.peak_acceleration_ratio) =
        computePeakLimitRatios(trajectory);

    // Within the (scaled) joint limits: accept this trace.
    const bool velocity_ok =
        plan_result.peak_velocity_ratio <= velocity_target * ratio_tolerance + kEps;
    const bool acceleration_ok =
        plan_result.peak_acceleration_ratio <= acceleration_target * ratio_tolerance + kEps;
    if ((velocity_ok && acceleration_ok) || iter + 1 == kMaxSlowdownIters) {
      break;
    }

    // Slow down so the next trace lands within limits: acceleration scales with the square of the
    // speed scale (so sqrt of its overshoot), velocity scales linearly. Damp by 1.1 for headroom.
    double slowdown = 1.0;
    if (!acceleration_ok) {
      slowdown = std::max(
          slowdown, std::sqrt(plan_result.peak_acceleration_ratio / acceleration_target) * 1.1);
    }
    if (!velocity_ok) {
      slowdown = std::max(slowdown, plan_result.peak_velocity_ratio / velocity_target * 1.1);
    }
    speed_scale /= slowdown;
  }

  return plan_result;
}

tl::expected<CartesianPlanResult, std::string>
CartesianPathPlanner::planTimeOptimal(const CartesianPath& path,
                                      const Eigen::VectorXd& q_start_full) {
  // Resolve a dense geometric joint path that hugs the Cartesian path. The resolution
  // speed only sets sampling density and tracking tightness here (TOPP-RA assigns the
  // final timing), so use a deliberately low speed that essentially any robot can follow
  // without lagging: a faster resolution speed makes the robot trail the reference by up
  // to the tolerance band, and TOPP-RA would then faithfully reproduce that lag.
  constexpr double kResolutionLinearSpeed = 0.05;   // m/s
  constexpr double kResolutionAngularSpeed = 0.25;  // rad/s

  // Geometric resolution is velocity-level only; TOPP-RA enforces the acceleration limits in the
  // re-timing stage, so trace at a constant resolution feedrate (no Cartesian acceleration
  // profile).
  auto tracked = trackReference(path, q_start_full, kResolutionLinearSpeed, kResolutionAngularSpeed,
                                /*linear_acceleration=*/0.0, /*angular_acceleration=*/0.0);
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
  TOPPRAOptions toppra_options;
  toppra_options.dt = options_.dt;
  toppra_options.mode = SplineFittingMode::LinearBlend;
  toppra_options.velocity_scale = options_.velocity_scale;
  toppra_options.acceleration_scale = options_.acceleration_scale;
  toppra_options.max_blend_deviation = options_.toppra_blend_deviation;
  auto maybe_trajectory = toppra_.generate(joint_path, toppra_options);
  if (!maybe_trajectory) {
    return tl::make_unexpected("TOPP-RA time parameterization failed: " + maybe_trajectory.error());
  }

  CartesianPlanResult plan_result;
  plan_result.trajectory = std::move(maybe_trajectory.value());
  // Re-timing does not change the geometric path, so reuse the resolved length.
  plan_result.achieved_path_length = tracked->achieved_path_length;
  plan_result.feedrate_efficiency = tracked->feedrate_efficiency;
  std::tie(plan_result.peak_velocity_ratio, plan_result.peak_acceleration_ratio) =
      computePeakLimitRatios(plan_result.trajectory);
  return plan_result;
}

}  // namespace roboplan
