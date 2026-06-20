#include <roboplan_cartesian_planning/cartesian_path_planner.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <Eigen/Geometry>

#include <roboplan/core/path_utils.hpp>
#include <roboplan/core/pose_utils.hpp>
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
constexpr int kMaxIkConvergenceIters = 500;

/// @brief Turns smaller than this (radians) are treated as straight by the corner feedrate cap.
constexpr double kCornerAngleMin = 1.0 * M_PI / 180.0;

/// @brief Maximum number of whole-trace slow-down retries in the Bounded speed mode.
constexpr int kMaxSlowdownIters = 6;

/// @brief Relative tolerance when verifying a committed step against the joint velocity limits,
/// absorbing the QP's constraint-satisfaction tolerance.
constexpr double kLimitRelTolerance = 1e-2;

/// @brief Headroom factor applied to each slow-down so the next trace clears the limit rather
/// than landing exactly on it.
constexpr double kSlowdownHeadroom = 1.1;

/// @brief Hard cap on the number of waypoints handed to TOPP-RA. Normally the full dense diff-IK
/// trace is used as-is (the LinearBlend geometry is density-robust, so decimating only discards
/// path detail); this cap only kicks in for pathologically long paths, decimating to bound the
/// time-parameterization problem size (and hence planning time).
constexpr size_t kMaxToppraWaypoints = 10000;

/// @brief Fixed additive slack (in control steps) added to the servo loop's step budget, on top of
/// the estimate derived from the reference duration and throttling attempts. Guards short motions,
/// whose estimate is tiny, against tripping the safety cap on benign sub-dt fragments and retries.
constexpr int kHardCapStepMargin = 1000;

/// @brief Maximum feedrate (fraction of full speed) the reference may carry through the corner at
/// `cur` between `prev` and `next`, keeping the Cartesian acceleration within the commanded maxima.
/// @details The reference is piecewise-linear (linear position interpolation, SLERP orientation),
/// so at a waypoint where the path direction turns by an angle theta the commanded velocity
/// direction flips abruptly. Crossing the corner at Cartesian speed v changes the velocity by
/// |dv| = 2 v sin(theta/2) over roughly one control period dt, implying an acceleration |dv|/dt.
/// Capping that at the commanded maximum gives the cornering speed v <= a * dt / (2 sin(theta/2));
/// converted to a feedrate (v / commanded speed) and taking the binding of position and
/// orientation yields the cap. A near-reversal drives the cap toward zero (the servo loop's
/// feedrate floor crawls through it).
double cornerFeedrateCap(const Eigen::Matrix4d& prev, const Eigen::Matrix4d& cur,
                         const Eigen::Matrix4d& next, double linear_acceleration,
                         double linear_speed, double angular_acceleration, double angular_speed,
                         double dt) {
  double cap = 1.0;
  const auto cap_from_turn = [&](double deflection, double accel, double speed) {
    if (deflection <= kCornerAngleMin || speed <= 0.0) {
      return;
    }
    const double velocity_change = 2.0 * std::sin(0.5 * deflection);
    cap = std::min(cap, accel * dt / (speed * velocity_change));
  };

  // Position turn.
  const Eigen::Vector3d in_pos = cur.block<3, 1>(0, 3) - prev.block<3, 1>(0, 3);
  const Eigen::Vector3d out_pos = next.block<3, 1>(0, 3) - cur.block<3, 1>(0, 3);
  if (in_pos.norm() > kEps && out_pos.norm() > kEps) {
    const double cos_angle = std::clamp(in_pos.normalized().dot(out_pos.normalized()), -1.0, 1.0);
    cap_from_turn(std::acos(cos_angle), linear_acceleration, linear_speed);
  }
  // Orientation turn: change of rotation axis between the incoming and outgoing relative rotations
  // (only meaningful when both actually rotate).
  const Eigen::AngleAxisd in_rot(prev.block<3, 3>(0, 0).transpose() * cur.block<3, 3>(0, 0));
  const Eigen::AngleAxisd out_rot(cur.block<3, 3>(0, 0).transpose() * next.block<3, 3>(0, 0));
  if (in_rot.angle() > kEps && out_rot.angle() > kEps) {
    const double cos_angle = std::clamp(in_rot.axis().dot(out_rot.axis()), -1.0, 1.0);
    cap_from_turn(std::acos(cos_angle), angular_acceleration, angular_speed);
  }
  return std::clamp(cap, 0.0, 1.0);
}

/// @brief Factor (>= 1) by which to slow a trace so its peak joint velocity/acceleration ratios
/// fall within `tolerance` of the targets; 1.0 means both are already acceptable.
/// @details Re-timing the path in time-scale m multiplies joint velocity by 1/m and joint
/// acceleration by 1/m^2, so acceleration overshoot is corrected by sqrt of its ratio and velocity
/// linearly. Only the offending modality contributes; kSlowdownHeadroom adds margin.
double computeSlowdownFactor(double peak_velocity_ratio, double velocity_target,
                             double peak_acceleration_ratio, double acceleration_target,
                             double tolerance) {
  double slowdown = 1.0;
  if (peak_acceleration_ratio > acceleration_target * tolerance) {
    slowdown = std::max(slowdown, std::sqrt(peak_acceleration_ratio / acceleration_target) *
                                      kSlowdownHeadroom);
  }
  if (peak_velocity_ratio > velocity_target * tolerance) {
    slowdown = std::max(slowdown, peak_velocity_ratio / velocity_target * kSlowdownHeadroom);
  }
  return slowdown;
}

}  // namespace

Eigen::Matrix4d CartesianPathPlanner::FrameTrack::eval(double s) const {
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

  return interpolatePose(waypoints.at(i), waypoints.at(i + 1), fraction);
}

double CartesianPathPlanner::FeedrateProfile::sample(double s) const {
  if (!enabled || ceiling.empty()) {
    return 1.0;
  }
  const double x = std::clamp(s / ds, 0.0, static_cast<double>(ceiling.size() - 1));
  const size_t k = std::min(ceiling.size() - 2, static_cast<size_t>(x));
  const double frac = x - static_cast<double>(k);
  return ceiling.at(k) + frac * (ceiling.at(k + 1) - ceiling.at(k));
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
  buildStaticSolverComponents(std::nullopt);
}

CartesianPathPlanner::CartesianPathPlanner(const std::shared_ptr<Scene> scene,
                                           const CartesianPlannerOptions& options,
                                           const CartesianPlannerComponents& components)
    : scene_{scene}, options_{options}, oink_{components.oink}, toppra_{scene, options.group_name} {
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
  buildStaticSolverComponents(components);
}

void CartesianPathPlanner::buildStaticSolverComponents(
    const std::optional<CartesianPlannerComponents>& components) {
  const int num_variables = oink_->num_variables;
  const auto maybe_velocity_limits = scene_->getVelocityLimitVectors(options_.group_name);

  if (components) {
    // Caller-supplied setup: the solver objectives are fixed, so assemble them once. The tracking
    // tasks are prepended so they are always solved; everything else passes through. Cache the
    // tracking tasks for per-plan() wiring in buildFrameTracks().
    tracking_tasks_ = components->tracking_tasks;
    tasks_.clear();
    tasks_.reserve(tracking_tasks_.size() + components->extra_tasks.size());
    tasks_.insert(tasks_.end(), tracking_tasks_.begin(), tracking_tasks_.end());
    tasks_.insert(tasks_.end(), components->extra_tasks.begin(), components->extra_tasks.end());
    constraints_ = components->constraints;
    barriers_ = components->barriers;

    // The caller owns limit handling through the constraints they supplied, so the planner relies
    // entirely on their OInK setup here. The scene's physical velocity limits (unscaled) are used
    // only as an optional hard-limit verification net: if unavailable or mismatched, we simply
    // skip the per-step check rather than failing (unlike the default setup below, which must build
    // the VelocityLimit constraint itself and therefore requires the limits).
    if (maybe_velocity_limits && maybe_velocity_limits->second.size() == num_variables) {
      verify_v_max_ = maybe_velocity_limits->second.cwiseAbs();
      has_velocity_check_ = true;
    }
    return;
  }

  // Default setup: the velocity/position-limit constraints and the verification limits do not
  // depend on the path or seed, so build them once here. The per-end-effector FrameTasks and the
  // nullspace ConfigurationTask are (re)built in buildFrameTracks() because they do.
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

tl::expected<void, std::string> CartesianPlannerOptions::validate() const {
  if (dt <= 0.0) {
    return tl::make_unexpected("dt must be strictly positive.");
  }
  if (max_position_error <= 0.0 || max_orientation_error <= 0.0) {
    return tl::make_unexpected(
        "max_position_error and max_orientation_error must be strictly positive.");
  }
  if (velocity_scale <= 0.0 || velocity_scale > 1.0) {
    return tl::make_unexpected("velocity_scale must be in the interval (0, 1].");
  }
  if (acceleration_scale <= 0.0 || acceleration_scale > 1.0) {
    return tl::make_unexpected("acceleration_scale must be in the interval (0, 1].");
  }
  if (limit_ratio_tolerance < 1.0) {
    return tl::make_unexpected("limit_ratio_tolerance must be >= 1.0.");
  }
  return {};
}

tl::expected<JointTrajectory, std::string>
CartesianPathPlanner::plan(const CartesianPath& path, const JointConfiguration& q_start) {
  if (const auto valid = options_.validate(); !valid) {
    return tl::make_unexpected(valid.error());
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
  if (!tracking_tasks_.empty() && tracking_tasks_.size() != num_frames) {
    return tl::make_unexpected("The number of tracking tasks (" +
                               std::to_string(tracking_tasks_.size()) +
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

tl::expected<std::vector<CartesianPathPlanner::FrameTrack>, std::string>
CartesianPathPlanner::buildFrameTracks(const CartesianPath& path,
                                       const Eigen::VectorXd& q_start_full, double linear_speed,
                                       double angular_speed) {
  const size_t num_frames = path.tforms.size();

  // The Oink FrameTask expects its target expressed in the world frame, while the CartesianPath
  // waypoints are given relative to each frame's base. The base frame is fixed relative to the
  // world for a fixed-base robot, so compute world_T_base once per frame and use it to map each
  // base-relative reference pose into the world frame.
  std::vector<FrameTrack> tracks(num_frames);
  for (size_t f = 0; f < num_frames; ++f) {
    FrameTrack& track = tracks.at(f);
    track.tip_frame = path.tip_frames.at(f);

    // Arc-length timing: time each segment by whichever of the linear/angular motion takes longer
    // at the commanded speeds, so advancing the reference at feedrate 1.0 traces the path at the
    // binding commanded max speed on each segment.
    track.waypoints = path.tforms.at(f);
    track.cumulative_times.assign(track.waypoints.size(), 0.0);
    for (size_t i = 1; i < track.waypoints.size(); ++i) {
      const auto [linear_distance, angular_distance] =
          poseError(track.waypoints.at(i - 1), track.waypoints.at(i));
      const double linear_time = linear_speed > 0.0 ? linear_distance / linear_speed : 0.0;
      const double angular_time = angular_speed > 0.0 ? angular_distance / angular_speed : 0.0;
      track.cumulative_times.at(i) =
          track.cumulative_times.at(i - 1) + std::max(linear_time, angular_time);
    }
    track.total_time = track.cumulative_times.empty() ? 0.0 : track.cumulative_times.back();

    // Resolve the tip frame up front so the FrameTask construction below cannot throw.
    if (const auto maybe_tip_id = scene_->getFrameId(track.tip_frame); !maybe_tip_id) {
      return tl::make_unexpected("Could not resolve tip frame '" + track.tip_frame +
                                 "': " + maybe_tip_id.error());
    }
    try {
      track.world_T_base = scene_->forwardKinematics(q_start_full, path.base_frames.at(f));
    } catch (const std::exception& e) {
      return tl::make_unexpected(std::string("Could not resolve base frame '") +
                                 path.base_frames.at(f) + "': " + e.what());
    }
  }

  // Wire up the per-end-effector tracking tasks. The constraints, barriers, and velocity-limit
  // verification were assembled once at construction (see buildStaticSolverComponents).
  if (!tracking_tasks_.empty()) {
    // Caller-supplied setup: map each pre-assembled tracking task to its path frame (matched by
    // order) and validate the ordering. The solver task list (tasks_) is already built.
    for (size_t f = 0; f < num_frames; ++f) {
      const auto& tracking_task = tracking_tasks_.at(f);
      if (tracking_task->frame_name != tracks.at(f).tip_frame) {
        return tl::make_unexpected(
            "Tracking task " + std::to_string(f) + " tracks frame '" + tracking_task->frame_name +
            "' but path end-effector " + std::to_string(f) + " is '" + tracks.at(f).tip_frame +
            "'. Tracking tasks must be ordered to match the path's tip frames.");
      }
      tracks.at(f).task = tracking_task;
    }
    return tracks;
  }

  // Default setup: (re)build one priority-1 frame task per end-effector tracking its reference
  // pose, plus a priority-2 configuration task that gently regularizes redundant joints toward
  // the seed using only the nullspace the frame tasks leave free. Reuse the tasks_ buffer.
  Oink& oink = *oink_;
  const int num_variables = oink.num_variables;
  tasks_.clear();
  tasks_.reserve(num_frames + 1);
  for (size_t f = 0; f < num_frames; ++f) {
    CartesianConfiguration target;
    target.base_frame = "";  // FrameTask interprets the target tform in the world frame.
    target.tip_frame = tracks.at(f).tip_frame;
    target.tform = tracks.at(f).target(0.0);
    FrameTaskOptions frame_options;
    frame_options.position_cost = options_.position_cost;
    frame_options.orientation_cost = options_.orientation_cost;
    frame_options.task_gain = options_.task_gain;
    frame_options.lm_damping = options_.lm_damping;
    frame_options.priority = 1;
    tracks.at(f).task = std::make_shared<FrameTask>(oink, *scene_, target, frame_options);
    tasks_.push_back(tracks.at(f).task);
  }

  const Eigen::VectorXd joint_weights =
      Eigen::VectorXd::Constant(num_variables, options_.config_task_weight);
  ConfigurationTaskOptions config_options;
  config_options.priority = 2;
  const Eigen::VectorXd target_q = q_start_full(oink.q_indices);
  tasks_.push_back(
      std::make_shared<ConfigurationTask>(oink, target_q, joint_weights, config_options));
  return tracks;
}

CartesianPathPlanner::FeedrateProfile CartesianPathPlanner::buildFeedrateProfile(
    const std::vector<FrameTrack>& tracks, double total_time, double linear_speed,
    double angular_speed, double linear_acceleration, double angular_acceleration) const {
  FeedrateProfile profile;
  if (linear_acceleration <= 0.0 || angular_acceleration <= 0.0) {
    return profile;  // Disabled: constant full feedrate (used by the TimeOptimal resolution pass).
  }

  // Bound the feedrate acceleration s_ddot (units 1/s) so the Cartesian acceleration stays within
  // the commanded maxima. On a straight segment the Cartesian linear acceleration is
  // s_ddot * v_nominal, where v_nominal <= linear_speed; bounding s_ddot <= linear_acceleration /
  // linear_speed therefore keeps it within linear_acceleration (and likewise for rotation). Take
  // the binding (smaller) limit, ignoring a modality the path has no motion in.
  double total_linear_distance = 0.0;
  double total_angular_distance = 0.0;
  for (const auto& track : tracks) {
    const auto& waypoints = track.waypoints;
    for (size_t i = 1; i < waypoints.size(); ++i) {
      const auto [linear_distance, angular_distance] =
          poseError(waypoints.at(i - 1), waypoints.at(i));
      total_linear_distance += linear_distance;
      total_angular_distance += angular_distance;
    }
  }
  double s_ddot_max = std::numeric_limits<double>::infinity();
  if (total_linear_distance > kEps && linear_speed > 0.0) {
    s_ddot_max = std::min(s_ddot_max, linear_acceleration / linear_speed);
  }
  if (total_angular_distance > kEps && angular_speed > 0.0) {
    s_ddot_max = std::min(s_ddot_max, angular_acceleration / angular_speed);
  }
  if (!std::isfinite(s_ddot_max) || s_ddot_max <= 0.0 || total_time <= 0.0) {
    return profile;  // Degenerate (zero-length path): fall back to constant feedrate.
  }

  // Feedrate ceiling on a uniform grid in reference time (one knot per control period): start at
  // full speed everywhere, force a stop at the path end, then clamp each corner knot to its cap.
  const size_t num_intervals =
      std::max<size_t>(1, static_cast<size_t>(std::ceil(total_time / options_.dt)));
  const double ds = total_time / static_cast<double>(num_intervals);
  std::vector<double> ceiling(num_intervals + 1, 1.0);
  ceiling.back() = 0.0;
  size_t num_corner_knots = 0;
  for (const auto& track : tracks) {
    const auto& waypoints = track.waypoints;
    if (waypoints.size() < 3) {
      continue;
    }
    for (size_t i = 1; i + 1 < waypoints.size(); ++i) {
      const double cap = cornerFeedrateCap(waypoints.at(i - 1), waypoints.at(i),
                                           waypoints.at(i + 1), linear_acceleration, linear_speed,
                                           angular_acceleration, angular_speed, options_.dt);
      const size_t k = std::min(
          num_intervals, static_cast<size_t>(std::llround(track.cumulative_times.at(i) / ds)));
      ceiling.at(k) = std::min(ceiling.at(k), cap);
      ++num_corner_knots;
    }
  }
  // Backward pass: enforce that the feedrate at each knot can still decelerate to the cap at the
  // next knot within s_ddot_max (v^2 <= v_next^2 + 2 * a * ds). This turns the per-corner caps
  // into a continuous deceleration envelope the servo loop ramps up against.
  for (size_t k = ceiling.size() - 1; k-- > 0;) {
    const double feasible =
        std::sqrt(ceiling.at(k + 1) * ceiling.at(k + 1) + 2.0 * s_ddot_max * ds);
    ceiling.at(k) = std::min(ceiling.at(k), feasible);
  }

  profile.enabled = true;
  profile.ceiling = std::move(ceiling);
  profile.ds = ds;
  profile.s_ddot_max = s_ddot_max;
  profile.num_corner_knots = num_corner_knots;
  return profile;
}

tl::expected<void, std::string>
CartesianPathPlanner::solveStep(const std::vector<FrameTrack>& tracks, const Eigen::VectorXd& q,
                                double s, Eigen::VectorXd& q_candidate, Eigen::VectorXd& delta_q,
                                double& position_error, double& orientation_error) {
  Oink& oink = *oink_;

  // Refresh the scene state to the committed configuration so the Oink tasks read the correct
  // current pose, and retarget every tracking task.
  scene_->setJointPositions(q);
  for (const auto& track : tracks) {
    track.task->setTargetFrameTransform(track.target(s));
  }
  delta_q.setZero();
  const auto result =
      oink.solveIk(*scene_, tasks_, constraints_, barriers_, delta_q, options_.regularization);
  if (!result) {
    return tl::make_unexpected(result.error());
  }
  Eigen::VectorXd delta_q_full = Eigen::VectorXd::Zero(scene_->getModel().nv);
  delta_q_full(oink.v_indices) = delta_q;
  q_candidate = scene_->integrate(q, delta_q_full);

  // Worst-case pose error across all tracked frames drives the tolerance/throttling logic.
  position_error = 0.0;
  orientation_error = 0.0;
  for (const auto& track : tracks) {
    const Eigen::Matrix4d fk = scene_->forwardKinematics(q_candidate, track.tip_frame);
    const auto [frame_position_error, frame_orientation_error] = poseError(fk, track.target(s));
    position_error = std::max(position_error, frame_position_error);
    orientation_error = std::max(orientation_error, frame_orientation_error);
  }
  return {};
}

tl::expected<void, std::string>
CartesianPathPlanner::convergeToStart(const std::vector<FrameTrack>& tracks, Eigen::VectorXd& q) {
  Eigen::VectorXd delta_q(oink_->num_variables);
  Eigen::VectorXd q_candidate;
  double position_error = 0.0;
  double orientation_error = 0.0;
  for (int i = 0; i < kMaxIkConvergenceIters; ++i) {
    const auto step =
        solveStep(tracks, q, 0.0, q_candidate, delta_q, position_error, orientation_error);
    if (!step) {
      return tl::make_unexpected("Oink solve failed while converging to the first waypoint: " +
                                 step.error());
    }
    q = q_candidate;
    if (position_error <= options_.max_position_error &&
        orientation_error <= options_.max_orientation_error) {
      return {};
    }
  }
  return tl::make_unexpected(
      "Could not converge to the first waypoint within tolerance (position error " +
      std::to_string(position_error) + " m, orientation error " +
      std::to_string(orientation_error) +
      " rad). Ensure q_start places the tool(s) at or near the first waypoint.");
}

tl::expected<JointTrajectory, std::string>
CartesianPathPlanner::runServoLoop(const std::vector<FrameTrack>& tracks,
                                   const FeedrateProfile& profile, double total_time,
                                   Eigen::VectorXd& q) {
  Oink& oink = *oink_;
  const int num_variables = oink.num_variables;

  // Initialize the trace at the converged start. The accelerations are filled by the plan*()
  // caller; here we only produce positions, velocities, and times.
  JointTrajectory trajectory;
  trajectory.joint_names = joint_group_info_.joint_names;
  trajectory.positions.push_back(q(oink.q_indices).eval());
  trajectory.velocities.push_back(Eigen::VectorXd::Zero(num_variables));
  trajectory.times.push_back(0.0);

  // The acceleration ramps (start, end, and every corner where the feedrate dips to a stop) extend
  // the motion past the nominal full-speed duration, so size the safety cap on an estimate that
  // adds a full ramp-up/ramp-down per corner knot plus the endpoints.
  const double profile_duration =
      profile.enabled ? total_time + (static_cast<double>(profile.num_corner_knots) + 2.0) * 2.0 /
                                         profile.s_ddot_max
                      : total_time;
  const int hard_cap = static_cast<int>(std::ceil(profile_duration / options_.dt)) *
                           (options_.max_attempts_per_step + 2) +
                       kHardCapStepMargin;

  // Per-step scratch reused across iterations.
  Eigen::VectorXd delta_q(num_variables);
  Eigen::VectorXd q_candidate;
  double position_error = 0.0;
  double orientation_error = 0.0;

  double s = 0.0;
  double feedrate = 0.0;  // Current feedrate ds/dt in [0, 1]; ramped under the trapezoidal profile.
  int total_steps = 0;
  while (s < total_time - kEps) {
    // Commanded feedrate for this step. Without the profile this is full speed; with it, ramp
    // toward full speed at s_ddot_max but never above the precomputed deceleration envelope (which
    // forces the feedrate down before each corner cap and to a stop at the path end), then floor it
    // at one ramp step so a near-reversal corner is crawled through rather than deadlocking.
    double feedrate_cmd = 1.0;
    if (profile.enabled) {
      const double ramped = feedrate + profile.s_ddot_max * options_.dt;
      feedrate_cmd = std::min({1.0, profile.sample(s), ramped});
      feedrate_cmd = std::max(feedrate_cmd, profile.s_ddot_max * options_.dt);
    }

    bool committed = false;
    double feedrate_eff = feedrate_cmd;
    double committed_s = s;
    for (int attempt = 0; attempt < options_.max_attempts_per_step; ++attempt) {
      const double s_try = std::min(s + feedrate_eff * options_.dt, total_time);
      const auto step =
          solveStep(tracks, q, s_try, q_candidate, delta_q, position_error, orientation_error);
      if (!step) {
        return tl::make_unexpected("Oink solve failed at reference time " + std::to_string(s) +
                                   "s: " + step.error());
      }
      if (position_error <= options_.max_position_error &&
          orientation_error <= options_.max_orientation_error) {
        committed = true;
        committed_s = s_try;
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

    // The velocity constraint keeps each step within the joint velocity limits inside the QP;
    // verify the committed step actually does, to guard against solver constraint relaxation or
    // numerical drift. Skipped when no velocity limits are available.
    if (has_velocity_check_) {
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
    trajectory.times.push_back(total_steps * options_.dt);
    trajectory.positions.push_back(q(oink.q_indices).eval());
    trajectory.velocities.push_back((delta_q / options_.dt).eval());

    if (total_steps > hard_cap) {
      return tl::make_unexpected(
          "Cartesian planner exceeded the maximum number of control steps (" +
          std::to_string(hard_cap) + "). The path may be infeasible at the requested tolerance.");
    }
  }

  return trajectory;
}

tl::expected<JointTrajectory, std::string>
CartesianPathPlanner::trackReference(const CartesianPath& path, const Eigen::VectorXd& q_start_full,
                                     double linear_speed, double angular_speed,
                                     double linear_acceleration, double angular_acceleration) {
  auto tracks = buildFrameTracks(path, q_start_full, linear_speed, angular_speed);
  if (!tracks) {
    return tl::make_unexpected(tracks.error());
  }

  // Common timeline: a single reference time `s` advances all frames together; each reference
  // clamps `s` to its own duration, so a shorter motion simply holds at its final waypoint.
  double total_time = 0.0;
  for (const auto& track : *tracks) {
    total_time = std::max(total_time, track.total_time);
  }

  const FeedrateProfile profile = buildFeedrateProfile(
      *tracks, total_time, linear_speed, angular_speed, linear_acceleration, angular_acceleration);

  Eigen::VectorXd q = q_start_full;
  if (const auto converged = convergeToStart(*tracks, q); !converged) {
    return tl::make_unexpected(converged.error());
  }
  return runServoLoop(*tracks, profile, total_time, q);
}

void CartesianPathPlanner::fillAccelerations(JointTrajectory& trajectory) const {
  // Fill accelerations by backward finite difference of the velocities.
  const int num_variables =
      trajectory.velocities.empty() ? 0 : static_cast<int>(trajectory.velocities.front().size());
  trajectory.accelerations.assign(trajectory.velocities.size(),
                                  Eigen::VectorXd::Zero(num_variables));
  for (size_t i = 1; i < trajectory.velocities.size(); ++i) {
    trajectory.accelerations.at(i) =
        (trajectory.velocities.at(i) - trajectory.velocities.at(i - 1)) / options_.dt;
  }
  // The motion starts and ends at rest, so the boundary accelerations are zero. The final servo
  // step lands exactly on the path end as a sub-dt fragment, which makes its finite-difference
  // acceleration spuriously large; zero both boundaries so this sampling artifact does not pollute
  // the reported peak or trigger an unnecessary slow-down.
  if (!trajectory.accelerations.empty()) {
    trajectory.accelerations.front().setZero();
    trajectory.accelerations.back().setZero();
  }
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

double CartesianPathPlanner::computeAchievedPathLength(const JointTrajectory& trajectory,
                                                       const CartesianPath& path) const {
  if (trajectory.positions.size() < 2 || path.tip_frames.empty()) {
    return 0.0;
  }

  // The trajectory stores only the group coordinates; forwardKinematics needs a full model
  // configuration. Reuse one buffer, writing each waypoint into the group slice. The non-group
  // joints are held at the scene's current state: that is a constant rigid offset on every tip
  // pose, which cancels in the per-step differences below.
  Eigen::VectorXd q_full = scene_->getCurrentJointPositions();

  double length = 0.0;
  for (const auto& tip_frame : path.tip_frames) {
    q_full(joint_group_info_.q_indices) = trajectory.positions.front();
    Eigen::Vector3d previous = scene_->forwardKinematics(q_full, tip_frame).block<3, 1>(0, 3);
    for (size_t i = 1; i < trajectory.positions.size(); ++i) {
      q_full(joint_group_info_.q_indices) = trajectory.positions.at(i);
      const Eigen::Vector3d current =
          scene_->forwardKinematics(q_full, tip_frame).block<3, 1>(0, 3);
      length += (current - previous).norm();
      previous = current;
    }
  }
  return length;
}

tl::expected<JointTrajectory, std::string>
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
  // motion slower and retry until the (scaled) joint limits are met (see computeSlowdownFactor).
  // The commanded values therefore act as maxima that are relaxed only as the kinematics require.
  const double velocity_target = options_.velocity_scale;
  const double acceleration_target = options_.acceleration_scale;
  const double ratio_tolerance = options_.limit_ratio_tolerance;

  double speed_scale = 1.0;
  JointTrajectory trajectory;
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

    // trackReference returns the trace with positions/velocities/times; fill its accelerations so
    // the peak limit ratios that drive the slow-down decision are available.
    trajectory = std::move(*tracked);
    fillAccelerations(trajectory);
    const auto [peak_velocity_ratio, peak_acceleration_ratio] = computePeakLimitRatios(trajectory);

    // Accept the trace once it lands within the (scaled) joint limits; otherwise slow the whole
    // motion down and retry.
    const double slowdown =
        computeSlowdownFactor(peak_velocity_ratio, velocity_target, peak_acceleration_ratio,
                              acceleration_target, ratio_tolerance);
    if (slowdown <= 1.0 || iter + 1 == kMaxSlowdownIters) {
      break;
    }
    speed_scale /= slowdown;
  }

  return trajectory;
}

tl::expected<JointTrajectory, std::string>
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
  // (straight segments have zero curvature), so decimating only throws away path detail.
  // Decimate solely as a safety cap on the problem size for very long paths.
  const std::vector<Eigen::VectorXd>& trace_positions = tracked->positions;
  JointPath joint_path;
  joint_path.joint_names = joint_group_info_.joint_names;
  joint_path.positions = trace_positions.size() > kMaxToppraWaypoints
                             ? resampleUniform(trace_positions, kMaxToppraWaypoints, *scene_,
                                               joint_group_info_.q_indices)
                             : trace_positions;
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
  return std::move(maybe_trajectory.value());
}

}  // namespace roboplan
