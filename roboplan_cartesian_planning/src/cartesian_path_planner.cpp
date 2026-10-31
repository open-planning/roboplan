#include <roboplan_cartesian_planning/cartesian_path_planner.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <numeric>
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

/// @brief Hard cap on the number of waypoints handed to TOPP-RA, bounding the
/// time-parameterization problem size (and hence planning time) for pathologically long paths.
constexpr size_t kMaxToppraWaypoints = 10000;

/// @brief Number of blend deviations tried when TOPP-RA rejects a path, each half the last.
/// @details TOPP-RA's forward pass occasionally reports an infeasible interval on a path that is
/// perfectly feasible, and the same path with tighter blending solves fine. The first attempt uses
/// the caller's own value, so a healthy plan pays nothing, and each retry only tightens the
/// blending, so a rescued trajectory hugs the resolved path at least as closely as asked.
constexpr int kMaxBlendAttempts = 3;

/// @brief Hard cap on the number of samples the geometric resolution stage may take, bounding the
/// IK work for a pathologically long path or a very tight pose tolerance.
constexpr size_t kMaxResolutionSamples = 100000;

/// @brief Maximum number of differential-IK iterations spent converging onto one path sample.
constexpr int kMaxIkConvergenceIters = 500;

/// @brief Fraction of the pose tolerance the reference advances between IK seeding samples.
/// @details This walk exists so every IK solve starts from a near neighbor, which keeps the
/// solution away from large jumps. It is not the output resolution: which of these samples
/// become waypoints is decided afterwards, by tool deviation.
constexpr double kSeedStepFraction = 0.5;

/// @brief Share of the pose tolerance allotted to the gaps between resolved samples.
/// @details The tolerance is spent twice over: once on how far the straight joint-space hop between
/// consecutive waypoints bows off the commanded path, and again on how far the time
/// parameterization rounds the corners between them. Splitting it leaves room for both.
constexpr double kSubdivisionToleranceFraction = 0.5;

/// @brief Relative change in a frame's Cartesian velocity across a waypoint that makes the Bounded
/// mode stop there.
/// @details For a pure direction change this is about the turn angle in radians, so
/// a finely sampled curve flows through its waypoints while a sharp corner is stopped at.
constexpr double kCornerTolerance = 10.0 * std::numbers::pi / 180.0;

/// @brief Resolves a joint group, throwing the planner's construction error if it does not exist.
JointGroupInfo resolveJointGroup(const Scene& scene, const std::string& group_name) {
  auto maybe_joint_group_info = scene.getJointGroupInfo(group_name);
  if (!maybe_joint_group_info) {
    throw std::runtime_error("Could not initialize Cartesian path planner: " +
                             maybe_joint_group_info.error());
  }
  return maybe_joint_group_info.value();
}

}  // namespace

Eigen::Matrix4d CartesianPathPlanner::FrameReference::eval(double s) const {
  if (waypoints.empty()) {
    return Eigen::Matrix4d::Identity();
  }
  if (waypoints.size() == 1) {
    return waypoints.back();
  }

  s = std::clamp(s, 0.0, 1.0);

  // Find the segment [i, i+1] containing path parameter s.
  const auto upper = std::upper_bound(cumulative.begin(), cumulative.end(), s);
  size_t i = static_cast<size_t>(std::distance(cumulative.begin(), upper));
  i = std::clamp<size_t>(i, 1, cumulative.size() - 1) - 1;

  const double t0 = cumulative.at(i);
  const double t1 = cumulative.at(i + 1);
  const double span = t1 - t0;
  const double fraction = span > 0.0 ? (s - t0) / span : 0.0;

  return interpolatePose(waypoints.at(i), waypoints.at(i + 1), fraction);
}

CartesianPathPlanner::CartesianPathPlanner(const std::shared_ptr<Scene> scene,
                                           const CartesianPlannerOptions& options)
    : scene_{scene}, options_{options},
      joint_group_info_{resolveJointGroup(*scene, options.group_name)},
      oink_{std::make_shared<Oink>(*scene, options.group_name)},
      toppra_{scene, options.group_name} {
  buildStaticSolverComponents(std::nullopt);
}

CartesianPathPlanner::CartesianPathPlanner(const std::shared_ptr<Scene> scene,
                                           const CartesianPlannerOptions& options,
                                           const CartesianPlannerComponents& components)
    : scene_{scene}, options_{options},
      joint_group_info_{resolveJointGroup(*scene, options.group_name)}, oink_{components.oink},
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
  buildStaticSolverComponents(components);
}

void CartesianPathPlanner::buildStaticSolverComponents(
    const std::optional<CartesianPlannerComponents>& components) {
  const int num_variables = oink_->num_variables;
  const auto maybe_velocity_limits = scene_->getVelocityLimitVectors(options_.group_name);

  if (components) {
    // Caller-supplied setup: the solver objectives are fixed, so take them once.
    // Cache the tracking tasks for per-plan() wiring in buildFrameReferences().
    tracking_tasks_ = components->tracking_tasks;
    extra_tasks_ = components->extra_tasks;
    constraints_ = components->constraints;
    barriers_ = components->barriers;

    return;
  }

  // Default setup: the velocity/position-limit constraints do not depend on the path or seed, so
  // build them once here. The per-end-effector FrameTasks and the nullspace ConfigurationTask are
  // (re)built in buildFrameReferences() because they do.
  if (!maybe_velocity_limits) {
    throw std::runtime_error("Could not initialize Cartesian path planner: could not get joint "
                             "velocity limits: " +
                             maybe_velocity_limits.error());
  }
  Eigen::VectorXd v_max = maybe_velocity_limits->second.cwiseAbs();
  if (v_max.size() != num_variables) {
    throw std::runtime_error(
        "Could not initialize Cartesian path planner: velocity limit vector size (" +
        std::to_string(v_max.size()) + ") does not match the group velocity DOF count (" +
        std::to_string(num_variables) + ").");
  }

  // Bound each IK iteration inside the QP:
  //   - VelocityLimit caps a single step at dt * v_max, keeping iterations well conditioned.
  //   - PositionLimit restricts each step so the integrated configuration stays within limits.
  constraints_ = {std::make_shared<VelocityLimit>(*oink_, options_.dt, v_max),
                  std::make_shared<PositionLimit>(*oink_, options_.position_limit_gain)};
}

tl::expected<void, std::string> CartesianPlannerOptions::validate() const {
  if (dt <= 0.0) {
    return tl::make_unexpected("dt must be strictly positive.");
  }
  if (position_tolerance <= 0.0 || orientation_tolerance <= 0.0) {
    return tl::make_unexpected(
        "position_tolerance and orientation_tolerance must be strictly positive.");
  }
  if (velocity_scale <= 0.0 || velocity_scale > 1.0) {
    return tl::make_unexpected("velocity_scale must be in the interval (0, 1].");
  }
  if (acceleration_scale <= 0.0 || acceleration_scale > 1.0) {
    return tl::make_unexpected("acceleration_scale must be in the interval (0, 1].");
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

  const bool bounded = options_.speed_mode == CartesianSpeedMode::Bounded;
  if (bounded) {
    if (options_.max_linear_speed <= 0.0 || options_.max_angular_speed <= 0.0) {
      return tl::make_unexpected(
          "max_linear_speed and max_angular_speed must be strictly positive.");
    }
    if (options_.max_linear_acceleration <= 0.0 || options_.max_angular_acceleration <= 0.0) {
      return tl::make_unexpected(
          "max_linear_acceleration and max_angular_acceleration must be strictly positive.");
    }
  }

  const auto resolved = resolvePath(path, q_start.positions);
  if (!resolved) {
    return tl::make_unexpected(resolved.error());
  }
  return bounded ? timeBounded(*resolved) : timeOptimal(*resolved);
}

tl::expected<std::vector<CartesianPathPlanner::FrameReference>, std::string>
CartesianPathPlanner::buildFrameReferences(const CartesianPath& path,
                                           const Eigen::VectorXd& q_start_full) {
  const size_t num_frames = path.tforms.size();

  // The Oink FrameTask expects its target expressed in the world frame, while the CartesianPath
  // waypoints are given relative to each frame's base. The base frame is fixed relative to the
  // world for a fixed-base robot, so compute world_T_base once per frame and use it to map each
  // base-relative reference pose into the world frame.
  std::vector<FrameReference> references(num_frames);
  for (size_t f = 0; f < num_frames; ++f) {
    FrameReference& reference = references.at(f);
    reference.tip_frame = path.tip_frames.at(f);
    reference.waypoints = path.tforms.at(f);

    // Arc-length parameterization: measure each segment as the larger of its share of the total
    // translation and its share of the total rotation, so a frame progresses evenly through
    // whichever motion dominates locally. The result is normalized to [0, 1] and carries no
    // timing, which is the whole point: every frame reaches its end at s = 1 together.
    const size_t count = reference.waypoints.size();
    std::vector<double> linear_steps(count, 0.0);
    std::vector<double> angular_steps(count, 0.0);
    for (size_t i = 1; i < count; ++i) {
      const auto [linear_distance, angular_distance] =
          poseError(reference.waypoints.at(i - 1), reference.waypoints.at(i));
      linear_steps.at(i) = linear_distance;
      angular_steps.at(i) = angular_distance;
      reference.linear_length += linear_distance;
      reference.angular_length += angular_distance;
    }

    reference.cumulative.assign(count, 0.0);
    for (size_t i = 1; i < count; ++i) {
      const double linear_share =
          reference.linear_length > kEps ? linear_steps.at(i) / reference.linear_length : 0.0;
      const double angular_share =
          reference.angular_length > kEps ? angular_steps.at(i) / reference.angular_length : 0.0;
      reference.cumulative.at(i) =
          reference.cumulative.at(i - 1) + std::max(linear_share, angular_share);
    }
    const double total = reference.cumulative.back();
    if (total > kEps) {
      for (double& value : reference.cumulative) {
        value /= total;
      }
    }
    reference.cumulative.back() = 1.0;

    // Resolve the tip frame up front so the FrameTask construction below cannot throw.
    if (const auto maybe_tip_id = scene_->getFrameId(reference.tip_frame); !maybe_tip_id) {
      return tl::make_unexpected("Could not resolve tip frame '" + reference.tip_frame +
                                 "': " + maybe_tip_id.error());
    }

    const auto& base_frame = path.base_frames.at(f);
    if (base_frame.empty()) {
      // An empty base frame means the waypoints are already in the world frame.
      reference.world_T_base = Eigen::Matrix4d::Identity();
    } else {
      try {
        reference.world_T_base = oink_->getContext().forwardKinematics(q_start_full, base_frame);
      } catch (const std::exception& e) {
        return tl::make_unexpected(std::string("Could not resolve base frame '") + base_frame +
                                   "': " + e.what());
      }
    }
  }

  // Wire up the per-end-effector tracking tasks.
  // Constraints and barriers were assembled once at construction (see buildStaticSolverComponents).
  if (!tracking_tasks_.empty()) {
    // Caller-supplied setup: map each pre-assembled tracking task to its path frame (matched by
    // order) and validate the ordering.
    for (size_t f = 0; f < num_frames; ++f) {
      const auto& tracking_task = tracking_tasks_.at(f);
      if (tracking_task->frame_name != references.at(f).tip_frame) {
        return tl::make_unexpected("Tracking task " + std::to_string(f) + " tracks frame '" +
                                   tracking_task->frame_name + "' but path end-effector " +
                                   std::to_string(f) + " is '" + references.at(f).tip_frame +
                                   "'. Tracking tasks must be ordered to match the path's tip "
                                   "frames.");
      }
      references.at(f).task = tracking_task;
    }
    return references;
  }

  // Default setup: (re)build one priority-1 frame task per end-effector tracking its reference
  // pose, plus a priority-2 configuration task that gently regularizes redundant joints toward
  // the seed using only the nullspace the frame tasks leave free.
  Oink& oink = *oink_;
  const int num_variables = oink.num_variables;
  for (size_t f = 0; f < num_frames; ++f) {
    CartesianConfiguration target;
    target.base_frame = "";  // FrameTask interprets the target tform in the world frame.
    target.tip_frame = references.at(f).tip_frame;
    target.tform = references.at(f).target(0.0);
    FrameTaskOptions frame_options;
    frame_options.position_cost = options_.position_cost;
    frame_options.orientation_cost = options_.orientation_cost;
    frame_options.task_gain = options_.task_gain;
    frame_options.lm_damping = options_.lm_damping;
    frame_options.position_tolerance = options_.position_tolerance;
    frame_options.orientation_tolerance = options_.orientation_tolerance;
    frame_options.priority = 1;
    references.at(f).task = std::make_shared<FrameTask>(oink, *scene_, target, frame_options);
  }

  const Eigen::VectorXd joint_weights =
      Eigen::VectorXd::Constant(num_variables, options_.config_task_weight);
  ConfigurationTaskOptions config_options;
  config_options.priority = 2;
  const Eigen::VectorXd target_q = q_start_full(oink.q_indices);
  extra_tasks_ = {
      std::make_shared<ConfigurationTask>(oink, target_q, joint_weights, config_options)};
  return references;
}

tl::expected<CartesianPathPlanner::ResolvedPath, std::string>
CartesianPathPlanner::resolvePath(const CartesianPath& path, const Eigen::VectorXd& q_start_full) {
  auto references = buildFrameReferences(path, q_start_full);
  if (!references) {
    return tl::make_unexpected(references.error());
  }
  ResolvedPath resolved;
  resolved.samples.joint_names = scene_->getJointNamesWithMimics();
  resolved.references = std::move(*references);

  // Every frame's waypoints are breakpoints, so every span between consecutive breakpoints is a
  // straight interpolation for all frames.
  std::vector<double> breakpoints;
  for (const auto& reference : resolved.references) {
    breakpoints.insert(breakpoints.end(), reference.cumulative.begin(), reference.cumulative.end());
  }
  std::sort(breakpoints.begin(), breakpoints.end());
  breakpoints.erase(std::unique(breakpoints.begin(), breakpoints.end(),
                                [](double a, double b) { return b - a <= kEps; }),
                    breakpoints.end());

  // For each span, the number of IK seed steps it needs, and for each breakpoint, whether it is a
  // corner to stop at. A corner is where some frame's Cartesian velocity per unit path parameter
  // (translation, and rotation vector in the base frame) changes by more than `kCornerTolerance`
  // of its magnitude, which no smooth timing can pass through.
  const double seed_linear_step = kSeedStepFraction * options_.position_tolerance;
  const double seed_angular_step = kSeedStepFraction * options_.orientation_tolerance;
  std::vector<double> span_steps(breakpoints.size() - 1, 0.0);
  std::vector<bool> corners(breakpoints.size(), false);
  corners.front() = true;
  corners.back() = true;
  for (const auto& reference : resolved.references) {
    Eigen::Matrix<double, 6, 1> rate_before = Eigen::Matrix<double, 6, 1>::Zero();
    for (size_t j = 0; j + 1 < breakpoints.size(); ++j) {
      const Eigen::Matrix4d from = reference.eval(breakpoints.at(j));
      const Eigen::Matrix4d to = reference.eval(breakpoints.at(j + 1));
      const Eigen::Vector3d translation = to.topRightCorner<3, 1>() - from.topRightCorner<3, 1>();
      const Eigen::AngleAxisd rotation(
          Eigen::Matrix3d(to.topLeftCorner<3, 3>() * from.topLeftCorner<3, 3>().transpose()));
      span_steps.at(j) = std::max({span_steps.at(j), translation.norm() / seed_linear_step,
                                   rotation.angle() / seed_angular_step});
      Eigen::Matrix<double, 6, 1> rate;
      rate << translation, rotation.angle() * rotation.axis();
      rate /= breakpoints.at(j + 1) - breakpoints.at(j);
      for (const int offset : {0, 3}) {
        const double change = (rate - rate_before).segment<3>(offset).norm();
        const double scale =
            std::max(rate_before.segment<3>(offset).norm(), rate.segment<3>(offset).norm());
        // The floor ignores the rounding noise of a motion that has no translation or rotation.
        corners.at(j) = corners.at(j) || (j > 0 && change > kCornerTolerance * scale + kEps);
      }
      rate_before = rate;
    }
  }

  // Walk evenly between corners in small steps so every IK solve starts from a near neighbor.
  // Only the corners are resolved exactly: forcing a sample at every waypoint of a finely sampled
  // curve would crowd the samples closer than the IK precision resolves. Each run takes at least
  // two steps, so the tool can get moving between its stops.
  const double total_steps = std::accumulate(span_steps.begin(), span_steps.end(), 0.0);
  const double step_scale = std::min(1.0, static_cast<double>(kMaxResolutionSamples) / total_steps);
  resolved.stops.push_back(0);
  resolved.parameters.push_back(breakpoints.front());
  size_t run_start = 0;
  double run_steps = 0.0;
  for (size_t j = 1; j < breakpoints.size(); ++j) {
    run_steps += span_steps.at(j - 1);
    if (!corners.at(j)) {
      continue;
    }
    const auto steps = std::max<size_t>(2, static_cast<size_t>(std::ceil(run_steps * step_scale)));
    const double from = breakpoints.at(run_start);
    const double to = breakpoints.at(j);
    for (size_t k = 1; k <= steps; ++k) {
      resolved.parameters.push_back(from + static_cast<double>(k) / steps * (to - from));
    }
    resolved.stops.push_back(resolved.parameters.size() - 1);
    run_start = j;
    run_steps = 0.0;
  }

  // Each sample is solved to convergence rather than tracked by a single step, so the robot never
  // trails the reference and no throttling or lag-recovery is needed. The solve keeps iterating
  // past the path tolerance until the error plateaus, so the resolved path is shaped by the
  // commanded motion rather than by residual IK error. A random restart would break the path's
  // continuity, so none are allowed.
  std::vector<std::shared_ptr<Task>> tracking_tasks;
  for (const auto& reference : resolved.references) {
    tracking_tasks.push_back(reference.task);
  }
  IterativeSolveOptions ik_options;
  ik_options.max_iters = kMaxIkConvergenceIters;
  ik_options.max_time = std::numeric_limits<double>::infinity();
  ik_options.max_restarts = 0;
  ik_options.regularization = options_.regularization;
  ik_options.fast_return = false;

  Eigen::VectorXd q = q_start_full;
  for (const double s : resolved.parameters) {
    for (const auto& reference : resolved.references) {
      reference.task->setTargetFrameTransform(reference.target(s));
    }
    const auto solution = oink_->solveIterativeIk(q, tracking_tasks, extra_tasks_, constraints_,
                                                  barriers_, ik_options);
    if (!solution) {
      return tl::make_unexpected(
          "Could not resolve path fraction " + std::to_string(s) + ": " + solution.error() +
          ". The path may leave the reachable workspace or pass through a singularity" +
          (resolved.samples.positions.empty()
               ? ", or q_start may be too far from the first waypoint."
               : "."));
    }
    q = *solution;
    resolved.samples.positions.push_back(q);
  }

  return resolved;
}

tl::expected<JointTrajectory, std::string>
CartesianPathPlanner::timeOptimal(const ResolvedPath& resolved) {
  // Choose the fewest waypoints that still describe the motion. Every waypoint handed to the time
  // parameterization becomes a corner it must slow through, and its blend radius is capped by the
  // adjacent segment, so waypoint count sets the achievable speed directly. An unnecessary sample
  // costs a near-stop and buys nothing, so keep it only when dropping it would let the tool frame
  // deviate away from the intended path.
  //
  // Starting from just the first and last sample, we recursively check whether a straight
  // joint-space interpolation between two kept samples stays close enough to the reference at all
  // the intermediate samples. If the worst deviation exceeds the budget, that sample must be kept
  // and we split the span. Otherwise every intermediate sample is redundant and can be removed.
  //
  // The budget is `kSubdivisionToleranceFraction` of the user's specified pose tolerance, reserving
  // the rest for the corner-rounding that TOPP-RA applies during time parameterization.
  const auto& walked = resolved.samples.positions;
  const double position_budget = kSubdivisionToleranceFraction * options_.position_tolerance;
  const double orientation_budget = kSubdivisionToleranceFraction * options_.orientation_tolerance;
  std::vector<bool> keep(walked.size(), false);
  keep.front() = true;
  keep.back() = true;
  std::vector<std::pair<size_t, size_t>> pending{{0, walked.size() - 1}};
  while (!pending.empty()) {
    const auto [lo, hi] = pending.back();
    pending.pop_back();
    if (hi <= lo + 1) {
      continue;
    }
    double worst = 0.0;
    size_t worst_index = lo;
    for (size_t i = lo + 1; i < hi; ++i) {
      const double fraction = static_cast<double>(i - lo) / static_cast<double>(hi - lo);
      const Eigen::VectorXd q_interp = scene_->interpolate(walked.at(lo), walked.at(hi), fraction);
      for (const auto& reference : resolved.references) {
        const Eigen::Matrix4d fk =
            oink_->getContext().forwardKinematics(q_interp, reference.tip_frame);
        const auto [position_error, orientation_error] =
            poseError(fk, reference.target(resolved.parameters.at(i)));
        // Compare both modalities against their own budget so whichever is tighter decides.
        const double excess = std::max(position_error / std::max(position_budget, kEps),
                                       orientation_error / std::max(orientation_budget, kEps));
        if (excess > worst) {
          worst = excess;
          worst_index = i;
        }
      }
    }
    if (worst > 1.0) {
      keep.at(worst_index) = true;
      pending.push_back({lo, worst_index});
      pending.push_back({worst_index, hi});
    }
  }

  std::vector<Eigen::VectorXd> waypoints;
  for (size_t i = 0; i < walked.size(); ++i) {
    if (keep.at(i)) {
      waypoints.push_back(walked.at(i)(oink_->q_indices).eval());
    }
  }
  if (waypoints.size() < 2) {
    return tl::make_unexpected(
        "Resolved joint path has fewer than 2 waypoints; the Cartesian path may be degenerate "
        "or too short to time-parameterize.");
  }

  TOPPRAOptions toppra_options;
  toppra_options.dt = options_.dt;
  toppra_options.mode = SplineFittingMode::LinearBlend;
  toppra_options.velocity_scale = options_.velocity_scale;
  toppra_options.acceleration_scale = options_.acceleration_scale;

  // A hard cap still bounds the time-parameterization problem size for very long paths.
  JointPath joint_path;
  joint_path.joint_names = joint_group_info_.joint_names;
  joint_path.positions =
      waypoints.size() > kMaxToppraWaypoints
          ? resampleUniform(waypoints, kMaxToppraWaypoints, *scene_, joint_group_info_.q_indices)
          : waypoints;

  std::string last_error;
  double blend_deviation = options_.toppra_blend_deviation;
  for (int attempt = 0; attempt < kMaxBlendAttempts; ++attempt) {
    toppra_options.max_blend_deviation = blend_deviation;
    auto maybe_trajectory = toppra_.generate(joint_path, toppra_options);
    if (maybe_trajectory) {
      return std::move(maybe_trajectory.value());
    }
    last_error = maybe_trajectory.error();
    blend_deviation *= 0.5;
  }
  return tl::make_unexpected("TOPP-RA time parameterization failed: " + last_error);
}

tl::expected<JointTrajectory, std::string>
CartesianPathPlanner::timeBounded(const ResolvedPath& resolved) const {
  const auto maybe_velocity_limits = scene_->getVelocityLimitVectors(options_.group_name);
  const auto maybe_acceleration_limits = scene_->getAccelerationLimitVectors(options_.group_name);
  if (!maybe_velocity_limits || !maybe_acceleration_limits) {
    return tl::make_unexpected("Could not get the joint velocity and acceleration limits.");
  }
  const Eigen::VectorXd v_max = maybe_velocity_limits->second.cwiseAbs() * options_.velocity_scale;
  const Eigen::VectorXd a_max =
      maybe_acceleration_limits->second.cwiseAbs() * options_.acceleration_scale;
  const Eigen::VectorXi& v_indices = joint_group_info_.v_indices;
  const auto& s = resolved.parameters;
  const auto& samples = resolved.samples.positions;
  const double dt = options_.dt;

  // Each run between stops is timed on its own and sampled right away, on one clock.
  JointTrajectory trajectory;
  trajectory.joint_names = joint_group_info_.joint_names;
  double start_time = 0.0;
  size_t i = 0;
  for (size_t r = 0; r + 1 < resolved.stops.size(); ++r) {
    const size_t first = resolved.stops.at(r);
    const size_t last = resolved.stops.at(r + 1);
    const size_t count = last - first + 1;

    // Joint path derivatives q' and q'' with respect to the path parameter at each sample of the
    // run, by finite differences in the tangent space. Inside the run these are the three-point
    // formulas for uneven spacing. The run's ends reuse the neighboring q'' and extrapolate q'.
    std::vector<Eigen::VectorXd> steps(count - 1);
    std::vector<double> spacing(count - 1);
    for (size_t j = 0; j + 1 < count; ++j) {
      steps.at(j) = scene_->difference(samples.at(first + j), samples.at(first + j + 1))(v_indices);
      spacing.at(j) = s.at(first + j + 1) - s.at(first + j);
    }
    std::vector<Eigen::VectorXd> dq(count), ddq(count, Eigen::VectorXd::Zero(v_indices.size()));
    for (size_t j = 1; j + 1 < count; ++j) {
      const double h1 = spacing.at(j - 1);
      const double h2 = spacing.at(j);
      const Eigen::VectorXd slope1 = steps.at(j - 1) / h1;
      const Eigen::VectorXd slope2 = steps.at(j) / h2;
      dq.at(j) = (h2 * slope1 + h1 * slope2) / (h1 + h2);
      ddq.at(j) = 2.0 * (slope2 - slope1) / (h1 + h2);
    }
    if (count > 2) {
      ddq.front() = ddq.at(1);
      ddq.back() = ddq.at(count - 2);
    }
    dq.front() = steps.front() / spacing.front() - 0.5 * spacing.front() * ddq.front();
    dq.back() = steps.back() / spacing.back() + 0.5 * spacing.back() * ddq.back();

    // The commanded Cartesian maxima, as rates of the path parameter, for the fastest-moving frame.
    double linear_rate = 0.0;
    double angular_rate = 0.0;
    for (const auto& reference : resolved.references) {
      for (size_t k = first; k < last; ++k) {
        const auto [linear, angular] =
            poseError(reference.eval(s.at(k)), reference.eval(s.at(k + 1)));
        linear_rate = std::max(linear_rate, linear / (s.at(k + 1) - s.at(k)));
        angular_rate = std::max(angular_rate, angular / (s.at(k + 1) - s.at(k)));
      }
    }
    double speed = std::numeric_limits<double>::infinity();
    double acceleration = std::numeric_limits<double>::infinity();
    if (linear_rate > kEps) {
      speed = std::min(speed, options_.max_linear_speed / linear_rate);
      acceleration = std::min(acceleration, options_.max_linear_acceleration / linear_rate);
    }
    if (angular_rate > kEps) {
      speed = std::min(speed, options_.max_angular_speed / angular_rate);
      acceleration = std::min(acceleration, options_.max_angular_acceleration / angular_rate);
    }
    if (!std::isfinite(speed)) {
      return tl::make_unexpected("The Cartesian path has a span with no tool motion.");
    }

    // The fastest allowed path speed and acceleration at each sample. Along the path, the joint
    // velocity is q' * s_dot and the joint acceleration is q' * s_ddot + q'' * s_dot^2. The speed
    // is capped so that the q'' term takes at most half of each joint's acceleration limit, and the
    // path acceleration gets the rest. Each sample takes the larger derivatives of itself and its
    // neighbors, so the limits also hold between samples.
    std::vector<double> speeds(count), accelerations(count);
    for (size_t j = 0; j < count; ++j) {
      const size_t before = j > 0 ? j - 1 : j;
      const size_t after = j + 1 < count ? j + 1 : j;
      const Eigen::ArrayXd d1 =
          dq.at(before).cwiseAbs().cwiseMax(dq.at(j).cwiseAbs()).cwiseMax(dq.at(after).cwiseAbs());
      const Eigen::ArrayXd d2 = ddq.at(before)
                                    .cwiseAbs()
                                    .cwiseMax(ddq.at(j).cwiseAbs())
                                    .cwiseMax(ddq.at(after).cwiseAbs());
      speeds.at(j) = std::min(
          {speed, (v_max.array() / d1).minCoeff(), (0.5 * a_max.array() / d2).sqrt().minCoeff()});
      accelerations.at(j) = std::min(
          acceleration, ((a_max.array() - d2 * speeds.at(j) * speeds.at(j)) / d1).minCoeff());
    }

    // Start and end at rest, changing speed no faster than the path acceleration allows. Over a
    // span of length h at acceleration a, the squared speed changes by at most 2 * a * h, so a
    // forward and a backward pass give the fastest such profile under the caps. Where no joint
    // limit binds, this is a trapezoid at the commanded Cartesian maxima.
    speeds.front() = 0.0;
    speeds.back() = 0.0;
    for (size_t j = 0; j + 1 < count; ++j) {
      const double a = std::min(accelerations.at(j), accelerations.at(j + 1));
      speeds.at(j + 1) = std::min(speeds.at(j + 1),
                                  std::sqrt(speeds.at(j) * speeds.at(j) + 2.0 * a * spacing.at(j)));
    }
    for (size_t j = count - 1; j > 0; --j) {
      const double a = std::min(accelerations.at(j), accelerations.at(j - 1));
      speeds.at(j - 1) = std::min(
          speeds.at(j - 1), std::sqrt(speeds.at(j) * speeds.at(j) + 2.0 * a * spacing.at(j - 1)));
    }

    // Each span is crossed at constant path acceleration, which gives the time at every sample.
    std::vector<double> times(count, start_time);
    for (size_t j = 0; j + 1 < count; ++j) {
      times.at(j + 1) = times.at(j) + 2.0 * spacing.at(j) / (speeds.at(j) + speeds.at(j + 1));
    }

    // Sample the run on the shared clock.
    // The last run also takes the final point, at rest on the path's end.
    const double end_time = times.back();
    const double sample_until = r + 2 == resolved.stops.size() ? end_time + dt : end_time;
    size_t j = 0;
    for (; static_cast<double>(i) * dt < sample_until; ++i) {
      const double time = std::min(static_cast<double>(i) * dt, end_time);
      while (j + 2 < count && times.at(j + 1) <= time) {
        ++j;
      }
      const double h = spacing.at(j);
      const double path_acceleration =
          (speeds.at(j + 1) * speeds.at(j + 1) - speeds.at(j) * speeds.at(j)) / (2.0 * h);
      const double tau = time - times.at(j);
      const double path_speed = speeds.at(j) + path_acceleration * tau;
      const double u =
          std::clamp((speeds.at(j) * tau + 0.5 * path_acceleration * tau * tau) / h, 0.0, 1.0);
      const Eigen::VectorXd q =
          scene_->interpolate(samples.at(first + j), samples.at(first + j + 1), u);
      const Eigen::VectorXd d1 = (1.0 - u) * dq.at(j) + u * dq.at(j + 1);
      const Eigen::VectorXd d2 = (1.0 - u) * ddq.at(j) + u * ddq.at(j + 1);
      trajectory.times.push_back(time);
      trajectory.positions.push_back(q(joint_group_info_.q_indices));
      trajectory.velocities.push_back(d1 * path_speed);
      trajectory.accelerations.push_back(d1 * path_acceleration + d2 * path_speed * path_speed);
    }
    start_time = end_time;
  }
  return trajectory;
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
        // Skip joints with negligible limits to avoid divide-by-zero, and unlimited joints
        // (infinite, or the max-double default used when no limit is set).
        if (std::abs(limit(i)) > kEps && std::abs(limit(i)) < std::numeric_limits<double>::max()) {
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

  Eigen::VectorXd q_full = oink_->getContext().getJointPositions();
  double length = 0.0;
  for (const auto& tip_frame : path.tip_frames) {
    q_full(joint_group_info_.q_indices) = trajectory.positions.front();
    Eigen::Vector3d previous =
        oink_->getContext().forwardKinematics(q_full, tip_frame).block<3, 1>(0, 3);
    for (size_t i = 1; i < trajectory.positions.size(); ++i) {
      q_full(joint_group_info_.q_indices) = trajectory.positions.at(i);
      const Eigen::Vector3d current =
          oink_->getContext().forwardKinematics(q_full, tip_frame).block<3, 1>(0, 3);
      length += (current - previous).norm();
      previous = current;
    }
  }
  return length;
}

}  // namespace roboplan
