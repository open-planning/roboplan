#include <roboplan_cartesian_planning/cartesian_path_planner.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
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

/// @brief Maximum passes the Bounded mode spends slowing a trajectory under the Cartesian maxima.
/// @details Scaling the joint limits by (1/m, 1/m^2) reproduces the m-times-slower trajectory
/// exactly, so one pass normally suffices; the rest absorb TOPP-RA's discretization.
constexpr int kMaxBoundedSlowdownPasses = 4;

/// @brief Relative slack on the Cartesian maxima, so the Bounded slow-down stops once it is close
/// rather than chasing the last fraction of a percent through repeated re-timings.
constexpr double kBoundedSpeedTolerance = 1.02;

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

  double velocity_scale = options_.velocity_scale;
  double acceleration_scale = options_.acceleration_scale;
  auto trajectory = timeParameterize(*resolved, velocity_scale, acceleration_scale);
  if (!trajectory) {
    return tl::make_unexpected(trajectory.error());
  }
  if (!bounded) {
    return trajectory;
  }

  // Bounded mode: the path is timed against the joint limits above, then the whole motion is
  // slowed until the tool obeys the commanded Cartesian maxima too. Re-timing in time-scale m
  // divides speed by m and acceleration by m^2, and handing TOPP-RA joint limits scaled the same
  // way reproduces exactly that slower trajectory, so the joint limits stay satisfied for free.
  for (int pass = 0; pass < kMaxBoundedSlowdownPasses; ++pass) {
    const CartesianPeaks peaks = computeCartesianPeaks(*trajectory, path);
    double slowdown = 1.0;
    slowdown = std::max(slowdown, peaks.linear_speed / options_.max_linear_speed);
    slowdown = std::max(slowdown, peaks.angular_speed / options_.max_angular_speed);
    slowdown =
        std::max(slowdown, std::sqrt(peaks.linear_acceleration / options_.max_linear_acceleration));
    slowdown = std::max(slowdown,
                        std::sqrt(peaks.angular_acceleration / options_.max_angular_acceleration));
    if (slowdown <= kBoundedSpeedTolerance) {
      break;
    }
    velocity_scale /= slowdown;
    acceleration_scale /= slowdown * slowdown;
    auto slower = timeParameterize(*resolved, velocity_scale, acceleration_scale);
    if (!slower) {
      // The faster trajectory is still valid against the joint limits, so keep it rather than
      // failing outright; it simply exceeds the commanded Cartesian caps.
      break;
    }
    trajectory = std::move(slower);
  }
  return trajectory;
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

tl::expected<std::vector<Eigen::VectorXd>, std::string>
CartesianPathPlanner::resolvePath(const CartesianPath& path, const Eigen::VectorXd& q_start_full) {
  auto references = buildFrameReferences(path, q_start_full);
  if (!references) {
    return tl::make_unexpected(references.error());
  }

  // Walk the path in small steps so every IK solve starts from a near neighbor.
  // This helps seed IK solutions to see if the path can be followed without large jumps.
  const double seed_linear_step = kSeedStepFraction * options_.max_position_error;
  const double seed_angular_step = kSeedStepFraction * options_.max_orientation_error;
  double steps = 1.0;
  for (const auto& reference : *references) {
    if (seed_linear_step > 0.0) {
      steps = std::max(steps, reference.linear_length / seed_linear_step);
    }
    if (seed_angular_step > 0.0) {
      steps = std::max(steps, reference.angular_length / seed_angular_step);
    }
  }
  const size_t num_steps =
      std::clamp<size_t>(static_cast<size_t>(std::ceil(steps)), 1, kMaxResolutionSamples);

  // Each sample is solved to convergence rather than tracked by a single step, so the robot never
  // trails the reference and no throttling or lag-recovery is needed. The solve keeps iterating
  // past the path tolerance until the error plateaus, so the resolved path is shaped by the
  // commanded motion rather than by residual IK error. A random restart would break the path's
  // continuity, so none are allowed.
  std::vector<std::shared_ptr<Task>> tracking_tasks;
  for (const auto& reference : *references) {
    tracking_tasks.push_back(reference.task);
  }
  IterativeSolveOptions ik_options;
  ik_options.max_iters = kMaxIkConvergenceIters;
  ik_options.max_time = std::numeric_limits<double>::infinity();
  ik_options.max_restarts = 0;
  ik_options.max_task_error_norm =
      std::min(options_.max_position_error, options_.max_orientation_error);
  ik_options.regularization = options_.regularization;
  ik_options.fast_return = false;

  std::vector<double> parameters(num_steps + 1);
  std::vector<Eigen::VectorXd> walked(num_steps + 1);
  Eigen::VectorXd q = q_start_full;
  for (size_t k = 0; k <= num_steps; ++k) {
    const double s = static_cast<double>(k) / static_cast<double>(num_steps);
    for (const auto& reference : *references) {
      reference.task->setTargetFrameTransform(reference.target(s));
    }
    const auto solution = oink_->solveIterativeIk(q, tracking_tasks, extra_tasks_, constraints_,
                                                  barriers_, ik_options);
    if (!solution) {
      return tl::make_unexpected(
          "Could not resolve path fraction " + std::to_string(s) + ": " + solution.error() +
          ". The path may leave the reachable workspace or pass through a singularity" +
          (k == 0 ? ", or q_start may be too far from the first waypoint." : "."));
    }
    parameters.at(k) = s;
    walked.at(k) = q = *solution;
  }

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
  const double position_budget = kSubdivisionToleranceFraction * options_.max_position_error;
  const double orientation_budget = kSubdivisionToleranceFraction * options_.max_orientation_error;
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
      for (const auto& reference : *references) {
        const Eigen::Matrix4d fk =
            oink_->getContext().forwardKinematics(q_interp, reference.tip_frame);
        const auto [position_error, orientation_error] =
            poseError(fk, reference.target(parameters.at(i)));
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

  std::vector<Eigen::VectorXd> resolved;
  resolved.reserve(walked.size());
  for (size_t i = 0; i < walked.size(); ++i) {
    if (keep.at(i)) {
      resolved.push_back(walked.at(i)(oink_->q_indices).eval());
    }
  }
  return resolved;
}

tl::expected<JointTrajectory, std::string>
CartesianPathPlanner::timeParameterize(const std::vector<Eigen::VectorXd>& resolved,
                                       double velocity_scale, double acceleration_scale) {
  if (resolved.size() < 2) {
    return tl::make_unexpected(
        "Resolved joint path has fewer than 2 waypoints; the Cartesian path may be degenerate "
        "or too short to time-parameterize.");
  }

  TOPPRAOptions toppra_options;
  toppra_options.dt = options_.dt;
  toppra_options.mode = SplineFittingMode::LinearBlend;
  toppra_options.velocity_scale = velocity_scale;
  toppra_options.acceleration_scale = acceleration_scale;

  // A hard cap still bounds the time-parameterization problem size for very long paths.
  JointPath joint_path;
  joint_path.joint_names = joint_group_info_.joint_names;
  joint_path.positions =
      resolved.size() > kMaxToppraWaypoints
          ? resampleUniform(resolved, kMaxToppraWaypoints, *scene_, joint_group_info_.q_indices)
          : resolved;

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

CartesianPathPlanner::CartesianPeaks
CartesianPathPlanner::computeCartesianPeaks(const JointTrajectory& trajectory,
                                            const CartesianPath& path) const {
  CartesianPeaks peaks;
  if (trajectory.positions.size() < 2 || options_.dt <= 0.0) {
    return peaks;
  }

  Eigen::VectorXd q_full = oink_->getContext().getJointPositions();
  for (const auto& tip_frame : path.tip_frames) {
    std::vector<double> linear_speeds;
    std::vector<double> angular_speeds;
    linear_speeds.reserve(trajectory.positions.size());
    angular_speeds.reserve(trajectory.positions.size());

    q_full(joint_group_info_.q_indices) = trajectory.positions.front();
    Eigen::Matrix4d previous = oink_->getContext().forwardKinematics(q_full, tip_frame);
    for (size_t i = 1; i < trajectory.positions.size(); ++i) {
      q_full(joint_group_info_.q_indices) = trajectory.positions.at(i);
      const Eigen::Matrix4d current = oink_->getContext().forwardKinematics(q_full, tip_frame);
      const auto [linear_distance, angular_distance] = poseError(previous, current);
      linear_speeds.push_back(linear_distance / options_.dt);
      angular_speeds.push_back(angular_distance / options_.dt);
      previous = current;
    }

    for (const double speed : linear_speeds) {
      peaks.linear_speed = std::max(peaks.linear_speed, speed);
    }
    for (const double speed : angular_speeds) {
      peaks.angular_speed = std::max(peaks.angular_speed, speed);
    }
    for (size_t i = 1; i < linear_speeds.size(); ++i) {
      peaks.linear_acceleration =
          std::max(peaks.linear_acceleration,
                   std::abs(linear_speeds.at(i) - linear_speeds.at(i - 1)) / options_.dt);
      peaks.angular_acceleration =
          std::max(peaks.angular_acceleration,
                   std::abs(angular_speeds.at(i) - angular_speeds.at(i - 1)) / options_.dt);
    }
  }
  return peaks;
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
