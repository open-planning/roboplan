#pragma once

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <tl/expected.hpp>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>
#include <roboplan_oink/optimal_ik.hpp>
#include <roboplan_toppra/toppra.hpp>

namespace roboplan {

struct FrameTask;

/// @brief Selects how the planner assigns speed/timing along the Cartesian path.
enum class CartesianSpeedMode {
  /// @brief Trace the path under bounded Cartesian velocity and acceleration.
  /// @details Builds a trapezoidal feedrate profile that ramps the Cartesian tool speed up to the
  /// commanded linear/angular maxima and back down to a stop at the path end, bounding the
  /// Cartesian linear/angular acceleration by the commanded maxima. The profile is traced with the
  /// differential-IK tracker, and the feedrate is throttled further wherever the robot would
  /// otherwise exceed its joint velocity or acceleration limits, or fall outside the path
  /// tolerance (e.g., near a singularity). The commanded speeds and accelerations therefore act as
  /// maxima, not fixed values.
  Bounded,

  /// @brief Time-optimal re-timing respecting joint velocity/acceleration limits.
  /// @details Resolves the waypoints to a joint path and hands it to a
  /// PathParameterizerTOPPRA instance using linear segments with circular blends.
  /// Tool speed will vary along the path in this mode.
  TimeOptimal,
};

/// @brief Options struct for the Cartesian path planner.
struct CartesianPlannerOptions {
  /// @brief The joint group name to plan for. Empty means the full robot.
  std::string group_name = "";

  /// @brief The output trajectory sample period (control period), in seconds.
  /// @details This is also the sample time used by the OInK solver.
  double dt = 0.01;

  /// @brief Which timing/speed strategy to use.
  CartesianSpeedMode speed_mode = CartesianSpeedMode::Bounded;

  /// @brief Maximum linear tool speed along the path, in meters/second.
  /// @details Only used in Bounded speed mode.
  double max_linear_speed = 0.1;

  /// @brief Maximum angular tool speed along the path, in radians/second.
  /// @details Only used in Bounded speed mode.
  double max_angular_speed = 0.5;

  /// @brief Maximum linear tool acceleration along the path, in meters/second^2.
  /// @details Only used in Bounded speed mode, where the tool speed is ramped up and down so
  /// the Cartesian linear acceleration stays within this bound.
  double max_linear_acceleration = 0.5;

  /// @brief Maximum angular tool acceleration along the path, in radians/second^2.
  /// @details Only used in Bounded speed mode, where the tool speed is ramped up and down so
  /// the Cartesian angular acceleration stays within this bound.
  double max_angular_acceleration = 2.5;

  /// @brief Maximum allowed position deviation from the path, in meters.
  double max_position_error = 0.005;

  /// @brief Maximum allowed orientation deviation from the path, in radians.
  double max_orientation_error = 0.01;

  /// @brief Oink FrameTask position cost weight.
  double position_cost = 1.0;

  /// @brief Oink FrameTask orientation cost weight.
  double orientation_cost = 1.0;

  /// @brief Oink FrameTask proportional gain.
  double task_gain = 1.0;

  /// @brief Oink FrameTask Levenberg-Marquardt damping.
  double lm_damping = 0.01;

  /// @brief Tikhonov regularization weight for the Oink QP Hessian.
  double regularization = 1e-6;

  /// @brief Weight of the priority-2 ConfigurationTask that regularizes redundant
  /// joints toward the seed configuration (uses only nullspace freedom).
  double config_task_weight = 0.05;

  /// @brief Scaling factor (0, 1] applied to the joint velocity limits used to bound
  /// each differential-IK step.
  double velocity_scale = 1.0;

  /// @brief Scaling factor (0, 1] applied to the joint acceleration limits.
  /// @details Used both by the TimeOptimal re-timing and by the Bounded mode's joint-acceleration
  /// throttle (which slows the feedrate wherever a step would exceed the scaled limits).
  double acceleration_scale = 1.0;

  /// @brief Acceptance tolerance (>= 1.0) for the Bounded mode's slow-down retry.
  /// @details A trace is accepted once its peak joint velocity and acceleration ratios land within
  /// this factor of the (scaled) limits; otherwise the whole motion is re-timed slower and retried.
  /// Because the Bounded mode has no hard joint-acceleration constraint and estimates acceleration
  /// by finite difference, a value > 1.0 absorbs single-sample spikes at corners/tolerance events
  /// and avoids needless extra slow-down passes. Set to 1.0 to require the peaks to land within the
  /// (scaled) limits; for extra margin below the limits use velocity_scale/acceleration_scale
  /// instead. Only used in Bounded speed mode.
  double limit_ratio_tolerance = 1.05;

  /// @brief Corner-rounding tolerance (joint-space radians) for the TimeOptimal speed
  /// mode, which times the path with TOPP-RA over a straight-segment + circular-blend geometry.
  /// Each corner is rounded by a circular arc that deviates from the sharp corner by at most
  /// this much. Larger values round corners more aggressively (faster motion, but the joint
  /// path strays further from the resolved waypoints); a value <= 0 disables blending (the
  /// trajectory stops at every waypoint).
  double toppra_blend_deviation = 0.05;

  /// @brief Gain (0, 1] for the position-limit constraint that steers each step away
  /// from the joint position limits.
  double position_limit_gain = 1.0;

  /// @brief Maximum number of feedrate-throttling attempts per control step before stalling.
  /// @details A stall is declared when the robot cannot stay within tolerance even when nearly
  /// stationary.
  int max_attempts_per_step = 16;
};

/// @brief Result of a successful Cartesian plan.
struct CartesianPlanResult {
  /// @brief The time-parameterized joint trajectory that traces the path.
  JointTrajectory trajectory;

  /// @brief The achieved Cartesian path length (meters) of the resulting motion.
  double achieved_path_length = 0.0;

  /// @brief The fraction of control steps that ran at the full commanded feedrate
  /// (i.e. were not throttled). 1.0 means the commanded speed was held throughout.
  double feedrate_efficiency = 1.0;

  /// @brief Peak |joint velocity| / velocity-limit ratio over the trajectory.
  /// Values <= 1.0 mean the joint velocity limits are respected.
  double peak_velocity_ratio = 0.0;

  /// @brief Peak |joint acceleration| / acceleration-limit ratio over the trajectory.
  /// The Bounded mode throttles the feedrate to keep this near 1.0, but it can still spike
  /// briefly at hard tolerance/joint-limit events; the TimeOptimal mode bounds it directly.
  double peak_acceleration_ratio = 0.0;
};

/// @brief User-supplied OInK solver and objectives for the Cartesian path planner.
/// @details Lets callers fully customize the differential-IK problem the planner solves at
/// each control step instead of relying on the planner's built-in setup (one FrameTask per
/// end-effector plus a nullspace ConfigurationTask, bounded by VelocityLimit and PositionLimit
/// constraints). Pass an instance to the corresponding CartesianPathPlanner constructor to
/// inject your own solver, tasks, constraints, and barriers.
///
/// The planner drives the motion by repeatedly updating each tracking FrameTask's target pose,
/// so one tracking task must be provided per end-effector in the CartesianPath. All other
/// tasks/constraints/barriers are passed to the solver unchanged on every step. The same
/// objects are reused across all plan() calls; the planner never rebuilds or mutates them
/// (other than the tracking tasks' targets), so any q_start-dependent setup (e.g. seeding a
/// ConfigurationTask) is the caller's responsibility.
struct CartesianPlannerComponents {
  /// @brief The OInK solver to use.
  /// @details Must be constructed for the same scene and joint group as the planner.
  /// Must not be null.
  std::shared_ptr<Oink> oink;

  /// @brief The FrameTasks whose target poses uses to trace the path, one per end-effector.
  /// @details Entry i tracks the frame named by path.tip_frames[i] of the CartesianPath,
  /// so the count and order must match the path's specified tip frames.
  /// Each task must be constructed against `oink` and must track the matching tip frame.
  /// The tracking tasks are prepended to the solver's task list automatically.
  /// Must be non-empty with no null entries.
  std::vector<std::shared_ptr<FrameTask>> tracking_tasks;

  /// @brief Additional tasks solved alongside the tracking tasks
  /// (e.g., a nullspace ConfigurationTask). May be empty.
  std::vector<std::shared_ptr<Task>> extra_tasks;

  /// @brief Constraints applied at every control step (e.g. VelocityLimit, PositionLimit).
  /// May be empty.
  std::vector<std::shared_ptr<Constraints>> constraints;

  /// @brief Control barrier functions applied at every control step. May be empty.
  std::vector<std::shared_ptr<Barrier>> barriers;
};

/// @brief Offline Cartesian path planner that traces a CartesianPath in joint space.
/// @details Uses the Oink optimal IK solver as a differential-IK tracker.
class CartesianPathPlanner {
public:
  /// @brief Constructor that builds the default differential-IK setup internally.
  /// @details Constructs its own OInK solver and, on each plan() call, one FrameTask per
  /// end-effector in the path plus a nullspace ConfigurationTask, bounded by VelocityLimit and
  /// PositionLimit constraints, configured from `options`.
  /// @param scene A pointer to the scene to use for planning.
  /// @param options A struct containing planner options.
  /// @throws std::runtime_error if the joint group cannot be resolved.
  CartesianPathPlanner(const std::shared_ptr<Scene> scene, const CartesianPlannerOptions& options);

  /// @brief Constructor that uses a caller-supplied OInK solver and IK objectives.
  /// @details The planner traces the path by updating each `components.tracking_tasks` target
  /// every control step and solving with the provided solver, tasks, constraints, and barriers.
  /// The
  /// Oink-related fields of `options` (costs, gains, limits, etc.) are ignored in this mode
  /// since the caller owns the objectives; timing/tolerance fields (dt, speeds, max errors,
  /// speed_mode, scales) still apply.
  /// @param scene A pointer to the scene to use for planning.
  /// @param options A struct containing planner options.
  /// @param components The caller-supplied Oink solver and IK objectives.
  /// @throws std::runtime_error if the joint group cannot be resolved, or if
  /// `components.oink` is null, or `components.tracking_tasks` is empty or contains a null entry.
  CartesianPathPlanner(const std::shared_ptr<Scene> scene, const CartesianPlannerOptions& options,
                       const CartesianPlannerComponents& components);

  /// @brief Plans a joint trajectory that traces the provided Cartesian path.
  /// @details Supports one or more end-effector frames (each entry in the path's
  /// base_frames/tip_frames/tforms is traced simultaneously by its own FrameTask).
  /// @param path The Cartesian waypoint path to trace.
  /// @param q_start The seed/start configuration, as a full model configuration
  /// (size model.nq). The robot should already be at (or near) the first waypoint.
  /// @return The plan result on success, else a string describing the error.
  tl::expected<CartesianPlanResult, std::string> plan(const CartesianPath& path,
                                                      const JointConfiguration& q_start);

private:
  /// @brief A single arc-length time-parameterized SE(3) reference built from waypoints.
  struct Reference {
    std::vector<Eigen::Matrix4d> waypoints;  ///< The SE(3) waypoints.
    std::vector<double> cumulative_times;    ///< Cumulative reference time at each waypoint.
    double total_time = 0.0;                 ///< Total reference duration (s).

    /// @brief Evaluates the reference pose at reference time s in [0, total_time].
    Eigen::Matrix4d eval(double s) const;
  };

  /// @brief Output of resolving the Cartesian path into a joint-space trace.
  struct TrackResult {
    /// @brief Committed group joint positions (one per control step).
    std::vector<Eigen::VectorXd> group_positions;
    /// @brief Per-step group joint velocities (delta_q / dt).
    std::vector<Eigen::VectorXd> group_velocities;
    /// @brief Control-step timestamps (step * dt).
    std::vector<double> times;
    /// @brief Achieved Cartesian path length (m).
    double achieved_path_length = 0.0;
    /// @brief Fraction of steps that ran at full commanded feedrate.
    double feedrate_efficiency = 1.0;
  };

  /// @brief Builds the parts of the OInK problem that do not depend on the path or seed, once,
  /// at construction time.
  /// @details Populates the reused solver-input buffers (constraints_, barriers_) and the
  /// velocity-limit verification vector. In the custom-components mode this also assembles the
  /// full task list (tracking tasks followed by extra tasks), since the caller's objectives are
  /// fixed; in the default mode the per-end-effector tasks_ are (re)built per plan() because they
  /// depend on the path's frames and the seed configuration.
  /// @throws std::runtime_error if the joint velocity limits cannot be resolved (default mode).
  void buildStaticSolverComponents();

  /// @brief Builds the arc-length reference from the path waypoints using the
  /// provided linear/angular speeds.
  Reference buildReference(const std::vector<Eigen::Matrix4d>& waypoints, double linear_speed,
                           double angular_speed) const;

  /// @brief Resolves the Cartesian path into a joint-space trace with the Oink tracker.
  /// @details Advances an arc-length reference whose feedrate follows a trapezoidal velocity
  /// profile bounded by the given Cartesian speeds and accelerations, throttling the feedrate
  /// further to stay within the path tolerance. Joint velocity and position limits are enforced
  /// per step.
  /// @param linear_acceleration,angular_acceleration Cartesian acceleration maxima for the
  /// trapezoidal feedrate profile. A non-positive value disables the profile (constant feedrate).
  tl::expected<TrackResult, std::string> trackReference(const CartesianPath& path,
                                                        const Eigen::VectorXd& q_start_full,
                                                        double linear_speed, double angular_speed,
                                                        double linear_acceleration,
                                                        double angular_acceleration);

  /// @brief Generates a trajectory that traces the path under a trapezoidal Cartesian feedrate
  /// profile: the tool speed ramps up/down within the commanded Cartesian acceleration maxima and
  /// is capped at the commanded speeds.
  /// @details If the resulting motion still exceeds the (scaled) joint velocity or acceleration
  /// limits, the whole trace is re-timed slower (commanded speeds/accelerations scaled down) and
  /// retried, so the commanded values act as maxima that are relaxed only as needed.
  tl::expected<CartesianPlanResult, std::string> planBounded(const CartesianPath& path,
                                                             const Eigen::VectorXd& q_start_full);

  /// @brief Resolves the path geometrically, then time-parameterizes it with TOPP-RA so
  /// the result respects joint velocity and acceleration limits (tool speed varies).
  tl::expected<CartesianPlanResult, std::string>
  planTimeOptimal(const CartesianPath& path, const Eigen::VectorXd& q_start_full);

  /// @brief Computes the peak |velocity|/limit and |acceleration|/limit ratios across the
  /// trajectory, so callers can see how close the result is to the joint limits.
  /// @return A pair of {peak velocity ratio, peak acceleration ratio}.
  std::pair<double, double> computePeakLimitRatios(const JointTrajectory& trajectory) const;

  /// @brief A pointer to the scene.
  std::shared_ptr<Scene> scene_;

  /// @brief The planner options.
  CartesianPlannerOptions options_;

  /// @brief The resolved joint group info.
  JointGroupInfo joint_group_info_;

  /// @brief The differential-IK solver used to resolve the Cartesian path into a joint trace.
  /// @details Constructed once for the planner's joint group (or supplied by the caller) and
  /// reused across plan() calls.
  std::shared_ptr<Oink> oink_;

  /// @brief Caller-supplied Oink objectives, set only when the components constructor is used.
  /// @details When present, trackReference() uses these instead of building the default
  /// FrameTask/ConfigurationTask/VelocityLimit/PositionLimit setup.
  std::optional<CartesianPlannerComponents> components_;

  /// @brief Reused solver task list passed to Oink::solveIk each control step.
  /// @details Assembled once at construction in the custom-components mode; rebuilt in place each
  /// plan() in the default mode (the per-end-effector FrameTasks depend on the path and seed).
  std::vector<std::shared_ptr<Task>> tasks_;

  /// @brief Constraints passed to the solver each step. Built once at construction.
  std::vector<std::shared_ptr<Constraints>> constraints_;

  /// @brief Barriers passed to the solver each step. Built once at construction.
  std::vector<std::shared_ptr<Barrier>> barriers_;

  /// @brief Joint velocity limits used to verify each committed step. Built once at construction.
  /// @details Default mode uses the scene limits scaled by options_.velocity_scale; custom mode
  /// uses the unscaled scene limits as a hard-limit sanity net.
  Eigen::VectorXd verify_v_max_;

  /// @brief Whether the per-step velocity verification runs (false if no limits are available).
  bool has_velocity_check_ = false;

  /// @brief The TOPP-RA time parameterizer, used by the TimeOptimal speed mode.
  /// @details Constructed once for the planner's joint group and reused across plan() calls.
  PathParameterizerTOPPRA toppra_;
};

}  // namespace roboplan
