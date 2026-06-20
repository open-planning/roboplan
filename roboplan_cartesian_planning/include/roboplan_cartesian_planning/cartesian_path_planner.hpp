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

  /// @brief Validates the mode-independent option values (dt, tolerances, scales).
  /// @details Mode-specific options (e.g. the Bounded mode's commanded speeds/accelerations) are
  /// validated where they are used.
  /// @return Nothing on success, else a string describing the first invalid option.
  tl::expected<void, std::string> validate() const;
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
  /// @return The time-parameterized joint trajectory on success, else a string describing the
  /// error. Quality metrics (peak limit ratios, achieved path length) are not bundled in; compute
  /// them on demand from the returned trajectory with computePeakLimitRatios() /
  /// computeAchievedPathLength().
  tl::expected<JointTrajectory, std::string> plan(const CartesianPath& path,
                                                  const JointConfiguration& q_start);

  /// @brief Computes the peak |velocity|/limit and |acceleration|/limit ratios across the
  /// trajectory, so callers can see how close the result is to the joint limits.
  /// @param trajectory The joint trajectory to evaluate (e.g. the output of plan()).
  /// @return A pair of {peak velocity ratio, peak acceleration ratio}.
  /// Values <= 1.0 mean the respective joint limits are respected.
  std::pair<double, double> computePeakLimitRatios(const JointTrajectory& trajectory) const;

  /// @brief Computes the achieved Cartesian path length (meters) traced by the path's tip frames.
  /// @details Re-runs forward kinematics over the trajectory and sums the world-frame translation
  /// travelled by every tip frame in `path`. Joints outside the planning group are held at the
  /// scene's current state; because that contributes only a constant rigid offset, it cancels in
  /// the per-step differences and does not affect the result.
  /// @param trajectory The joint trajectory to evaluate (e.g. the output of plan()).
  /// @param path The Cartesian path whose tip frames were traced.
  /// @return The summed Cartesian path length (meters) across all tip frames.
  double computeAchievedPathLength(const JointTrajectory& trajectory,
                                   const CartesianPath& path) const;

private:
  /// @brief Per-end-effector tracking state for one frame of the path: the arc-length-timed
  /// waypoint reference plus the world transform, tip frame, and task used to trace it. Held
  /// together so the servo loop indexes a single vector instead of several parallel ones.
  struct FrameTrack {
    std::vector<Eigen::Matrix4d> waypoints;  ///< Base-relative SE(3) waypoints.
    std::vector<double> cumulative_times;    ///< Cumulative reference time at each waypoint (s).
    double total_time = 0.0;                 ///< Total reference duration (s).
    Eigen::Matrix4d world_T_base = Eigen::Matrix4d::Identity();  ///< Fixed world<-base transform.
    std::shared_ptr<FrameTask> task;  ///< Tracking task whose target is retargeted each step.
    std::string tip_frame;            ///< Name of the tip frame traced (path.tip_frames[f]).

    /// @brief Base-relative reference pose at reference time `s` in [0, total_time].
    Eigen::Matrix4d eval(double s) const;
    /// @brief World-frame target pose of this frame at reference time `s`.
    Eigen::Matrix4d target(double s) const { return world_T_base * eval(s); }
  };

  /// @brief Trapezoidal feedrate profile: a ceiling on the feedrate (ds/dt) over a uniform
  /// reference-time grid that bounds the Cartesian acceleration and forces a stop at the path end.
  struct FeedrateProfile {
    bool enabled = false;         ///< When false, sample() returns 1.0 (constant full feedrate).
    std::vector<double> ceiling;  ///< Feedrate ceiling at each grid knot.
    double ds = 0.0;              ///< Reference-time spacing between knots (s).
    double s_ddot_max = 0.0;      ///< Maximum feedrate acceleration (1/s).
    size_t num_corner_knots = 0;  ///< Number of corner caps applied (used to size the loop cap).

    /// @brief Samples the ceiling at reference time `s` (linear interpolation; 1.0 if disabled).
    double sample(double s) const;
  };

  /// @brief Builds the parts of the OInK problem that do not depend on the path or seed, once,
  /// at construction time.
  /// @details Populates the reused solver-input buffers (constraints_, barriers_) and the
  /// velocity-limit verification vector. When `components` is provided this also assembles the
  /// full task list (tracking tasks followed by extra tasks) and caches the tracking tasks
  /// (tracking_tasks_) for per-plan() wiring, since the caller's objectives are fixed; otherwise
  /// the per-end-effector tasks_ are (re)built per plan() because they depend on the path's frames
  /// and the seed configuration.
  /// @param components The caller-supplied objectives, or std::nullopt for the default setup.
  /// @throws std::runtime_error if the joint velocity limits cannot be resolved (default setup).
  void buildStaticSolverComponents(const std::optional<CartesianPlannerComponents>& components);

  /// @brief Builds the per-end-effector tracking state (arc-length-timed waypoints, world<-base
  /// transform, tip frame, and tracking task) for every frame in the path, and wires the tracking
  /// tasks into the reused solver task list (tasks_).
  /// @details In the custom-components mode the tasks are the caller's (validated to match the
  /// path's tip-frame order); otherwise one priority-1 FrameTask per end-effector plus a
  /// priority-2 nullspace ConfigurationTask are (re)built from the seed configuration.
  tl::expected<std::vector<FrameTrack>, std::string>
  buildFrameTracks(const CartesianPath& path, const Eigen::VectorXd& q_start_full,
                   double linear_speed, double angular_speed);

  /// @brief Precomputes the trapezoidal feedrate profile for the given references.
  /// @details Bounds the feedrate acceleration so the Cartesian acceleration stays within the
  /// commanded maxima, caps the feedrate at each corner (see cornerFeedrateCap), and runs a
  /// backward pass so the feedrate can always decelerate to each corner cap and stop at the end.
  /// A non-positive acceleration returns a disabled profile (constant full feedrate).
  FeedrateProfile buildFeedrateProfile(const std::vector<FrameTrack>& tracks, double total_time,
                                       double linear_speed, double angular_speed,
                                       double linear_acceleration,
                                       double angular_acceleration) const;

  /// @brief Runs one differential-IK step that retargets every frame to its pose at reference
  /// time `s` from the committed configuration `q`. Writes the candidate configuration, the group
  /// step `delta_q`, and the worst-case (max over frames) FK pose error. Does not commit.
  tl::expected<void, std::string> solveStep(const std::vector<FrameTrack>& tracks,
                                            const Eigen::VectorXd& q, double s,
                                            Eigen::VectorXd& q_candidate, Eigen::VectorXd& delta_q,
                                            double& position_error, double& orientation_error);

  /// @brief Drives every frame onto its first waypoint within tolerance, updating `q` in place.
  tl::expected<void, std::string> convergeToStart(const std::vector<FrameTrack>& tracks,
                                                  Eigen::VectorXd& q);

  /// @brief Runs the timed servo loop from the converged start `q`, advancing the reference under
  /// the feedrate profile and throttling further to stay within the path tolerance and joint
  /// limits.
  /// @return A trajectory with positions, velocities, and times populated. The accelerations are
  /// left for the plan*() caller to fill.
  tl::expected<JointTrajectory, std::string> runServoLoop(const std::vector<FrameTrack>& tracks,
                                                          const FeedrateProfile& profile,
                                                          double total_time, Eigen::VectorXd& q);

  /// @brief Resolves the Cartesian path into a joint-space trace with the Oink tracker.
  /// @details Orchestrates buildFrameTracks -> buildFeedrateProfile -> convergeToStart ->
  /// runServoLoop. The reference advances at the commanded Cartesian speeds, ramped under a
  /// trapezoidal feedrate profile bounded by the accelerations, and throttled to stay within the
  /// path tolerance. Joint velocity and position limits are enforced per step.
  /// @param linear_acceleration,angular_acceleration Cartesian acceleration maxima for the
  /// trapezoidal feedrate profile. A non-positive value disables the profile (constant feedrate).
  /// @return The trajectory from runServoLoop (positions, velocities, times; see there).
  tl::expected<JointTrajectory, std::string>
  trackReference(const CartesianPath& path, const Eigen::VectorXd& q_start_full,
                 double linear_speed, double angular_speed, double linear_acceleration,
                 double angular_acceleration);

  /// @brief Fills a trajectory's accelerations by backward finite difference of the velocities,
  /// zeroing the (at-rest) boundary accelerations.
  void fillAccelerations(JointTrajectory& trajectory) const;

  /// @brief Generates a trajectory that traces the path under a trapezoidal Cartesian feedrate
  /// profile: the tool speed ramps up/down within the commanded Cartesian acceleration maxima and
  /// is capped at the commanded speeds.
  /// @details If the resulting motion still exceeds the (scaled) joint velocity or acceleration
  /// limits, the whole trace is re-timed slower (commanded speeds/accelerations scaled down) and
  /// retried, so the commanded values act as maxima that are relaxed only as needed.
  tl::expected<JointTrajectory, std::string> planBounded(const CartesianPath& path,
                                                         const Eigen::VectorXd& q_start_full);

  /// @brief Resolves the path geometrically, then time-parameterizes it with TOPP-RA so
  /// the result respects joint velocity and acceleration limits (tool speed varies).
  tl::expected<JointTrajectory, std::string> planTimeOptimal(const CartesianPath& path,
                                                             const Eigen::VectorXd& q_start_full);

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

  /// @brief Caller-supplied tracking tasks, non-empty only when the components constructor is used.
  /// @details Cached at construction so buildFrameTracks() can wire each FrameTrack to its task on
  /// every plan() call; a non-empty value also marks the custom-components mode (in which the
  /// default FrameTask/ConfigurationTask/VelocityLimit/PositionLimit setup is bypassed). The other
  /// caller-supplied objectives are consumed into oink_/tasks_/constraints_/barriers_ at
  /// construction, so they do not need to be retained.
  std::vector<std::shared_ptr<FrameTask>> tracking_tasks_;

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
