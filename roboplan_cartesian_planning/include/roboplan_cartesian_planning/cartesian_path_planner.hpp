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
  /// @details Resolves and times the path exactly as TimeOptimal does, then slows the whole motion
  /// uniformly until the tool speed and acceleration are within the commanded Cartesian maxima.
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

  /// @brief The output trajectory sample period, in seconds.
  /// @details Also bounds each differential-IK step to dt * velocity limit while resolving the
  /// path, which only sets how far a single IK iteration may move.
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

  /// @brief Scaling factor (0, 1] applied to joint velocity limits when timing the path.
  /// @details This scale is applied by the re-timing step, in both speed modes.
  double velocity_scale = 1.0;

  /// @brief Scaling factor (0, 1] applied to joint acceleration limits when timing the path.
  /// @details This scale is applied by the re-timing step, in both speed modes.
  double acceleration_scale = 1.0;

  /// @brief Corner-rounding tolerance, in joint-space units, for the straight-segment +
  /// circular-blend geometry TOPP-RA times the path over.
  /// @details Each corner is replaced by a circular arc that strays from it by at most this much.
  /// Larger values round corners more aggressively, which means smoother motion with less stops,
  /// but the joint path strays further from the resolved waypoints. The tolerance is a joint-space
  /// bound, so the tool deviation it produces varies with the arm and is not checked against
  /// max_position_error. A value <= 0 disables blending, so the trajectory stops at every waypoint.
  double toppra_blend_deviation = 0.0025;

  /// @brief Gain (0, 1] for the position-limit constraint that steers each step away
  /// from the joint position limits.
  double position_limit_gain = 1.0;

  /// @brief Validates the mode-independent option values (dt, tolerances, scales).
  /// @details Mode-specific options (e.g. the Bounded mode's commanded speeds/accelerations) are
  /// validated where they are used.
  /// @return Nothing on success, else a string describing the first invalid option.
  tl::expected<void, std::string> validate() const;
};

/// @brief User-supplied OInK solver and objectives for the Cartesian path planner.
/// @details Lets callers fully customize the differential-IK problem the planner solves
/// instead of relying on the planner's built-in setup (one FrameTask per end effector plus
/// a nullspace ConfigurationTask, bounded by VelocityLimit and PositionLimit constraints).
/// The planner drives the motion by repeatedly updating each tracking FrameTask's target pose,
/// so one tracking task must be provided per end-effector in the CartesianPath.
/// All other tasks/constraints/barriers are passed to the solver unchanged on every step.
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
/// Note that this planner uses the scene passed into it in a way that is not thread-safe; when
/// using it, the robot must not be moving and no other planning steps must be running concurrently.
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
  /// @return The timed joint trajectory on success, else a string describing the error.
  tl::expected<JointTrajectory, std::string> plan(const CartesianPath& path,
                                                  const JointConfiguration& q_start);

  /// @brief Computes the peak |velocity|/limit and |acceleration|/limit ratios across the
  /// trajectory, so callers can see how close the result is to the joint limits.
  /// @param trajectory The joint trajectory to evaluate (e.g., the output of plan()).
  /// @return A pair of {peak velocity ratio, peak acceleration ratio}.
  /// Values <= 1.0 mean the respective joint limits are respected.
  std::pair<double, double> computePeakLimitRatios(const JointTrajectory& trajectory) const;

  /// @brief Computes the achieved Cartesian path length (meters) traced by the path's tip frames.
  /// @param trajectory The joint trajectory to evaluate (e.g. the output of plan()).
  /// @param path The Cartesian path whose tip frames were traced.
  /// @return The summed Cartesian path length (meters) across all tip frames.
  double computeAchievedPathLength(const JointTrajectory& trajectory,
                                   const CartesianPath& path) const;

private:
  /// @brief Per-end-effector geometric reference for one frame of the path: the base-relative
  /// waypoints plus the world transform, tip frame, and task used to resolve it.
  /// @details Parameterized by a normalized path parameter `s` in [0, 1] that carries no timing.
  /// All frames share the same `s`, so they progress through their own waypoints together.
  struct FrameReference {
    std::vector<Eigen::Matrix4d> waypoints;  ///< Base-relative SE(3) waypoints.
    std::vector<double> cumulative;          ///< Normalized progress in [0, 1] at each waypoint.
    double linear_length = 0.0;              ///< Total tool travel along the reference (m).
    double angular_length = 0.0;             ///< Total tool rotation along the reference (rad).
    Eigen::Matrix4d world_T_base = Eigen::Matrix4d::Identity();  ///< Fixed world<-base transform.
    std::shared_ptr<FrameTask> task;  ///< Tracking task whose target is retargeted per sample.
    std::string tip_frame;            ///< Name of the tip frame traced (path.tip_frames[f]).

    /// @brief Base-relative reference pose at path parameter `s` in [0, 1].
    Eigen::Matrix4d eval(double s) const;
    /// @brief World-frame target pose of this frame at path parameter `s`.
    Eigen::Matrix4d target(double s) const { return world_T_base * eval(s); }
  };

  /// @brief Peak Cartesian rates observed over a timed trajectory, across all tracked tips.
  struct CartesianPeaks {
    double linear_speed = 0.0;          ///< Peak tool speed (m/s).
    double angular_speed = 0.0;         ///< Peak tool angular speed (rad/s).
    double linear_acceleration = 0.0;   ///< Peak tool acceleration (m/s^2).
    double angular_acceleration = 0.0;  ///< Peak tool angular acceleration (rad/s^2).
  };

  /// @brief Builds the parts of the OInK problem that do not depend on the path or seed, once,
  /// at construction time.
  /// @details Populates the reused solver-input buffers (constraints_, barriers_) and the group
  /// velocity limits. When `components` is provided this also assembles the full task list
  /// (tracking tasks followed by extra tasks) and caches the tracking tasks (tracking_tasks_) for
  /// per-plan() wiring, since the caller's objectives are fixed; otherwise the per-end-effector
  /// tasks_ are (re)built per plan() because they depend on the path's frames and the seed.
  /// @param components The caller-supplied objectives, or std::nullopt for the default setup.
  /// @throws std::runtime_error if the joint velocity limits cannot be resolved (default setup).
  void buildStaticSolverComponents(const std::optional<CartesianPlannerComponents>& components);

  /// @brief Builds the per-end-effector geometric reference for every frame in the path, and wires
  /// the tracking tasks into the reused solver task list (tasks_).
  /// @details In the custom-components mode the tasks are the caller's (validated to match the
  /// path's tip-frame order); otherwise one priority-1 FrameTask per end-effector plus a
  /// priority-2 nullspace ConfigurationTask are (re)built from the seed configuration.
  tl::expected<std::vector<FrameReference>, std::string>
  buildFrameReferences(const CartesianPath& path, const Eigen::VectorXd& q_start_full);

  /// @brief Runs one differential-IK step that retargets every frame to its pose at path parameter
  /// `s` from the configuration `q`. Writes the candidate configuration, the group step `delta_q`,
  /// and the worst-case (max over frames) FK pose error. Does not commit.
  tl::expected<void, std::string> solveStep(const std::vector<FrameReference>& references,
                                            const Eigen::VectorXd& q, double s,
                                            Eigen::VectorXd& q_candidate, Eigen::VectorXd& delta_q,
                                            double& position_error, double& orientation_error);

  /// @brief Iterates the differential IK at a fixed `s` until every frame is within the pose
  /// tolerance, updating `q` in place.
  /// @details This is what keeps the resolved path on the Cartesian path: each sample is solved to
  /// convergence rather than tracked by a single step, so the robot never trails the reference and
  /// no throttling or lag-recovery is needed.
  /// The tolerance here should be far tighter than the path tolerance: see kIkConvergenceFraction.
  /// @param position_tolerance Residual position error a sample must reach.
  /// @param orientation_tolerance Residual orientation error a sample must reach.
  tl::expected<void, std::string> converge(const std::vector<FrameReference>& references, double s,
                                           double position_tolerance, double orientation_tolerance,
                                           Eigen::VectorXd& q);

  /// @brief Stage one: resolves the Cartesian path into a purely geometric joint path.
  /// @details Samples the path by arc length at a density set by the pose tolerance, solving IK to
  /// convergence at each sample and seeding from the previous solution. The result carries no
  /// timing at all; stage two assigns it.
  /// @return The resolved group joint positions, or a description of where and why IK failed.
  tl::expected<std::vector<Eigen::VectorXd>, std::string>
  resolvePath(const CartesianPath& path, const Eigen::VectorXd& q_start_full);

  /// @brief Stage two: time-parameterizes a resolved joint path with TOPP-RA under the given
  /// scales, decimating it to its shape-carrying waypoints first and retrying nearby
  /// discretizations if the solver rejects one.
  tl::expected<JointTrajectory, std::string>
  timeParameterize(const std::vector<Eigen::VectorXd>& resolved, double velocity_scale,
                   double acceleration_scale);

  /// @brief Peak Cartesian tool rates over a timed trajectory, by finite difference of the tip
  /// poses at the trajectory's sample period.
  CartesianPeaks computeCartesianPeaks(const JointTrajectory& trajectory,
                                       const CartesianPath& path) const;

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
  /// @details Cached at construction so buildFrameReferences() can wire each reference to its task
  /// on every plan() call; a non-empty value also marks the custom-components mode (in which the
  /// default OInK setup is bypassed). The other caller-supplied objectives are consumed into
  /// oink_/tasks_/constraints_/barriers_ at construction, so they do not need to be retained.
  std::vector<std::shared_ptr<FrameTask>> tracking_tasks_;

  /// @brief Reused solver task list passed to Oink::solveIk each control step.
  /// @details Assembled once at construction in the custom-components mode; rebuilt in place each
  /// plan() in the default mode (the per-end-effector FrameTasks depend on the path and seed).
  std::vector<std::shared_ptr<Task>> tasks_;

  /// @brief Constraints passed to the solver each step. Built once at construction.
  std::vector<std::shared_ptr<Constraints>> constraints_;

  /// @brief Barriers passed to the solver each step. Built once at construction.
  std::vector<std::shared_ptr<Barrier>> barriers_;

  /// @brief The TOPP-RA time parameterizer, used by both speed modes.
  /// @details Constructed once for the planner's joint group and reused across plan() calls.
  PathParameterizerTOPPRA toppra_;
};

}  // namespace roboplan
