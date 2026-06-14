#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <tl/expected.hpp>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>
#include <roboplan_oink/optimal_ik.hpp>
#include <roboplan_toppra/toppra.hpp>

namespace roboplan {

/// @brief Selects how the planner assigns speed/timing along the Cartesian path.
enum class CartesianSpeedMode {
  /// @brief Trace the path at a (roughly) constant Cartesian tool speed.
  /// @details The reference advances at the commanded linear/angular speed wherever
  /// feasible, and is throttled below it only where the robot cannot otherwise stay
  /// within tolerance (e.g. near a singularity or a joint velocity limit).
  ConstantCartesianSpeed,

  /// @brief Time-optimal re-timing respecting joint velocity/acceleration limits.
  /// @details Not yet implemented. Reserved for resolving the waypoints to a joint
  /// path and handing it to roboplan_toppra's PathParameterizerTOPPRA. Tool speed
  /// will vary along the path in this mode.
  TimeOptimalToppra,
};

/// @brief Options struct for the Cartesian path planner.
struct CartesianPlannerOptions {
  /// @brief The joint group name to plan for. Empty means the full robot.
  std::string group_name = "";

  /// @brief The output trajectory sample period (control period), in seconds.
  double dt = 0.01;

  /// @brief Commanded linear tool speed along the path, in meters/second.
  double linear_speed = 0.1;

  /// @brief Commanded angular tool speed along the path, in radians/second.
  double angular_speed = 0.5;

  /// @brief Maximum allowed position deviation from the path, in meters.
  double max_position_error = 0.005;

  /// @brief Maximum allowed orientation deviation from the path, in radians.
  double max_orientation_error = 0.01;

  /// @brief Which timing/speed strategy to use.
  CartesianSpeedMode speed_mode = CartesianSpeedMode::ConstantCartesianSpeed;

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

  /// @brief Scaling factor (0, 1] applied to the joint acceleration limits. Only used by
  /// the TimeOptimalToppra speed mode.
  double acceleration_scale = 1.0;

  /// @brief Corner-rounding tolerance (joint-space radians) for the TimeOptimalToppra speed
  /// mode, which times the path with TOPP-RA over a straight-segment + circular-blend geometry.
  /// Each corner is rounded by a circular arc that deviates from the sharp corner by at most
  /// this much. Larger values round corners more aggressively (faster motion, but the joint
  /// path strays further from the resolved waypoints); a value <= 0 disables blending (the
  /// trajectory stops at every waypoint).
  double toppra_blend_deviation = 0.05;

  /// @brief Gain (0, 1] for the position-limit constraint that steers each step away
  /// from the joint position limits.
  double position_limit_gain = 1.0;

  /// @brief Maximum number of feedrate-throttling attempts per control step before
  /// declaring a stall (the robot cannot stay within tolerance even when nearly
  /// stationary).
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
  /// The ConstantCartesianSpeed mode is velocity-level and does not bound acceleration,
  /// so this can exceed 1.0; use the TimeOptimalToppra mode to keep it within limits.
  double peak_acceleration_ratio = 0.0;
};

/// @brief Offline Cartesian path planner that traces a CartesianPath in joint space.
/// @details Uses the Oink optimal IK solver as a differential-IK tracker. See the
/// package README for the algorithm.
class CartesianPathPlanner {
public:
  /// @brief Constructor.
  /// @param scene A pointer to the scene to use for planning.
  /// @param options A struct containing planner options.
  /// @throws std::runtime_error if the joint group cannot be resolved.
  CartesianPathPlanner(const std::shared_ptr<Scene> scene, const CartesianPlannerOptions& options);

  /// @brief Plans a joint trajectory that traces the provided Cartesian path.
  /// @details v1 supports a single end-effector frame (one entry in the path's
  /// base_frames/tip_frames/tforms).
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

  /// @brief Builds the arc-length reference from the path waypoints using the
  /// provided linear/angular speeds.
  Reference buildReference(const std::vector<Eigen::Matrix4d>& waypoints, double linear_speed,
                           double angular_speed) const;

  /// @brief Resolves the Cartesian path into a joint-space trace with the Oink tracker,
  /// advancing the reference at the given Cartesian speeds and throttling the feedrate to
  /// stay within the path tolerance. Velocity and position limits are enforced per step.
  tl::expected<TrackResult, std::string> trackReference(const CartesianPath& path,
                                                        const Eigen::VectorXd& q_start_full,
                                                        double linear_speed, double angular_speed);

  /// @brief Generates a trajectory that traces the path at a (throttled) constant
  /// Cartesian speed. This is a velocity-level trace: it respects joint velocity and
  /// position limits but not acceleration/jerk limits (see peak ratios in the result).
  tl::expected<CartesianPlanResult, std::string>
  planConstantSpeed(const CartesianPath& path, const Eigen::VectorXd& q_start_full);

  /// @brief Resolves the path geometrically, then time-parameterizes it with TOPP-RA so
  /// the result respects joint velocity and acceleration limits (tool speed varies).
  tl::expected<CartesianPlanResult, std::string> planToppra(const CartesianPath& path,
                                                            const Eigen::VectorXd& q_start_full);

  /// @brief Computes the peak |velocity|/limit and |acceleration|/limit ratios across the
  /// trajectory, so callers can see how close the result is to the joint limits.
  void computePeakLimitRatios(const JointTrajectory& trajectory, double& velocity_ratio,
                              double& acceleration_ratio) const;

  /// @brief A pointer to the scene.
  std::shared_ptr<Scene> scene_;

  /// @brief The planner options.
  CartesianPlannerOptions options_;

  /// @brief The resolved joint group info.
  JointGroupInfo joint_group_info_;

  /// @brief The differential-IK solver used to resolve the Cartesian path into a joint trace.
  /// @details Constructed once for the planner's joint group and reused across plan() calls.
  Oink oink_;

  /// @brief The TOPP-RA time parameterizer, used by the TimeOptimalToppra speed mode.
  /// @details Constructed once for the planner's joint group and reused across plan() calls.
  PathParameterizerTOPPRA toppra_;
};

}  // namespace roboplan
