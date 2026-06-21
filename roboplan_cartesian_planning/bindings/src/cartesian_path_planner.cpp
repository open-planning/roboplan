#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/pair.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>
#include <roboplan_cartesian_planning/cartesian_path_planner.hpp>
#include <roboplan_oink/optimal_ik.hpp>
#include <roboplan_oink/tasks/frame.hpp>

#include <modules/cartesian_path_planner.hpp>
#include <roboplan_bindings/expected.hpp>

namespace roboplan {

using namespace nanobind::literals;

void init_cartesian_path_planner(nanobind::module_& m) {
  nanobind::enum_<CartesianSpeedMode>(
      m, "CartesianSpeedMode", "Selects how the planner assigns speed/timing along the path.")
      .value("Bounded", CartesianSpeedMode::Bounded,
             "Trace the path under bounded Cartesian velocity and acceleration.")
      .value("TimeOptimal", CartesianSpeedMode::TimeOptimal,
             "Time-optimal re-timing respecting joint velocity and acceleration limits.");

  nanobind::class_<CartesianPlannerOptions>(m, "CartesianPlannerOptions",
                                            "Options for the Cartesian path planner.")
      .def(nanobind::init<std::string, double, CartesianSpeedMode, double, double, double, double,
                          double, double, double, double, double, double, double, double, double,
                          double, double, double, double, int>(),
           "group_name"_a = "", "dt"_a = 0.01, "speed_mode"_a = CartesianSpeedMode::Bounded,
           "max_linear_speed"_a = 0.1, "max_angular_speed"_a = 0.5,
           "max_linear_acceleration"_a = 0.5, "max_angular_acceleration"_a = 2.5,
           "max_position_error"_a = 0.005, "max_orientation_error"_a = 0.01,
           "position_cost"_a = 1.0, "orientation_cost"_a = 1.0, "task_gain"_a = 1.0,
           "lm_damping"_a = 0.01, "regularization"_a = 1e-6, "config_task_weight"_a = 0.05,
           "velocity_scale"_a = 1.0, "acceleration_scale"_a = 1.0, "limit_ratio_tolerance"_a = 1.05,
           "toppra_blend_deviation"_a = 0.05, "position_limit_gain"_a = 1.0,
           "max_attempts_per_step"_a = 16)
      .def_rw("group_name", &CartesianPlannerOptions::group_name, "Joint group name.")
      .def_rw("dt", &CartesianPlannerOptions::dt, "Output trajectory sample period (s).")
      .def_rw("speed_mode", &CartesianPlannerOptions::speed_mode, "Speed/timing strategy.")
      .def_rw("max_linear_speed", &CartesianPlannerOptions::max_linear_speed,
              "Maximum linear tool speed (m/s).")
      .def_rw("max_angular_speed", &CartesianPlannerOptions::max_angular_speed,
              "Maximum angular tool speed (rad/s).")
      .def_rw("max_linear_acceleration", &CartesianPlannerOptions::max_linear_acceleration,
              "Maximum linear tool acceleration (m/s^2), Bounded mode.")
      .def_rw("max_angular_acceleration", &CartesianPlannerOptions::max_angular_acceleration,
              "Maximum angular tool acceleration (rad/s^2), Bounded mode.")
      .def_rw("max_position_error", &CartesianPlannerOptions::max_position_error,
              "Maximum position deviation from the path (m).")
      .def_rw("max_orientation_error", &CartesianPlannerOptions::max_orientation_error,
              "Maximum orientation deviation from the path (rad).")
      .def_rw("position_cost", &CartesianPlannerOptions::position_cost,
              "Oink frame task position cost.")
      .def_rw("orientation_cost", &CartesianPlannerOptions::orientation_cost,
              "Oink frame task orientation cost.")
      .def_rw("task_gain", &CartesianPlannerOptions::task_gain, "Oink frame task gain.")
      .def_rw("lm_damping", &CartesianPlannerOptions::lm_damping,
              "Oink frame task Levenberg-Marquardt damping.")
      .def_rw("regularization", &CartesianPlannerOptions::regularization,
              "Tikhonov regularization for the Oink QP.")
      .def_rw("config_task_weight", &CartesianPlannerOptions::config_task_weight,
              "Weight of the nullspace configuration-regularization task.")
      .def_rw("velocity_scale", &CartesianPlannerOptions::velocity_scale,
              "Scaling factor for joint velocity limits.")
      .def_rw("acceleration_scale", &CartesianPlannerOptions::acceleration_scale,
              "Scaling factor for joint acceleration limits (TimeOptimal re-timing and Bounded "
              "joint-acceleration throttle).")
      .def_rw("limit_ratio_tolerance", &CartesianPlannerOptions::limit_ratio_tolerance,
              "Acceptance tolerance (>= 1.0) for the Bounded mode's slow-down retry; the peak "
              "velocity/acceleration ratios may exceed the (scaled) limits by up to this factor.")
      .def_rw("toppra_blend_deviation", &CartesianPlannerOptions::toppra_blend_deviation,
              "Corner-rounding tolerance (rad) for the TimeOptimal mode's TOPP-RA line+blend "
              "geometry.")
      .def_rw("position_limit_gain", &CartesianPlannerOptions::position_limit_gain,
              "Gain for the joint position-limit constraint.")
      .def_rw("max_attempts_per_step", &CartesianPlannerOptions::max_attempts_per_step,
              "Maximum feedrate-throttling attempts per control step.");

  nanobind::class_<CartesianPlannerComponents>(
      m, "CartesianPlannerComponents",
      "Caller-supplied OInK solver and IK objectives for the Cartesian path planner.")
      .def(nanobind::init<>())
      .def_rw("oink", &CartesianPlannerComponents::oink, "The OInK solver to use.")
      .def_rw("tracking_tasks", &CartesianPlannerComponents::tracking_tasks,
              "FrameTasks that the planner updates each step, one per end-effector "
              "(ordered to match the path's tip frames).")
      .def_rw("extra_tasks", &CartesianPlannerComponents::extra_tasks,
              "Additional tasks solved alongside the tracking tasks.")
      .def_rw("constraints", &CartesianPlannerComponents::constraints,
              "Constraints applied at every control step.")
      .def_rw("barriers", &CartesianPlannerComponents::barriers,
              "Control barrier functions applied at every control step.");

  nanobind::class_<CartesianPathPlanner>(
      m, "CartesianPathPlanner",
      "Offline Cartesian path planner that traces a CartesianPath in joint space using Oink.")
      .def(nanobind::init<const std::shared_ptr<Scene>, const CartesianPlannerOptions&>(),
           "scene"_a, "options"_a)
      .def(nanobind::init<const std::shared_ptr<Scene>, const CartesianPlannerOptions&,
                          const CartesianPlannerComponents&>(),
           "scene"_a, "options"_a, "components"_a,
           "Constructs a planner that uses a caller-supplied OInK solver and objectives.")
      .def("plan", unwrap_expected(&CartesianPathPlanner::plan),
           "Plans a joint trajectory that traces the provided Cartesian path.", "path"_a,
           "q_start"_a)
      .def("compute_peak_limit_ratios", &CartesianPathPlanner::computePeakLimitRatios,
           "Computes the (peak velocity / limit, peak acceleration / limit) ratios over a "
           "trajectory.",
           "trajectory"_a)
      .def("compute_achieved_path_length", &CartesianPathPlanner::computeAchievedPathLength,
           "Computes the achieved Cartesian path length (m) traced by the path's tip frames.",
           "trajectory"_a, "path"_a);
}

}  // namespace roboplan
