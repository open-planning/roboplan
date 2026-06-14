#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <roboplan/core/scene.hpp>
#include <roboplan/core/types.hpp>
#include <roboplan_cartesian_planning/cartesian_path_planner.hpp>

#include <modules/cartesian_path_planner.hpp>
#include <roboplan_bindings/expected.hpp>

namespace roboplan {

using namespace nanobind::literals;

void init_cartesian_path_planner(nanobind::module_& m) {
  nanobind::enum_<CartesianSpeedMode>(
      m, "CartesianSpeedMode", "Selects how the planner assigns speed/timing along the path.")
      .value("Constant", CartesianSpeedMode::Constant,
             "Trace the path at a (roughly) constant Cartesian tool speed.")
      .value("Toppra", CartesianSpeedMode::Toppra,
             "Time-optimal re-timing respecting joint limits (not yet implemented).");

  nanobind::class_<CartesianPlannerOptions>(m, "CartesianPlannerOptions",
                                            "Options for the Cartesian path planner.")
      .def(nanobind::init<std::string, double, double, double, double, double, CartesianSpeedMode,
                          double, double, double, double, double, double, double, double, double,
                          double, int>(),
           "group_name"_a = "", "dt"_a = 0.01, "linear_speed"_a = 0.1, "angular_speed"_a = 0.5,
           "max_position_error"_a = 0.005, "max_orientation_error"_a = 0.01,
           "speed_mode"_a = CartesianSpeedMode::Constant, "position_cost"_a = 1.0,
           "orientation_cost"_a = 1.0, "task_gain"_a = 1.0, "lm_damping"_a = 0.01,
           "regularization"_a = 1e-6, "config_task_weight"_a = 0.05, "velocity_scale"_a = 1.0,
           "acceleration_scale"_a = 1.0, "toppra_blend_deviation"_a = 0.05,
           "position_limit_gain"_a = 1.0, "max_attempts_per_step"_a = 16)
      .def_rw("group_name", &CartesianPlannerOptions::group_name, "Joint group name.")
      .def_rw("dt", &CartesianPlannerOptions::dt, "Output trajectory sample period (s).")
      .def_rw("linear_speed", &CartesianPlannerOptions::linear_speed,
              "Commanded linear tool speed (m/s).")
      .def_rw("angular_speed", &CartesianPlannerOptions::angular_speed,
              "Commanded angular tool speed (rad/s).")
      .def_rw("max_position_error", &CartesianPlannerOptions::max_position_error,
              "Maximum position deviation from the path (m).")
      .def_rw("max_orientation_error", &CartesianPlannerOptions::max_orientation_error,
              "Maximum orientation deviation from the path (rad).")
      .def_rw("speed_mode", &CartesianPlannerOptions::speed_mode, "Speed/timing strategy.")
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
              "Scaling factor for joint acceleration limits (Toppra mode).")
      .def_rw("toppra_blend_deviation", &CartesianPlannerOptions::toppra_blend_deviation,
              "Corner-rounding tolerance (rad) for the Toppra line+blend geometry.")
      .def_rw("position_limit_gain", &CartesianPlannerOptions::position_limit_gain,
              "Gain for the joint position-limit constraint.")
      .def_rw("max_attempts_per_step", &CartesianPlannerOptions::max_attempts_per_step,
              "Maximum feedrate-throttling attempts per control step.");

  nanobind::class_<CartesianPlanResult>(m, "CartesianPlanResult",
                                        "Result of a successful Cartesian plan.")
      .def_ro("trajectory", &CartesianPlanResult::trajectory,
              "The joint trajectory that traces the path.")
      .def_ro("achieved_path_length", &CartesianPlanResult::achieved_path_length,
              "Achieved Cartesian path length (m).")
      .def_ro("feedrate_efficiency", &CartesianPlanResult::feedrate_efficiency,
              "Fraction of control steps run at the full commanded feedrate.")
      .def_ro("peak_velocity_ratio", &CartesianPlanResult::peak_velocity_ratio,
              "Peak |joint velocity| / velocity-limit ratio over the trajectory.")
      .def_ro("peak_acceleration_ratio", &CartesianPlanResult::peak_acceleration_ratio,
              "Peak |joint acceleration| / acceleration-limit ratio over the trajectory.");

  nanobind::class_<CartesianPathPlanner>(
      m, "CartesianPathPlanner",
      "Offline Cartesian path planner that traces a CartesianPath in joint space using Oink.")
      .def(nanobind::init<const std::shared_ptr<Scene>, const CartesianPlannerOptions&>(),
           "scene"_a, "options"_a)
      .def("plan", unwrap_expected(&CartesianPathPlanner::plan),
           "Plans a joint trajectory that traces the provided Cartesian path.", "path"_a,
           "q_start"_a);
}

}  // namespace roboplan
