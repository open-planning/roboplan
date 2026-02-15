#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <roboplan/core/scene.hpp>
#include <roboplan_simple_ik/simple_ik.hpp>

#include <modules/simple_ik.hpp>

namespace roboplan {

using namespace nanobind::literals;

void init_simple_ik(nanobind::module_& m) {

  nanobind::class_<SimpleIkOptions>(m, "SimpleIkOptions", "Options struct for simple IK solver.")
      .def(nanobind::init<const std::string&, size_t, double, size_t, double, double, double,
                          double, bool>(),
           "group_name"_a = "", "max_iters"_a = 100, "max_time"_a = 0.01, "max_restarts"_a = 2,
           "step_size"_a = 0.01, "damping"_a = 0.001, "max_linear_error_norm"_a = 0.001,
           "max_angular_error_norm"_a = 0.001, "check_collisions"_a = true)
      .def_rw("group_name", &SimpleIkOptions::group_name,
              "The joint group name to be used by the solver.")
      .def_rw("max_iters", &SimpleIkOptions::max_iters, "Max iterations for one try of the solver.")
      .def_rw("max_time", &SimpleIkOptions::max_time, "Max total computation time, in seconds.")
      .def_rw("max_restarts", &SimpleIkOptions::max_restarts,
              "Maximum number of restarts until success.")
      .def_rw("step_size", &SimpleIkOptions::step_size, "The integration step for the solver.")
      .def_rw("damping", &SimpleIkOptions::damping, "Damping value for the Jacobian pseudoinverse.")
      .def_rw("max_linear_error_norm", &SimpleIkOptions::max_linear_error_norm,
              "The maximum linear error norm, in meters.")
      .def_rw("max_angular_error_norm", &SimpleIkOptions::max_angular_error_norm,
              "The maximum angular error norm, in radians.")
      .def_rw("check_collisions", &SimpleIkOptions::check_collisions,
              "Whether to check collisions.");

  nanobind::class_<SimpleIk>(
      m, "SimpleIk", "Simple inverse kinematics (IK) solver based on the Jacobian pseudoinverse.")
      .def(nanobind::init<const std::shared_ptr<Scene>, const SimpleIkOptions&>(), "scene"_a,
           "options"_a)
      .def("solveIk",
           nanobind::overload_cast<const CartesianConfiguration&, const JointConfiguration&,
                                   JointConfiguration&>(&SimpleIk::solveIk),
           "Solves inverse kinematics (single goal).", "goal"_a, "start"_a, "solution"_a)
      .def("solveIk",
           nanobind::overload_cast<const std::vector<CartesianConfiguration>&,
                                   const JointConfiguration&, JointConfiguration&>(
               &SimpleIk::solveIk),
           "Solves inverse kinematics (multiple goal).", "goals"_a, "start"_a, "solution"_a);
}

}  // namespace roboplan
