#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include <roboplan/core/scene.hpp>
#include <roboplan_simple_ik/simple_ik.hpp>

#include <modules/simple_ik.hpp>

namespace roboplan {

using namespace nanobind::literals;

void init_simple_ik(nanobind::module_& m) {

  nanobind::class_<SimpleIkOptions>(m, "SimpleIkOptions", "Options struct for simple IK solver.")
      .def(nanobind::init<>())  // Default constructor
      .def_rw("group_name", &SimpleIkOptions::group_name,
              "The joint group name to be used by the solver.")
      .def_rw("max_iters", &SimpleIkOptions::max_iters, "Max iterations for one try of the solver.")
      .def_rw("max_time", &SimpleIkOptions::max_time, "Max total computation time, in seconds.")
      .def_rw("max_restarts", &SimpleIkOptions::max_restarts,
              "Maximum number of restarts until success.")
      .def_rw("step_size", &SimpleIkOptions::step_size, "The integration step for the solver.")
      .def_rw("damping", &SimpleIkOptions::damping, "Damping value for the Jacobian pseudoinverse.")
      .def_rw("max_error_norm", &SimpleIkOptions::max_error_norm, "The maximum error norm.")
      .def_rw("check_collisions", &SimpleIkOptions::check_collisions,
              "Whether to check collisions.");

  nanobind::class_<SimpleIk>(
      m, "SimpleIk", "Simple inverse kinematics (IK) solver based on the Jacobian pseudoinverse.")
      .def(nanobind::init<const std::shared_ptr<Scene>, const SimpleIkOptions&>(), "scene"_a,
           "options"_a)
      .def("solveIk", &SimpleIk::solveIk, "Solves inverse kinematics.", "goal"_a, "start"_a,
           "solution"_a);
}

}  // namespace roboplan
