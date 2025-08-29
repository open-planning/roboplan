#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include <roboplan/core/scene.hpp>
#include <roboplan_simple_ik/simple_ik.hpp>

#include <modules/simple_ik.hpp>

namespace roboplan {

using namespace nanobind::literals;

void init_simple_ik(nanobind::module_& m) {

  nanobind::class_<SimpleIkOptions>(m, "SimpleIkOptions")
      .def(nanobind::init<>())  // Default constructor
      .def_rw("max_iters", &SimpleIkOptions::max_iters)
      .def_rw("step_size", &SimpleIkOptions::step_size)
      .def_rw("damping", &SimpleIkOptions::damping)
      .def_rw("max_error_norm", &SimpleIkOptions::max_error_norm);

  nanobind::class_<SimpleIk>(m, "SimpleIk")
      .def(nanobind::init<const std::shared_ptr<Scene>, const SimpleIkOptions&>())
      .def("solveIk", &SimpleIk::solveIk);
}

}  // namespace roboplan
