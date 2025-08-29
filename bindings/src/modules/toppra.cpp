#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>

#include <roboplan/core/scene.hpp>
#include <roboplan_toppra/toppra.hpp>

#include <modules/toppra.hpp>
#include <utils/expected.hpp>

namespace roboplan {

using namespace nanobind::literals;

void init_toppra(nanobind::module_& m) {
  nanobind::class_<PathParameterizerTOPPRA>(m, "PathParameterizerTOPPRA")
      .def(nanobind::init<const std::shared_ptr<Scene>>())
      .def("generate", unwrap_expected(&PathParameterizerTOPPRA::generate), "path"_a, "dt"_a,
           "velocity_scale"_a = 1.0, "acceleration_scale"_a = 1.0);
}

}  // namespace roboplan
