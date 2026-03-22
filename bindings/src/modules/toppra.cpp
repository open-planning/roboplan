#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include <roboplan/core/scene.hpp>
#include <roboplan_toppra/toppra.hpp>

#include <modules/toppra.hpp>
#include <utils/expected.hpp>

namespace roboplan {

using namespace nanobind::literals;

void init_toppra(nanobind::module_& m) {
  nanobind::enum_<SplineFittingMode>(m, "SplineFittingMode",
                                     "Enumeration for TOPP-RA spline fitting mode.")
      .value("Hermite", SplineFittingMode::Hermite)
      .value("Cubic", SplineFittingMode::Cubic);

  nanobind::class_<PathParameterizerTOPPRA>(
      m, "PathParameterizerTOPPRA", "Trajectory time parameterizer using the TOPP-RA algorithm.")
      .def(nanobind::init<const std::shared_ptr<Scene>, std::string>(), "scene"_a,
           "group_name"_a = "")
      .def("generate", unwrap_expected(&PathParameterizerTOPPRA::generate),
           "Time-parameterizes a joint-space path using TOPP-RA.", "path"_a, "dt"_a,
           "mode"_a = SplineFittingMode::Hermite, "velocity_scale"_a = 1.0,
           "acceleration_scale"_a = 1.0);
}

}  // namespace roboplan
