#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>

#include <roboplan/core/scene.hpp>
#include <roboplan_toppra/toppra.hpp>

#include <modules/toppra.hpp>
#include <roboplan_bindings/expected.hpp>

namespace roboplan {

using namespace nanobind::literals;

void init_toppra(nanobind::module_& m) {
  nanobind::enum_<SplineFittingMode>(m, "SplineFittingMode",
                                     "Enumeration for TOPP-RA spline fitting mode.")
      .value("Hermite", SplineFittingMode::Hermite)
      .value("Cubic", SplineFittingMode::Cubic)
      .value("Adaptive", SplineFittingMode::Adaptive)
      .value("LinearBlend", SplineFittingMode::LinearBlend);

  nanobind::class_<TOPPRAOptions>(m, "TOPPRAOptions",
                                  "Options controlling TOPP-RA time parameterization.")
      .def(nanobind::init<>())
      .def(
          "__init__",
          [](TOPPRAOptions* self, double dt, SplineFittingMode mode, double velocity_scale,
             double acceleration_scale, int max_adaptive_iterations, double max_adaptive_step_size,
             double max_blend_deviation) {
            new (self) TOPPRAOptions{dt,
                                     mode,
                                     velocity_scale,
                                     acceleration_scale,
                                     max_adaptive_iterations,
                                     max_adaptive_step_size,
                                     max_blend_deviation};
          },
          "dt"_a = 0.01, "mode"_a = SplineFittingMode::Hermite, "velocity_scale"_a = 1.0,
          "acceleration_scale"_a = 1.0, "max_adaptive_iterations"_a = 10,
          "max_adaptive_step_size"_a = 0.05, "max_blend_deviation"_a = 0.01)
      .def_rw("dt", &TOPPRAOptions::dt)
      .def_rw("mode", &TOPPRAOptions::mode)
      .def_rw("velocity_scale", &TOPPRAOptions::velocity_scale)
      .def_rw("acceleration_scale", &TOPPRAOptions::acceleration_scale)
      .def_rw("max_adaptive_iterations", &TOPPRAOptions::max_adaptive_iterations)
      .def_rw("max_adaptive_step_size", &TOPPRAOptions::max_adaptive_step_size)
      .def_rw("max_blend_deviation", &TOPPRAOptions::max_blend_deviation);

  nanobind::class_<PathParameterizerTOPPRA>(
      m, "PathParameterizerTOPPRA", "Trajectory time parameterizer using the TOPP-RA algorithm.")
      .def(nanobind::init<const std::shared_ptr<Scene>, std::string>(), "scene"_a,
           "group_name"_a = "")
      .def("generate", unwrap_expected(&PathParameterizerTOPPRA::generate),
           "Time-parameterizes a joint-space path using TOPP-RA.", "path"_a,
           "options"_a = TOPPRAOptions{});
}

}  // namespace roboplan
