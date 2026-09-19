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
      .def_rw("dt", &TOPPRAOptions::dt,
              "The sample time of the output trajectory, in seconds. Must be strictly positive.")
      .def_rw("mode", &TOPPRAOptions::mode,
              "The mode to use for spline fitting the path. Hermite fits a cubic Hermite spline "
              "with zero velocity at all waypoints, which can be slow but adheres to the path "
              "exactly. Cubic fits a smoother spline with zero acceleration only at the "
              "endpoints; it can deviate from the path, so it is collision checked and falls "
              "back to Hermite. Adaptive uses Cubic but adds intermediate points where it finds "
              "collisions, up to max_adaptive_iterations, then falls back to Hermite. "
              "LinearBlend joins straight-line segments with circular corner blends (see "
              "max_blend_deviation), is collision checked, and falls back to Hermite.")
      .def_rw("velocity_scale", &TOPPRAOptions::velocity_scale,
              "A scaling factor in (0, 1] for velocity limits.")
      .def_rw("acceleration_scale", &TOPPRAOptions::acceleration_scale,
              "A scaling factor in (0, 1] for acceleration limits.")
      .def_rw("max_adaptive_iterations", &TOPPRAOptions::max_adaptive_iterations,
              "Maximum number of adaptive iterations, if adaptive mode is enabled.")
      .def_rw("max_adaptive_step_size", &TOPPRAOptions::max_adaptive_step_size,
              "If adaptive mode is enabled, the maximum joint configuration step size to sample "
              "generated splines for collision checking.")
      .def_rw("max_blend_deviation", &TOPPRAOptions::max_blend_deviation,
              "Maximum distance a corner blend may deviate from the sharp corner, in the joint "
              "configuration's units. Only used by LinearBlend. Larger values round corners more "
              "(faster, but the path strays further from the waypoints); values <= 0 disable "
              "blending.");

  nanobind::class_<PathParameterizerTOPPRA>(
      m, "PathParameterizerTOPPRA", "Trajectory time parameterizer using the TOPP-RA algorithm.")
      .def(nanobind::init<const std::shared_ptr<Scene>, std::string>(), "scene"_a,
           "group_name"_a = "")
      .def("generate", unwrap_expected(&PathParameterizerTOPPRA::generate),
           nanobind::call_guard<nanobind::gil_scoped_release>(),
           "Time-parameterizes a joint-space path using TOPP-RA.", "path"_a,
           "options"_a = TOPPRAOptions{});
}

}  // namespace roboplan
