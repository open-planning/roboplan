#include <nanobind/eigen/dense.h>
#include <nanobind/nanobind.h>
#include <nanobind/stl/shared_ptr.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>

#include <roboplan/core/scene.hpp>
#include <roboplan_vex/vex.hpp>

#include <roboplan_bindings/expected.hpp>

namespace roboplan {

using namespace nanobind::literals;

namespace {

void initVex(nanobind::module_& m) {
  nanobind::enum_<VexStatus>(m, "VexStatus", "Termination status of a vex solve.")
      .value("Converged", VexStatus::Converged,
             "Converged with all nonconvex constraints satisfied within the tolerance.")
      .value("ConvergedInfeasible", VexStatus::ConvergedInfeasible,
             "Converged, but nonconvex constraints are still violated after the maximum number "
             "of penalty increases.")
      .value("MaxIterations", VexStatus::MaxIterations,
             "Stopped after `max_iterations` QP iterations.")
      .value("QpFailure", VexStatus::QpFailure, "A convex subproblem could not be solved.")
      .value("Timeout", VexStatus::Timeout, "Stopped after `max_time` seconds.");

  nanobind::class_<VexOptions>(m, "VexOptions",
                               "Options of the vex trajectory optimizer. Joint limits are read "
                               "from the Scene.")
      .def(nanobind::init<>())
      .def_rw("group_name", &VexOptions::group_name, "The name of the joint group to optimize.")
      .def_rw("num_nodes", &VexOptions::num_nodes,
              "The number of trajectory nodes N (>= 3), uniformly spaced in time.")
      .def_rw("samples_per_segment", &VexOptions::samples_per_segment,
              "Collision samples per segment, in addition to the nodes (0 = nodes only).")
      .def_rw("velocity_weight", &VexOptions::velocity_weight,
              "Weight of the path-energy cost, the integral of ||v(t)||^2.")
      .def_rw("acceleration_weight", &VexOptions::acceleration_weight,
              "Weight of the smoothing cost, the integral of ||a(t)||^2.")
      .def_rw("collision_margin", &VexOptions::collision_margin,
              "Minimum signed distance d_safe (m) between every active collision pair.")
      .def_rw("collision_check_distance", &VexOptions::collision_check_distance,
              "Pairs closer than this distance d_check (m) enter the QP; must exceed d_safe.")
      .def_rw("initial_penalty", &VexOptions::initial_penalty,
              "Initial l1 penalty weight on nonconvex constraint violations.")
      .def_rw("penalty_increase", &VexOptions::penalty_increase,
              "Factor by which the penalty grows when converged but still infeasible.")
      .def_rw("max_penalty_increases", &VexOptions::max_penalty_increases,
              "Maximum number of penalty increases.")
      .def_rw("initial_trust_box", &VexOptions::initial_trust_box,
              "Initial half-width of the trust box, in position units.")
      .def_rw("trust_shrink", &VexOptions::trust_shrink,
              "Factor applied to the trust box after a rejected step.")
      .def_rw("trust_expand", &VexOptions::trust_expand,
              "Factor applied to the trust box after an accepted step.")
      .def_rw("improve_ratio", &VexOptions::improve_ratio,
              "Minimum ratio of actual to predicted merit improvement to accept a step.")
      .def_rw("max_iterations", &VexOptions::max_iterations,
              "Maximum number of SCP iterations (QP solves after the initial projection).")
      .def_rw("max_time", &VexOptions::max_time, "Maximum wall time of the SCP loop, in seconds.")
      .def_rw("tolerance", &VexOptions::tolerance,
              "Tolerance on constraint violation, predicted merit improvement and trust box size.");

  nanobind::class_<VexResult>(m, "VexResult", "Result of a vex solve.")
      .def_ro("trajectory", &VexResult::trajectory,
              "The optimized trajectory over the group joints. Not retimed.")
      .def_ro("status", &VexResult::status, "The termination status.")
      .def_ro("iterations", &VexResult::iterations, "The number of SCP iterations.")
      .def_ro("cost", &VexResult::cost, "The final convex (smoothness) cost.")
      .def_ro("max_violation", &VexResult::max_violation,
              "The final maximum violation of the nonconvex constraints.")
      .def_ro("continuous_collision_free", &VexResult::continuous_collision_free,
              "Whether a dense collision post-check over all active pairs passed.");

  nanobind::class_<Vex>(m, "Vex",
                        "Offline, kinematic-only trajectory optimizer based on sequential convex "
                        "programming.")
      .def(nanobind::init<std::shared_ptr<Scene>, VexOptions>(), "scene"_a, "options"_a)
      .def("solve", unwrap_expected(&Vex::solve),
           nanobind::call_guard<nanobind::gil_scoped_release>(),
           "Optimizes a rest-to-rest trajectory of the given duration (s) from start to goal, "
           "starting from the seed path. Raises RuntimeError for invalid input, a start or goal "
           "in collision, or an infeasible problem.",
           "start"_a, "goal"_a, "duration"_a, "seed"_a);
}

}  // namespace

// Compiled extension backing the `roboplan.vex` Python package.
NB_MODULE(_vex_ext, m) {
  m.attr("__version__") = ROBOPLAN_VERSION;

  // Ensure core types (e.g. Scene) are registered before referencing them.
  nanobind::module_::import_("roboplan.core");

  initVex(m);
}

}  // namespace roboplan
