#include <nanobind/nanobind.h>

#include <modules/cartesian_path_planner.hpp>

namespace roboplan {

// Compiled extension backing the `roboplan.cartesian_planning` Python package.
NB_MODULE(_cartesian_ext, m) {
  m.attr("__version__") = ROBOPLAN_VERSION;

  // Ensure core types (e.g. Scene, CartesianPath) are registered before referencing them.
  nanobind::module_::import_("roboplan.core");
  // Ensure Oink types are registered so the custom-components constructor can accept them.
  nanobind::module_::import_("roboplan.optimal_ik");

  init_cartesian_path_planner(m);
}

}  // namespace roboplan
