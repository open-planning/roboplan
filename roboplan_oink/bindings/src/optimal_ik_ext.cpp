#include <nanobind/nanobind.h>

#include <modules/optimal_ik.hpp>

namespace roboplan {

// Compiled extension backing the `roboplan.optimal_ik` Python package.
NB_MODULE(_optimal_ik_ext, m) {
  m.attr("__version__") = ROBOPLAN_VERSION;

  // Ensure core types (e.g. Scene) are registered before referencing them.
  nanobind::module_::import_("roboplan.core");

  init_optimal_ik(m);
}

}  // namespace roboplan
