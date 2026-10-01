#include <nanobind/nanobind.h>

#include <modules/aligator.hpp>

namespace roboplan {

// Compiled extension backing the `roboplan.aligator` Python package.
NB_MODULE(_aligator_ext, m) {
  m.attr("__version__") = ROBOPLAN_VERSION;

  // Ensure core types (e.g. Scene) are registered before referencing them.
  nanobind::module_::import_("roboplan.core");

  init_aligator(m);
}

}  // namespace roboplan
