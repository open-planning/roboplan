#include <nanobind/nanobind.h>

#include <modules/toppra.hpp>

namespace roboplan {

// Compiled extension backing the `roboplan.toppra` Python package.
NB_MODULE(_toppra_ext, m) {
  m.attr("__version__") = ROBOPLAN_VERSION;

  // Ensure core types (e.g. Scene) are registered before referencing them.
  nanobind::module_::import_("roboplan.core");

  init_toppra(m);
}

}  // namespace roboplan
