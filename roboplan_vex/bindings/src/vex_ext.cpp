#include <nanobind/nanobind.h>

namespace roboplan {

// Compiled extension backing the `roboplan.vex` Python package.
NB_MODULE(_vex_ext, m) {
  m.attr("__version__") = ROBOPLAN_VERSION;

  // Ensure core types (e.g. Scene) are registered before referencing them.
  nanobind::module_::import_("roboplan.core");
}

}  // namespace roboplan
