#include <nanobind/nanobind.h>

#include <roboplan_ompl_bindings/ompl.hpp>

namespace roboplan {

// Compiled extension backing the `roboplan.ompl` Python package.
NB_MODULE(_ompl_ext, m) {
  m.attr("__version__") = ROBOPLAN_VERSION;

  // Ensure the core extension is imported first so that its types (e.g. Scene)
  // are registered in nanobind's process-wide type registry before this module
  // references them across the extension boundary.
  nanobind::module_::import_("roboplan.core");

  initOmpl(m);
}

}  // namespace roboplan
