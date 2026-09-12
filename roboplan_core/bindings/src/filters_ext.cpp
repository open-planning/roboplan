#include <nanobind/nanobind.h>

#include <roboplan_bindings/filters.hpp>

namespace roboplan {

// Compiled extension backing the `roboplan.filters` Python package.
NB_MODULE(_filters_ext, m) {
  m.attr("__version__") = ROBOPLAN_VERSION;

  init_filters(m);
}

}  // namespace roboplan
