#include <nanobind/nanobind.h>

#include <modules/example_models.hpp>

namespace roboplan {

// Compiled extension backing the `roboplan.example_models` Python package.
NB_MODULE(_example_models_ext, m) {
  m.attr("__version__") = ROBOPLAN_VERSION;

  init_example_models(m);
}

}  // namespace roboplan
