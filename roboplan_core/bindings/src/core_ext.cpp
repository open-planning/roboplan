#include <nanobind/nanobind.h>

#include <roboplan_bindings/core.hpp>

namespace roboplan {

using namespace nanobind::literals;

// Compiled extension backing the `roboplan.core` Python package.
NB_MODULE(_core_ext, m) {
  m.attr("__version__") = ROBOPLAN_VERSION;

  init_core_types(m);
  init_core_geometry_wrappers(m);
  init_core_scene(m);
  init_core_path_utils(m);
  init_core_pose_utils(m);
  init_core_scene_utils(m);
}

}  // namespace roboplan
