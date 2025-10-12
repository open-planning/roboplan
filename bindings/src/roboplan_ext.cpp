#include <nanobind/nanobind.h>

#include <modules/core.hpp>
#include <modules/example_models.hpp>
#include <modules/rrt.hpp>
#include <modules/simple_ik.hpp>
#include <modules/toppra.hpp>

namespace roboplan {

using namespace nanobind::literals;

NB_MODULE(roboplan_ext, m) {

  /// Core module
  nanobind::module_ m_core = m.def_submodule("core", "Core roboplan module");
  init_core_types(m_core);
  init_core_geometry_wrappers(m_core);
  init_core_scene(m_core);
  init_core_path_utils(m_core);
  init_core_scene_utils(m_core);

  /// Example models module
  nanobind::module_ m_example_models = m.def_submodule("example_models", "Example models");
  init_example_models(m_example_models);

  /// RRT module
  nanobind::module_ m_rrt = m.def_submodule("rrt", "RRT module");
  init_rrt(m_rrt);

  /// Simple IK module
  nanobind::module_ m_simple_ik = m.def_submodule("simple_ik", "Simple IK solver module");
  init_simple_ik(m_simple_ik);

  /// TOPP-RA module
  nanobind::module_ m_toppra = m.def_submodule("toppra", "TOPP-RA module");
  init_toppra(m_toppra);
}

}  // namespace roboplan
