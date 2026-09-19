#include <nanobind/nanobind.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/string.h>

#include <roboplan_example_models/resources.hpp>

#include <modules/example_models.hpp>

namespace roboplan {

using namespace nanobind::literals;

void init_example_models(nanobind::module_& m) {

  m.def("get_install_prefix", &example_models::get_install_prefix,
        "Returns the install prefix, located at runtime from this shared library.");
  m.def("get_package_share_dir", &example_models::get_package_share_dir,
        "Returns the `share` directory under the install prefix.");
  m.def("get_package_models_dir", &example_models::get_package_models_dir,
        "Returns the example robot models directory (`share/roboplan_example_models/models`).");
}

}  // namespace roboplan
