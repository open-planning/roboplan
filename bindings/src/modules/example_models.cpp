#include <nanobind/nanobind.h>
#include <nanobind/stl/filesystem.h>
#include <nanobind/stl/string.h>

#include <roboplan_example_models/resources.hpp>

#include <modules/example_models.hpp>
#include <utils/expected.hpp>

namespace roboplan {

using namespace nanobind::literals;

void init_example_models(nanobind::module_& m) {

  m.def("get_install_prefix", &example_models::get_install_prefix,
        "Provides compile time access to the resources install directory.");
  m.def("get_package_share_dir", &example_models::get_package_share_dir,
        "Provides compile time access to the resources shared directory for accessing robot models "
        "or other resource files.");
  m.def("get_package_models_dir", &example_models::get_package_models_dir,
        "Provides compile time access to the directory under the resources shared directory which "
        "contains all the example robot models.");
}

}  // namespace roboplan
