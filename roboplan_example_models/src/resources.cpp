#include <cstdlib>
#include <dlfcn.h>

#include <roboplan_example_models/resources.hpp>

namespace roboplan::example_models {

namespace anchor {
extern void example_models_location_anchor();
}

std::filesystem::path get_install_prefix() {
  // This would be a lot easier if it were an ament package, instead we use
  // dynamic linking to get the filesystem path of the example resources shared
  // object file.
  Dl_info dl_info;
  dladdr((void*)&anchor::example_models_location_anchor, &dl_info);
  const auto lib_path = std::filesystem::path(dl_info.dli_fname).lexically_normal();

  // Then we can just pull the relative path to the share directory
  // <install_directory>/lib/roboplan_example_models/<executable>
  return lib_path.parent_path().parent_path();
}

std::filesystem::path get_package_share_dir() { return get_install_prefix() / "share"; }

std::filesystem::path get_package_models_dir() {
  return get_package_share_dir() / "roboplan_example_models" / "models";
}
}  // namespace roboplan::example_models
