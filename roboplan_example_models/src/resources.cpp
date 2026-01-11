#include <cstdlib>
#include <dlfcn.h>

#include <roboplan_example_models/resources.hpp>

namespace roboplan::example_models {

std::filesystem::path get_install_prefix() {
  // If roboplan was installed using conda, get it from the prefix.
  // Note that this will break in the event that you've built from source and are
  // using conda yourself, but it's a better default than not supporting conda at all.
  const auto conda_prefix = std::getenv("CONDA_PREFIX");
  if (conda_prefix) {
    return std::filesystem::path(conda_prefix);
  }

  // This would be a lot easier if it were an ament package, instead we use
  // dynamic linking to get the filesystem path of the example resources shared
  // object file.
  Dl_info dl_info;
  dladdr((void*)get_install_prefix, &dl_info);
  std::filesystem::path lib_path = dl_info.dli_fname;

  // Then we can just pull the relative path to the share directory
  // <install_directory>/lib/roboplan_example_models/<executable>
  return lib_path.parent_path().parent_path();
}

std::filesystem::path get_package_share_dir() { return get_install_prefix() / "share"; }

std::filesystem::path get_package_models_dir() {
  return get_package_share_dir() / "roboplan_example_models" / "models";
}
}  // namespace roboplan::example_models
