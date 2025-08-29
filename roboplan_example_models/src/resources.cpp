#include <dlfcn.h>

#include <roboplan_example_models/resources.hpp>

namespace roboplan::example_models {

std::filesystem::path get_install_prefix() {
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

std::filesystem::path get_package_share_dir() {
  return get_install_prefix() / "share" / "roboplan_example_models" / "models";
}

}  // namespace roboplan::example_models
