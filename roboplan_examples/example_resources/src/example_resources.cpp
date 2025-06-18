#include <dlfcn.h>
#include <filesystem>
#include <iostream>

#include <example_resources/example_resources.hpp>

namespace roboplan_example_resources {
std::string get_install_prefix() {
  // This would be a lot easier if it were an ament package, instead we use
  // dynamic linking to get the filesystem path of this library.
  Dl_info dl_info;
  dladdr((void*)get_install_prefix, &dl_info);
  std::filesystem::path lib_path = dl_info.dli_fname;

  // Then we can just pull the relative path to the share directory
  // <install_directory>/lib/libroboplan_example_resources.so
  return lib_path.parent_path().parent_path();
}

std::string get_package_share_dir() {
  return get_install_prefix() + "/share/roboplan_example_resources";
}
} // namespace roboplan_example_resources
