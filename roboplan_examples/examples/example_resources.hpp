#pragma once

#include <dlfcn.h>
#include <filesystem>
#include <string>

namespace roboplan_examples {

/**
 * Provides compile time access to the resources install directory.
 */
inline std::filesystem::path get_install_prefix() {
  // This would be a lot easier if it were an ament package, instead we use
  // dynamic linking to get the filesystem path of the example executable.
  Dl_info dl_info;
  dladdr((void*)get_install_prefix, &dl_info);
  std::filesystem::path lib_path = dl_info.dli_fname;

  // Then we can just pull the relative path to the share directory
  // <install_directory>/lib/roboplan_examples/<executable>
  return lib_path.parent_path().parent_path().parent_path();
}

/**
 * Provides compile time access to the resources shared directory for accessing
 * robot models or other resource files.
 */
inline std::filesystem::path get_package_share_dir() {
  return get_install_prefix() / "share" / "roboplan_examples";
}

} // namespace roboplan_examples
