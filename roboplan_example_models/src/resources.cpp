#include <cstdlib>
#include <dlfcn.h>  // Provided by dlfcn-win32 on Windows.
#include <filesystem>

#include <roboplan_example_models/resources.hpp>

namespace roboplan::example_models {

namespace anchor {
extern void exampleModelsLocationAnchor();
}

std::filesystem::path get_install_prefix() {
  // Not an ament package, so find this shared library's path with dladdr.
  Dl_info dl_info;
  dladdr((void*)&anchor::exampleModelsLocationAnchor, &dl_info);
  const auto lib_path = std::filesystem::path(dl_info.dli_fname).lexically_normal();

  // The library sits in <install_directory>/lib (bin on Windows), so the prefix is two levels up.
  const auto prefix = lib_path.parent_path().parent_path();

  // For compiled installs with symlinks, dladdr may follow and break (e.g. on MacOS).
  // If the path doesn't exist we must rely on a compile time prefix.
  if (std::filesystem::exists(prefix / "share" / "roboplan_example_models")) {
    return prefix;
  }

  // Fall back to compile-time install prefix.
  return ROBOPLAN_INSTALL_PREFIX;
}

std::filesystem::path get_package_share_dir() { return get_install_prefix() / "share"; }

std::filesystem::path get_package_models_dir() {
  return get_package_share_dir() / "roboplan_example_models" / "models";
}
}  // namespace roboplan::example_models
