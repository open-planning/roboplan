#include <cstdlib>
#include <dlfcn.h>
#include <filesystem>

#include <roboplan_example_models/resources.hpp>

namespace roboplan::example_models {

namespace anchor {
extern void exampleModelsLocationAnchor();
}

std::filesystem::path get_install_prefix() {
  // This would be a lot easier if it were an ament package, instead we use
  // dynamic linking to get the filesystem path of the example resources shared
  // object file.
  Dl_info dl_info;
  dladdr((void*)&anchor::exampleModelsLocationAnchor, &dl_info);
  const auto lib_path = std::filesystem::path(dl_info.dli_fname).lexically_normal();

  // Then we can just pull the relative path to the share directory
  // <install_directory>/lib/roboplan_example_models/<executable>
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
