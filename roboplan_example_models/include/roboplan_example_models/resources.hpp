#pragma once

#include <filesystem>

namespace roboplan::example_models {

/// @brief Provides compile time access to the resources install directory.
std::filesystem::path get_install_prefix();

/// @brief Provides compile time access to the resources shared directory for
/// accessing robot models or other resource files.
std::filesystem::path get_package_share_dir();

}  // namespace roboplan::example_models
