#pragma once

#include <filesystem>

namespace roboplan::example_models {

/// @brief Provides compile time access to the resources install directory.
std::filesystem::path get_install_prefix();

/// @brief Provides compile time access to the resources shared directory for
/// accessing robot models or other resource files.
std::filesystem::path get_package_share_dir();

/// @brief Provides compile time access to the directory under the resources
/// shared directory which contains all the example robot models.
std::filesystem::path get_package_models_dir();

}  // namespace roboplan::example_models
