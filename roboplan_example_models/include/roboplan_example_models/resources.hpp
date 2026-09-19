#pragma once

#include <filesystem>

namespace roboplan::example_models {

/// @brief Returns the install prefix, located at runtime from this shared library.
/// @details Falls back to the compile-time install prefix if `share/roboplan_example_models` is
/// not found beside the library.
std::filesystem::path get_install_prefix();

/// @brief Returns the `share` directory under the install prefix.
std::filesystem::path get_package_share_dir();

/// @brief Returns the example robot models directory (`share/roboplan_example_models/models`).
std::filesystem::path get_package_models_dir();

}  // namespace roboplan::example_models
