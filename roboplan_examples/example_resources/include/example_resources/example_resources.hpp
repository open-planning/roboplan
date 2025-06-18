#pragma once

#include <string>

namespace roboplan_example_resources {

/**
 * Provides compile time access to the resources install directory.
 */
std::string get_install_prefix();

/**
 * Provides compile time access to the resources shared directory for accessing
 * robot models or other resource files.
 */
std::string get_package_share_dir();

} // namespace roboplan_example_resources
