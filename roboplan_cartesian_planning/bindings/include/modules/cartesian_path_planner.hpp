#pragma once

#include <nanobind/nanobind.h>

namespace roboplan {

/// @brief Initializes Python bindings for the Cartesian path planner.
/// @param m The nanobind core module.
void init_cartesian_path_planner(nanobind::module_& m);

}  // namespace roboplan
