#pragma once

#include <nanobind/nanobind.h>

namespace roboplan {

/// @brief Initializes Python bindings for the RRT path planner.
/// @param m The nanobind core module.
void init_rrt(nanobind::module_& m);

}  // namespace roboplan
