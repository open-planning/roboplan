#pragma once

#include <nanobind/nanobind.h>

namespace roboplan {

/// @brief Initializes Python bindings for the simple IK solver.
/// @param m The nanobind core module.
void init_simple_ik(nanobind::module_& m);

}  // namespace roboplan
