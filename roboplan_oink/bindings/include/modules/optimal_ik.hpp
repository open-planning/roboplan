#pragma once

#include <nanobind/nanobind.h>

namespace roboplan {

/// @brief Initializes Python bindings for the optimal IK solver.
/// @param m The nanobind optimal_ik module.
void init_optimal_ik(nanobind::module_& m);

}  // namespace roboplan
