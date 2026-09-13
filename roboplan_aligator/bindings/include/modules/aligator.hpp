#pragma once

#include <nanobind/nanobind.h>

namespace roboplan {

/// @brief Initializes Python bindings for the aligator trajectory optimizer.
/// @param m The nanobind core module.
void init_aligator(nanobind::module_& m);

}  // namespace roboplan
