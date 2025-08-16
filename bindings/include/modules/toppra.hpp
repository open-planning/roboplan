#pragma once

#include <nanobind/nanobind.h>

namespace roboplan {

/// @brief Initializes Python bindings for the TOPP-RA path parameterizer.
/// @param m The nanobind core module.
void init_toppra(nanobind::module_& m);

}  // namespace roboplan
