#pragma once

#include <nanobind/nanobind.h>

namespace roboplan {

/// @brief Initializes Python bindings for example models module.
/// @param m The nanobind core module.
void init_example_models(nanobind::module_& m);

}  // namespace roboplan
