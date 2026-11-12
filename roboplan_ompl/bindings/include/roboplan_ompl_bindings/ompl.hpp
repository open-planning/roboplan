#pragma once

#include <nanobind/nanobind.h>

namespace roboplan {

/// @brief Initializes Python bindings for the OMPL planner wrapper.
/// @param m The nanobind module.
void initOmpl(nanobind::module_& m);

}  // namespace roboplan
