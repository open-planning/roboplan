#pragma once

#include <nanobind/nanobind.h>

namespace roboplan {

/// @brief Initializes Python bindings for filters.
/// @param m The nanobind filters module.
void init_filters(nanobind::module_& m);

}  // namespace roboplan
