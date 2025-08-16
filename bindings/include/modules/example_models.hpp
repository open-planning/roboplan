#pragma once

#include <nanobind/nanobind.h>

/// @brief Initializes Python bindings for example models module.
/// @param m The nanobind core module.
void init_example_models(nanobind::module_& m);
