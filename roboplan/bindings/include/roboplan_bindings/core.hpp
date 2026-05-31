#pragma once

#include <nanobind/nanobind.h>

namespace roboplan {

/// @brief Initializes Python bindings for core roboplan types.
/// @param m The nanobind core module.
void init_core_types(nanobind::module_& m);

/// @brief Initializes Python bindings for core geometry wrappers.
/// @param m The nanobind core module.
void init_core_geometry_wrappers(nanobind::module_& m);

/// @brief Initializes Python bindings for the main scene representation of roboplan.
/// @param m The nanobind core module.
void init_core_scene(nanobind::module_& m);

/// @brief Initializes Python bindings for core path utilities.
/// @param m The nanobind core module.
void init_core_path_utils(nanobind::module_& m);

/// @brief Initializes Python bindings for core scene utilities.
/// @param m The nanobind core module.
void init_core_scene_utils(nanobind::module_& m);

}  // namespace roboplan
