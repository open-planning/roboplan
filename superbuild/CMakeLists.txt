cmake_minimum_required(VERSION 3.22)
project(roboplan_superbuild)

# Superbuild: configures every RoboPlan package in one tree, so
# build/install/test run once instead of once per package. Driven by
# pyproject.toml and scripts/build_cmake.bash. See CMakePresets.json for
# compiler/linker/build-cache settings.
#
# Not at repo root on purpose: colcon would identify a bare CMakeLists.txt
# there as its own "cmake" package and stop recursing, hiding the real
# ament packages underneath. This dir has its own COLCON_IGNORE instead.
#
# Each package below is also an independent project resolving siblings via
# find_package(<name> REQUIRED), so it stays buildable standalone (colcon,
# conda-forge, its own wheel). roboplan_register_build_tree_packages() makes
# those calls resolve to the in-tree targets instead of requiring a real
# install -- see RoboplanSuperbuild.cmake.
get_filename_component(ROBOPLAN_REPOSITORY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)

include("${CMAKE_CURRENT_SOURCE_DIR}/RoboplanSuperbuild.cmake")
include("${ROBOPLAN_REPOSITORY_ROOT}/roboplan_common/cmake/RoboplanPythonInstall.cmake")

option(ROBOPLAN_BUILD_EXAMPLES "Build the RoboPlan examples" ON)

roboplan_register_build_tree_packages()

# compile_commands.json export and colored diagnostics live in
# CMakePresets.json, not here: configure with --preset default (or pixi).
# CMAKE_EXPORT_COMPILE_COMMANDS can't be set via set(... CACHE ...) here --
# CMake pre-seeds it blank before project code runs, so that's a no-op.

# Declared before any add_subdirectory() so the value applies everywhere:
# each package's own option() of the same name is a no-op once this one has
# already set it.
option(BUILD_TESTING "Build the RoboPlan C++ unit tests" ON)
option(BUILD_PYTHON_BINDINGS "Build the RoboPlan Python bindings" ON)

# Same default as standalone: off on Windows (build-tree DLL search path).
# Cross-package imports resolve via each package staging its extension into
# pystage/, not a real install.
if(WIN32)
  set(_generate_stubs_default OFF)
else()
  set(_generate_stubs_default ON)
endif()
option(GENERATE_PYTHON_STUBS "Generate .pyi stubs for the Python bindings" ${_generate_stubs_default})

if(BUILD_TESTING)
  enable_testing()
endif()

add_subdirectory("${ROBOPLAN_REPOSITORY_ROOT}/roboplan_common" "roboplan_common")
add_subdirectory("${ROBOPLAN_REPOSITORY_ROOT}/roboplan_example_models" "roboplan_example_models")
add_subdirectory("${ROBOPLAN_REPOSITORY_ROOT}/roboplan" "roboplan")
add_subdirectory("${ROBOPLAN_REPOSITORY_ROOT}/roboplan_simple_ik" "roboplan_simple_ik")
add_subdirectory("${ROBOPLAN_REPOSITORY_ROOT}/roboplan_oink" "roboplan_oink")
add_subdirectory("${ROBOPLAN_REPOSITORY_ROOT}/roboplan_rrt" "roboplan_rrt")
add_subdirectory("${ROBOPLAN_REPOSITORY_ROOT}/roboplan_toppra" "roboplan_toppra")
add_subdirectory("${ROBOPLAN_REPOSITORY_ROOT}/roboplan_cartesian_planning" "roboplan_cartesian_planning")
if(ROBOPLAN_BUILD_EXAMPLES)
  add_subdirectory("${ROBOPLAN_REPOSITORY_ROOT}/roboplan_examples" "roboplan_examples")
endif()

if(DEFINED ENV{CMEEL_BUILD})
  if(APPLE)
    set(_roboplan_library_rpath "@loader_path")
    set(_roboplan_extension_rpath "@loader_path/../../../../")
  else()
    set(_roboplan_library_rpath "$ORIGIN")
    set(_roboplan_extension_rpath "$ORIGIN/../../../../")
  endif()

  foreach(target IN ITEMS
      roboplan_example_models
      roboplan
      roboplan_filters
      roboplan_simple_ik
      roboplan_oink
      roboplan_rrt
      roboplan_toppra
      roboplan_cartesian_planning)
    if(TARGET ${target})
      set_property(TARGET ${target} PROPERTY INSTALL_RPATH
        "${_roboplan_library_rpath}")
    endif()
  endforeach()
  foreach(target IN ITEMS
      _example_models_ext
      _core_ext
      _filters_ext
      _simple_ik_ext
      _optimal_ik_ext
      _rrt_ext
      _toppra_ext
      _cartesian_ext)
    if(TARGET ${target})
      set_property(TARGET ${target} PROPERTY INSTALL_RPATH
        "${_roboplan_extension_rpath}")
    endif()
  endforeach()
endif()
