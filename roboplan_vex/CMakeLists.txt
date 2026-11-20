cmake_minimum_required(VERSION 3.22)
project(roboplan_vex VERSION 0.8.0)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND CMAKE_CXX_COMPILER_FRONTEND_VARIANT
                                             STREQUAL "MSVC")
  # clang-cl remaps plain -Wall to MSVC /Wall (clang's -Weverything); use the
  # /clang: prefix to pass the GNU-style flags through unchanged.
  add_compile_options(/clang:-Wall /clang:-Wextra /clang:-Wpedantic)
elseif(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Windows: auto-export all symbols so DLLs get import libraries.
if(WIN32)
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif()

# Set a new vex specific build testing option since the fetched QP solver build
# can mess with the original.
option(BUILD_TESTING "" ON)
option(BUILD_TESTING_VEX "" ${BUILD_TESTING})

include(FetchContent)

# Find dependencies.
if(NOT TARGET roboplan::roboplan)
  find_package(roboplan_core REQUIRED)
endif()

# Ament CMake is not immediately required, but we include it at the top so that
# if it is found we can use it in build testing before requiring a
# find_package(ament_cmake_test REQUIRED). Otherwise this can cause infinite
# recursion issues with symlink installs.
find_package(ament_cmake QUIET)

# vex uses the sparse ProxQP solver from proxsuite (header-only), at the same
# pin as roboplan_oink. In the superbuild, roboplan_oink may already have
# fetched it, in which case the target exists.
if(NOT TARGET proxsuite::proxsuite)
  find_package(proxsuite QUIET)
endif()

if(TARGET proxsuite::proxsuite)
  message(STATUS "Using proxsuite ${proxsuite_VERSION}.")
else()
  message(STATUS "proxsuite not found. Fetching from GitHub...")

  # Disable proxsuite features not needed by RoboPlan, as well as tests.
  set(BUILD_WITH_VECTORIZATION_SUPPORT
      OFF
      CACHE INTERNAL "Disable proxsuite vectorization (requires Simde)")
  set(PROXSUITE_BUILD_WITH_SERIALIZATION
      OFF
      CACHE INTERNAL "Disable proxsuite serialization (requires cereal)")
  set(BUILD_PYTHON_INTERFACE
      OFF
      CACHE INTERNAL "Disable proxsuite Python bindings")
  set(BUILD_TESTING OFF)

  FetchContent_Declare(
    proxsuite
    GIT_REPOSITORY https://github.com/Simple-Robotics/proxsuite.git
    GIT_TAG v0.8.2
    GIT_SHALLOW TRUE)
  FetchContent_MakeAvailable(proxsuite)

  # proxsuite creates a non-namespaced target when built via FetchContent.
  # Create a namespaced alias to match the installed package.
  if(TARGET proxsuite AND NOT TARGET proxsuite::proxsuite)
    add_library(proxsuite::proxsuite ALIAS proxsuite)
  endif()
endif()

# Add libraries
add_library(roboplan_vex SHARED src/collision_term.cpp src/qp_backend.cpp
                                src/scp.cpp src/vex.cpp)
target_include_directories(
  roboplan_vex PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                      $<INSTALL_INTERFACE:include>)
# proxsuite is linked PRIVATE: it is header-only and not part of roboplan_vex's
# public headers, so it must not leak into the exported link interface.
target_link_libraries(
  roboplan_vex
  PUBLIC roboplan::roboplan
  PRIVATE proxsuite::proxsuite)
set_property(TARGET roboplan_vex PROPERTY CXX_STANDARD 20)
# Alias so this works via add_subdirectory() too, not just installed
# find_package().
add_library(roboplan_vex::roboplan_vex ALIAS roboplan_vex)

install(
  TARGETS roboplan_vex
  EXPORT roboplan_vex-targets
  LIBRARY DESTINATION lib
  INCLUDES
  DESTINATION include)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Create and install the package configuration files
include(CMakePackageConfigHelpers)
configure_package_config_file(
  cmake/roboplan_vexConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/roboplan_vexConfig.cmake
  INSTALL_DESTINATION lib/cmake/roboplan_vex)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/roboplan_vexConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/roboplan_vexConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/roboplan_vexConfigVersion.cmake"
        DESTINATION "lib/cmake/roboplan_vex")

install(
  EXPORT roboplan_vex-targets
  FILE roboplan_vexTargets.cmake
  NAMESPACE roboplan_vex::
  DESTINATION lib/cmake/roboplan_vex)

# Add the unit tests, if enabled.
if(BUILD_TESTING_VEX)
  enable_testing()
  add_subdirectory(test)
endif()

# Build the Python bindings by default (for the scikit-build wheel, colcon, and
# pixi). Override with -DBUILD_PYTHON_BINDINGS=OFF for a pure C++ build.
option(BUILD_PYTHON_BINDINGS "Build the roboplan_vex Python bindings" ON)
if(BUILD_PYTHON_BINDINGS)
  add_subdirectory(bindings)
endif()

# If ament_cmake is detected, export the package to be visible in ROS 2.
if(ament_cmake_FOUND)
  message(STATUS "ament_cmake found. Exporting for ROS 2.")
  ament_package()
endif()
