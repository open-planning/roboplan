cmake_minimum_required(VERSION 3.22)
project(roboplan_aligator VERSION 0.5.1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# The aligator FetchContent fallback toggles BUILD_TESTING for its own build, which would
# clobber our test gating. Capture the desired test state under a package-local option
# before fetching (mirrors roboplan_oink's BUILD_TESTING_OINK).
option(BUILD_TESTING "" ON)
option(BUILD_TESTING_ALIGATOR "" ${BUILD_TESTING})

# Include FetchContent for downloading dependencies.
include(FetchContent)

# Find dependencies.
find_package(roboplan REQUIRED)

# Ament CMake is not immediately required, but we include it at the top so that
# if it is found we can use it in build testing before requiring a
# find_package(ament_cmake_test REQUIRED). Otherwise this can cause infinite recursion
# issues with symlink installs.
find_package(ament_cmake QUIET)

# roboplan_aligator wraps the aligator proximal-DDP trajectory optimizer. Prefer an
# installed aligator; otherwise fetch a pinned commit via FetchContent (approved
# dependency, see API_NOTES.md). aligator is BSD-2-Clause and built with Pinocchio
# support by default (BUILD_WITH_PINOCCHIO_SUPPORT -> ALIGATOR_WITH_PINOCCHIO).
find_package(aligator CONFIG QUIET)

if(aligator_FOUND)
  message(STATUS "Found installed aligator ${aligator_VERSION}.")
else()
  message(STATUS "aligator not found. Fetching from GitHub...")

  # Build only what roboplan_aligator links against: disable aligator's own tests,
  # benchmarks, examples, Python bindings, and documentation. Pinocchio support is
  # required (it gates the multibody spaces/dynamics/residuals we consume) and is ON by
  # default. BUILD_TESTING is forced OFF here so aligator's tests do not build; the
  # parent's desired test state is preserved in BUILD_TESTING_ALIGATOR above.
  set(BUILD_TESTING OFF)
  set(BUILD_BENCHMARKS OFF CACHE BOOL "Disable aligator benchmarks" FORCE)
  set(BUILD_EXAMPLES OFF CACHE BOOL "Disable aligator examples" FORCE)
  set(BUILD_PYTHON_INTERFACE OFF CACHE BOOL "Disable aligator Python bindings" FORCE)
  set(INSTALL_DOCUMENTATION OFF CACHE BOOL "Disable aligator documentation" FORCE)
  set(BUILD_WITH_PINOCCHIO_SUPPORT ON CACHE BOOL "Enable aligator Pinocchio support" FORCE)

  set(_aligator_fetch_extra "")
  if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24")
    list(APPEND _aligator_fetch_extra DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  endif()
  FetchContent_Declare(
    aligator
    GIT_REPOSITORY https://github.com/Simple-Robotics/aligator.git
    GIT_TAG c54d96450112a1bba0e3f798ac0d1f6887412df3
    ${_aligator_fetch_extra}
  )
  FetchContent_MakeAvailable(aligator)

  # aligator's create_library() already defines the aligator::aligator alias, but if a
  # future build path yields only the non-namespaced target, add the alias to match the
  # installed package (mirrors roboplan_oink's proxsuite handling).
  if(TARGET aligator AND NOT TARGET aligator::aligator)
    add_library(aligator::aligator ALIAS aligator)
  endif()
endif()

# Add libraries
add_library(roboplan_aligator SHARED
    src/reduced_group_model.cpp
    src/types.cpp
    src/problem_builder.cpp
    src/cost_factory.cpp
    src/constraint_factory.cpp
    src/trajectory_optimizer.cpp
)
target_include_directories(roboplan_aligator PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(roboplan_aligator
    PUBLIC roboplan::roboplan
    PUBLIC aligator::aligator
)
set_property(TARGET roboplan_aligator PROPERTY CXX_STANDARD 20)

install(
  TARGETS
  roboplan_aligator
  EXPORT roboplan_aligator-targets
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Generate the package configuration file
configure_file(
    "cmake/roboplan_aligatorConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/roboplan_aligatorConfig.cmake"
    @ONLY
)

# Create and install the configuration files
include(CMakePackageConfigHelpers)
configure_package_config_file(
    cmake/roboplan_aligatorConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/roboplan_aligatorConfig.cmake
    INSTALL_DESTINATION lib/cmake/roboplan_aligator
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/roboplan_aligatorConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/roboplan_aligatorConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/roboplan_aligatorConfigVersion.cmake"
    DESTINATION
        "lib/cmake/roboplan_aligator"
)

# Install the headers and targets
install(DIRECTORY
            include/roboplan_aligator
        DESTINATION lib/cmake/)

install(EXPORT roboplan_aligator-targets
        FILE roboplan_aligatorTargets.cmake
        NAMESPACE roboplan_aligator::
        DESTINATION lib/cmake/roboplan_aligator)

# Add the unit tests, if enabled.
if (BUILD_TESTING_ALIGATOR)
    enable_testing()
    add_subdirectory(test)
endif()

# Build the Python bindings by default (for the scikit-build wheel, colcon, and pixi).
# Override with -DBUILD_PYTHON_BINDINGS=OFF for a pure C++ build.
option(BUILD_PYTHON_BINDINGS "Build the roboplan_aligator Python bindings" ON)
if(BUILD_PYTHON_BINDINGS)
  add_subdirectory(bindings)
endif()

# If ament_cmake is detected, export the package to be visible in ROS 2.
if ( ament_cmake_FOUND )
    message(STATUS "ament_cmake found. Exporting for ROS 2.")
    ament_package()
endif()
