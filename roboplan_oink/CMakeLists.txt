cmake_minimum_required(VERSION 3.22)
project(roboplan_oink VERSION 0.6.1)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
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

# Set a new oink specific build testing option since the fetched QP solver build
# can mess with the original.
option(BUILD_TESTING "" ON)
option(BUILD_TESTING_OINK "" ${BUILD_TESTING})

# Include FetchContent for downloading dependencies
include(FetchContent)

# Find dependencies.
find_package(roboplan REQUIRED)

# Ament CMake is not immediately required, but we include it at the top so that
# if it is found we can use it in build testing before requiring a
# find_package(ament_cmake_test REQUIRED). Otherwise this can cause infinite recursion
# issues with symlink installs.
find_package(ament_cmake QUIET)

# OInK uses ProxQP from proxsuite (header-only) as its QP solver.
find_package(proxsuite QUIET)

if(proxsuite_FOUND)
  message(STATUS "Found installed proxsuite ${proxsuite_VERSION}.")
else()
  message(STATUS "proxsuite not found. Fetching from GitHub...")

  # Disable proxsuite vectorization support (would require Simde) and tests.
  set(BUILD_WITH_VECTORIZATION_SUPPORT OFF CACHE INTERNAL "Disable proxsuite vectorization")
  set(BUILD_TESTING OFF)

  set(_proxsuite_fetch_extra "")
  if(CMAKE_VERSION VERSION_GREATER_EQUAL "3.24")
    list(APPEND _proxsuite_fetch_extra DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  endif()
  FetchContent_Declare(
    proxsuite
    URL https://github.com/Simple-Robotics/proxsuite/archive/refs/tags/v0.7.3.tar.gz
    URL_HASH SHA256=e634babff534c8812c6dbc6b022b1d01f9a2a4a9a73cf87a6dc299a4de996904
    ${_proxsuite_fetch_extra}
  )
  FetchContent_MakeAvailable(proxsuite)

  # proxsuite creates a non-namespaced target when built via FetchContent.
  # Create a namespaced alias to match the installed package.
  if(TARGET proxsuite AND NOT TARGET proxsuite::proxsuite)
    add_library(proxsuite::proxsuite ALIAS proxsuite)
  endif()
endif()

# Add libraries
add_library(roboplan_oink SHARED
    src/optimal_ik.cpp
    src/qp_backend.cpp
    src/tasks/frame.cpp
    src/tasks/configuration.cpp
    src/constraints/position_limit.cpp
    src/constraints/velocity_limit.cpp
    src/constraints/acceleration_limit.cpp
    src/barriers/position_barrier.cpp
    src/barriers/self_collision_barrier.cpp
)
target_include_directories(roboplan_oink PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
# proxsuite is linked PRIVATE: it is header-only and not part of roboplan_oink's public
# headers, so it must not leak into the exported link interface.
target_link_libraries(roboplan_oink
    PUBLIC roboplan::roboplan
    PRIVATE proxsuite::proxsuite
)
set_property(TARGET roboplan_oink PROPERTY CXX_STANDARD 20)
# Alias so this works via add_subdirectory() too, not just installed find_package().
add_library(roboplan_oink::roboplan_oink ALIAS roboplan_oink)

install(
  TARGETS
  roboplan_oink
  EXPORT roboplan_oink-targets
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Generate the package configuration file
configure_file(
    "cmake/roboplan_oinkConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/roboplan_oinkConfig.cmake"
    @ONLY
)

# Create and install the configuration files
include(CMakePackageConfigHelpers)
configure_package_config_file(
    cmake/roboplan_oinkConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/roboplan_oinkConfig.cmake
    INSTALL_DESTINATION lib/cmake/roboplan_oink
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/roboplan_oinkConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/roboplan_oinkConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/roboplan_oinkConfigVersion.cmake"
    DESTINATION
        "lib/cmake/roboplan_oink"
)

# Install the headers and targets
install(DIRECTORY
            include/roboplan_oink
        DESTINATION lib/cmake/)

install(EXPORT roboplan_oink-targets
        FILE roboplan_oinkTargets.cmake
        NAMESPACE roboplan_oink::
        DESTINATION lib/cmake/roboplan_oink)

# Add the unit tests, if enabled.
if (BUILD_TESTING_OINK)
    enable_testing()
    add_subdirectory(test)
endif()

# Build the Python bindings by default (for the scikit-build wheel, colcon, and pixi).
# Override with -DBUILD_PYTHON_BINDINGS=OFF for a pure C++ build.
option(BUILD_PYTHON_BINDINGS "Build the roboplan_oink Python bindings" ON)
if(BUILD_PYTHON_BINDINGS)
  add_subdirectory(bindings)
endif()

# If ament_cmake is detected, export the package to be visible in ROS 2.
if ( ament_cmake_FOUND )
    message(STATUS "ament_cmake found. Exporting for ROS 2.")
    ament_package()
endif()
