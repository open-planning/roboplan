cmake_minimum_required(VERSION 3.22)
project(roboplan_core VERSION 0.7.0)

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

# Find dependencies.
find_package(Eigen3 REQUIRED)
find_package(pinocchio 4.1.0 REQUIRED)
find_package(console_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)

# Ament CMake is not immediately required, but we include it at the top so that
# if it is found we can use it in build testing before requiring a
# find_package(ament_cmake_test REQUIRED). Otherwise this can cause infinite
# recursion issues with symlink installs.
find_package(ament_cmake QUIET)

set(TARGETS_LIST roboplan)

# tl::expected: the system copy if there is one (Debian/Ubuntu libexpected-dev
# and conda-forge cpp-expected install the tl-expected CMake package), otherwise
# the bundled header from third_party/. Either way it is provided as the
# exported roboplan::tl_expected target, which downstream packages (e.g.
# roboplan_ros) use.
add_library(tl_expected INTERFACE)
find_package(tl-expected QUIET)
if(TARGET tl::expected)
  target_link_libraries(tl_expected INTERFACE tl::expected)
  set(ROBOPLAN_EXPECTED_PACKAGE tl-expected)
else()
  message(STATUS "tl-expected not found, using the bundled tl/expected.hpp")
  target_include_directories(
    tl_expected INTERFACE $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/third_party>
                          $<INSTALL_INTERFACE:include>)
  install(DIRECTORY third_party/tl DESTINATION include)
  set(ROBOPLAN_EXPECTED_PACKAGE "")
endif()
list(APPEND TARGETS_LIST tl_expected)

# tinyxml2: the system copy if there is one, otherwise the bundled sources from
# third_party/ (linked privately, so its header is not installed).
find_package(tinyxml2_vendor QUIET)
find_package(TinyXML2 QUIET)
if(NOT tinyxml2_vendor_FOUND
   AND NOT TinyXML2_FOUND
   AND NOT TARGET tinyxml2::tinyxml2)
  message(STATUS "tinyxml2 not found, using the bundled sources")
  add_library(tinyxml2 third_party/tinyxml2/tinyxml2.cpp
                       third_party/tinyxml2/tinyxml2.h)
  target_include_directories(
    tinyxml2
    PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/third_party/tinyxml2>)
  add_library(tinyxml2::tinyxml2 ALIAS tinyxml2)
  list(APPEND TARGETS_LIST tinyxml2)
endif()

# Add roboplan libraries.
add_library(
  roboplan SHARED
  src/core/scene_context.cpp
  src/core/path_utils.cpp
  src/core/pose_utils.cpp
  src/core/robot_body_filter.cpp
  src/core/scene.cpp
  src/core/scene_utils.cpp
  src/core/types.cpp
  src/filters/se3_low_pass_filter.cpp)
target_include_directories(
  roboplan PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                  $<INSTALL_INTERFACE:include>)
target_link_libraries(
  roboplan
  INTERFACE Eigen3::Eigen
  PUBLIC pinocchio::pinocchio tl_expected
  PRIVATE tinyxml2::tinyxml2 console_bridge::console_bridge)
if(yaml-cpp_VERSION VERSION_LESS "0.8") # for Ubuntu 22.04
  target_link_libraries(roboplan PUBLIC yaml-cpp)
else()
  target_link_libraries(roboplan PUBLIC yaml-cpp::yaml-cpp)
endif()
set_property(TARGET roboplan PROPERTY CXX_STANDARD 20)
# Alias so this works via add_subdirectory() too, not just installed
# find_package().
add_library(roboplan::roboplan ALIAS roboplan)

set(ROBOPLAN_TARGETS ${TARGETS_LIST})
install(
  TARGETS ${ROBOPLAN_TARGETS}
  EXPORT roboplan-targets
  LIBRARY DESTINATION lib
  INCLUDES
  DESTINATION include)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Generate the package configuration file
configure_file("cmake/roboplanConfig.cmake.in"
               "${CMAKE_CURRENT_BINARY_DIR}/roboplan_coreConfig.cmake" @ONLY)

# Create and install the configuration files
include(CMakePackageConfigHelpers)
configure_package_config_file(
  cmake/roboplanConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/roboplan_coreConfig.cmake
  INSTALL_DESTINATION lib/cmake/roboplan_core)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/roboplan_coreConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/roboplan_coreConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/roboplan_coreConfigVersion.cmake"
        DESTINATION "lib/cmake/roboplan_core")

install(
  EXPORT roboplan-targets
  FILE roboplanTargets.cmake
  NAMESPACE roboplan::
  DESTINATION lib/cmake/roboplan_core)

install(
  TARGETS roboplan
  EXPORT roboplan
  LIBRARY DESTINATION lib
  INCLUDES
  DESTINATION include)

# Build the Python bindings by default (for the scikit-build wheel, colcon, and
# pixi). Override with -DBUILD_PYTHON_BINDINGS=OFF for a pure C++ build.
option(BUILD_PYTHON_BINDINGS "Build the roboplan Python bindings" ON)
if(BUILD_PYTHON_BINDINGS)
  add_subdirectory(bindings)
endif()

# Add the unit tests, if enabled.
option(BUILD_TESTING "" ON)
if(BUILD_TESTING)
  enable_testing()
  add_subdirectory(test)
endif()

# If ament_cmake is detected, export the package to be visible in ROS 2.
if(ament_cmake_FOUND)
  message(STATUS "ament_cmake found. Exporting for ROS 2.")
  ament_package()
endif()
