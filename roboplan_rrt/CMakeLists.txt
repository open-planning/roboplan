cmake_minimum_required(VERSION 3.22)
project(roboplan_rrt VERSION 0.7.0)

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
if(NOT TARGET roboplan::roboplan)
  find_package(roboplan_core REQUIRED)
endif()

# Ament CMake is not immediately required, but we include it at the top so that
# if it is found we can use it in build testing before requiring a
# find_package(ament_cmake_test REQUIRED). Otherwise this can cause infinite
# recursion issues with symlink installs.
find_package(ament_cmake QUIET)

# nanoflann: the system copy if it is new enough for the manifold metrics
# (conda-forge nanoflann or ROS nanoflann_vendor; Ubuntu's libnanoflann-dev is
# too old), otherwise fetched at configure time. Linked privately, so it is
# neither installed nor exported.
find_package(nanoflann 2.0.1 QUIET)
if(NOT TARGET nanoflann::nanoflann)
  message(STATUS "nanoflann >= 2.0.1 not found, fetching it")
  include(FetchContent)
  FetchContent_Declare(
    nanoflann
    GIT_REPOSITORY https://github.com/jlblancoc/nanoflann.git
    # The 2.0.1 release commit; switch to the v2.0.1 tag once upstream adds it.
    GIT_TAG 18f4edf753e813644a7a12711a86139777a8ef00
    # Only the header is needed. A subdirectory without a CMakeLists.txt skips
    # nanoflann's own project, with its tests, examples and install rules.
    SOURCE_SUBDIR
    include)
  FetchContent_MakeAvailable(nanoflann)
  add_library(nanoflann::nanoflann INTERFACE IMPORTED)
  target_include_directories(nanoflann::nanoflann
                             INTERFACE ${nanoflann_SOURCE_DIR}/include)
endif()

# Add libraries
add_library(roboplan_rrt SHARED src/constraints.cpp src/kd_tree.cpp src/rrt.cpp)
target_include_directories(
  roboplan_rrt PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                      $<INSTALL_INTERFACE:include>)
set_property(TARGET roboplan_rrt PROPERTY CXX_STANDARD 20)
target_link_libraries(
  roboplan_rrt
  PUBLIC roboplan::roboplan
  PRIVATE nanoflann::nanoflann)
# Alias so this works via add_subdirectory() too, not just installed
# find_package().
add_library(roboplan_rrt::roboplan_rrt ALIAS roboplan_rrt)

install(
  TARGETS roboplan_rrt
  EXPORT roboplan_rrt-targets
  LIBRARY DESTINATION lib
  INCLUDES
  DESTINATION include)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Generate the package configuration file
configure_file("cmake/roboplan_rrtConfig.cmake.in"
               "${CMAKE_CURRENT_BINARY_DIR}/roboplan_rrtConfig.cmake" @ONLY)

# Create and install the configuration files
include(CMakePackageConfigHelpers)
configure_package_config_file(
  cmake/roboplan_rrtConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/roboplan_rrtConfig.cmake
  INSTALL_DESTINATION lib/cmake/roboplan_rrt)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/roboplan_rrtConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY AnyNewerVersion)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/roboplan_rrtConfig.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/roboplan_rrtConfigVersion.cmake"
        DESTINATION "lib/cmake/roboplan_rrt")

# Install the headers and targets
install(DIRECTORY include/roboplan_rrt DESTINATION lib/cmake/)

install(
  EXPORT roboplan_rrt-targets
  FILE roboplan_rrtTargets.cmake
  NAMESPACE roboplan_rrt::
  DESTINATION lib/cmake/roboplan_rrt)

install(
  TARGETS roboplan_rrt
  EXPORT roboplan_rrt
  LIBRARY DESTINATION lib
  INCLUDES
  DESTINATION include)

# Build the Python bindings by default (for the cmeel wheel, colcon, and pixi).
# Override with -DBUILD_PYTHON_BINDINGS=OFF for a pure C++ build.
option(BUILD_PYTHON_BINDINGS "Build the roboplan_rrt Python bindings" ON)
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
