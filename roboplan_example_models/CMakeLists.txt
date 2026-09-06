cmake_minimum_required(VERSION 3.22)
project(roboplan_example_models VERSION 0.6.1)

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

# Find dependencies.

# Ament CMake is not immediately required, but we include it at the top so that
# if it is found we can use it in build testing before requiring a
# find_package(ament_cmake_test REQUIRED). Otherwise this can cause infinite recursion
# issues with symlink installs.
find_package(ament_cmake QUIET)

# dladdr() in src/resources.cpp comes from dlfcn.h, which is POSIX-only.
# On Windows the dlfcn-win32 library provides a compatible implementation.
if(WIN32)
  find_package(dlfcn-win32 REQUIRED)
endif()

# Build and install the library containing the resource paths.
add_library(roboplan_example_models SHARED
    src/resources.cpp
    src/anchor.cpp
)
if(WIN32)
  target_link_libraries(roboplan_example_models PRIVATE dlfcn-win32::dl)
endif()
# Forward slashes keep the define a valid C string literal on Windows.
file(TO_CMAKE_PATH "${CMAKE_INSTALL_PREFIX}" roboplan_install_prefix)
target_compile_definitions(roboplan_example_models PRIVATE
  ROBOPLAN_INSTALL_PREFIX="${roboplan_install_prefix}")
target_include_directories(roboplan_example_models PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
set_property(TARGET roboplan_example_models PROPERTY CXX_STANDARD 20)
# Alias so this works via add_subdirectory() too, not just installed find_package().
add_library(roboplan_example_models::roboplan_example_models ALIAS roboplan_example_models)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS
  roboplan_example_models
  EXPORT roboplan_example_models-targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Create and install the configuration files
configure_file(
    "cmake/roboplan_example_modelsConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/roboplan_example_modelsConfig.cmake"
    @ONLY
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
    cmake/roboplan_example_modelsConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/roboplan_example_modelsConfig.cmake
    INSTALL_DESTINATION lib/cmake/roboplan_example_models
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/roboplan_example_modelsConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/roboplan_example_modelsConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/roboplan_example_modelsConfigVersion.cmake"
    DESTINATION
        "lib/cmake/roboplan_example_models"
)

# Install the headers and targets
install(DIRECTORY
            include/roboplan_example_models
        DESTINATION lib/cmake/)

install(EXPORT roboplan_example_models-targets
        FILE roboplan_example_modelsTargets.cmake
        NAMESPACE roboplan_example_models::
        DESTINATION lib/cmake/roboplan_example_models)

install(TARGETS roboplan_example_models
        EXPORT roboplan_example_models
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
        )

# Install all the models.
install(
  DIRECTORY
  models
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/roboplan_example_models
)

# Build the Python bindings by default (for the scikit-build wheel, colcon, and
# pixi). Override with -DBUILD_PYTHON_BINDINGS=OFF for a pure C++ build.
option(BUILD_PYTHON_BINDINGS "Build the roboplan_example_models Python bindings" ON)
if(BUILD_PYTHON_BINDINGS)
  add_subdirectory(bindings)
endif()

# If ament_cmake is detected, export the package to be visible in ROS 2.
if ( ament_cmake_FOUND )
    message(STATUS "ament_cmake found. Exporting for ROS 2.")
    ament_package()
endif()
