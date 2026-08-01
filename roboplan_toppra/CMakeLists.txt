cmake_minimum_required(VERSION 3.22)
project(roboplan_toppra VERSION 0.6.0)

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
find_package(roboplan REQUIRED)

# Ament CMake is not immediately required, but we include it at the top so that
# if it is found we can use it in build testing before requiring a
# find_package(ament_cmake_test REQUIRED). Otherwise this can cause infinite recursion
# issues with symlink installs.
find_package(ament_cmake QUIET)

# If toppra is not found, fetch it from GitHub.
# The find is skipped on Windows: the conda-forge win-64 libtoppra package is
# missing its import library, and its broken CMake export aborts the
# configure, so toppra is always built from source there.
include(FetchContent)
if(NOT WIN32)
    find_package(toppra QUIET)
endif()
if(NOT toppra_FOUND)
    message(STATUS "toppra not found. Fetching from GitHub...")
    set(BUILD_TESTS OFF CACHE BOOL "Disable toppra tests" FORCE)
    set(PYTHON_BINDINGS OFF CACHE BOOL "Disable toppra Python bindings" FORCE)
    # On Windows, build toppra statically: its install rules place the DLL in
    # lib/, which is not on the loader search path.
    # This can be removed when https://github.com/hungpham2511/toppra/pull/294 is released.
    if(WIN32)
        set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
    endif()
    FetchContent_Declare(
        toppra
        GIT_REPOSITORY https://github.com/hungpham2511/toppra.git
        GIT_TAG 0.6.8
        SOURCE_SUBDIR cpp
    )
    # toppra maps the deprecated BUILD_TESTS variable onto BUILD_TESTING with
    # a forced cache write, which would silently disable this package's own
    # tests below, so restore the previous value afterwards.
    if(DEFINED CACHE{BUILD_TESTING})
        set(roboplan_prev_build_testing "$CACHE{BUILD_TESTING}")
    else()
        set(roboplan_prev_build_testing ON)
    endif()
    FetchContent_MakeAvailable(toppra)
    set(BUILD_TESTING "${roboplan_prev_build_testing}" CACHE BOOL "" FORCE)

    # toppra creates non-namespaced targets when built via FetchContent,
    # so we will make an alias to correct this.
    if(TARGET toppra AND NOT TARGET toppra::toppra)
        add_library(toppra::toppra ALIAS toppra)
    endif()
endif()

# Add libraries
add_library(roboplan_toppra SHARED
    src/toppra.cpp
    src/linear_blend_path.cpp
)
target_include_directories(roboplan_toppra PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
set_property(TARGET roboplan_toppra PROPERTY CXX_STANDARD 20)
target_link_libraries(roboplan_toppra
    roboplan::roboplan
    toppra::toppra
)

install(
  TARGETS
  roboplan_toppra
  EXPORT roboplan_toppra-targets
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Generate the package configuration file
configure_file(
    "cmake/roboplan_toppraConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/roboplan_toppraConfig.cmake"
    @ONLY
)

# Create and install the configuration files
include(CMakePackageConfigHelpers)
configure_package_config_file(
    cmake/roboplan_toppraConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/roboplan_toppraConfig.cmake
    INSTALL_DESTINATION lib/cmake/roboplan_toppra
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/roboplan_toppraConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/roboplan_toppraConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/roboplan_toppraConfigVersion.cmake"
    DESTINATION
        "lib/cmake/roboplan_toppra"
)

# Install the headers and targets
install(DIRECTORY
            include/roboplan_toppra
        DESTINATION lib/cmake/)

install(EXPORT roboplan_toppra-targets
        FILE roboplan_toppraTargets.cmake
        NAMESPACE roboplan_toppra::
        DESTINATION lib/cmake/roboplan_toppra)

install(TARGETS roboplan_toppra
        EXPORT roboplan_toppra
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
        )

# Add the unit tests, if enabled.
option(BUILD_TESTING "" ON)
if (BUILD_TESTING)
    enable_testing()
    add_subdirectory(test)
endif()

# Build the Python bindings by default (for the scikit-build wheel, colcon, and
# pixi). Override with -DBUILD_PYTHON_BINDINGS=OFF for a pure C++ build.
option(BUILD_PYTHON_BINDINGS "Build the roboplan_toppra Python bindings" ON)
if(BUILD_PYTHON_BINDINGS)
  add_subdirectory(bindings)
endif()

# If ament_cmake is detected, export the package to be visible in ROS 2.
if ( ament_cmake_FOUND )
    message(STATUS "ament_cmake found. Exporting for ROS 2.")
    ament_package()
endif()
