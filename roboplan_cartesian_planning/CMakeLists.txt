cmake_minimum_required(VERSION 3.22)
project(roboplan_cartesian_planning VERSION 0.5.1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies.
find_package(roboplan REQUIRED)
find_package(roboplan_oink REQUIRED)
find_package(roboplan_toppra REQUIRED)

# Ament CMake is not immediately required, but we include it at the top so that
# if it is found we can use it in build testing before requiring a
# find_package(ament_cmake_test REQUIRED). Otherwise this can cause infinite recursion
# issues with symlink installs.
find_package(ament_cmake QUIET)

# Add libraries
add_library(roboplan_cartesian_planning SHARED
    src/cartesian_path_planner.cpp
)
target_include_directories(roboplan_cartesian_planning PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
set_property(TARGET roboplan_cartesian_planning PROPERTY CXX_STANDARD 20)
target_link_libraries(roboplan_cartesian_planning
    roboplan::roboplan
    roboplan_oink::roboplan_oink
    roboplan_toppra::roboplan_toppra
)

install(
  TARGETS
  roboplan_cartesian_planning
  EXPORT roboplan_cartesian_planning-targets
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Generate the package configuration file
configure_file(
    "cmake/roboplan_cartesian_planningConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/roboplan_cartesian_planningConfig.cmake"
    @ONLY
)

# Create and install the configuration files
include(CMakePackageConfigHelpers)
configure_package_config_file(
    cmake/roboplan_cartesian_planningConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/roboplan_cartesian_planningConfig.cmake
    INSTALL_DESTINATION lib/cmake/roboplan_cartesian_planning
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/roboplan_cartesian_planningConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/roboplan_cartesian_planningConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/roboplan_cartesian_planningConfigVersion.cmake"
    DESTINATION
        "lib/cmake/roboplan_cartesian_planning"
)

# Install the headers and targets
install(DIRECTORY
            include/roboplan_cartesian_planning
        DESTINATION lib/cmake/)

install(EXPORT roboplan_cartesian_planning-targets
        FILE roboplan_cartesian_planningTargets.cmake
        NAMESPACE roboplan_cartesian_planning::
        DESTINATION lib/cmake/roboplan_cartesian_planning)

install(TARGETS roboplan_cartesian_planning
        EXPORT roboplan_cartesian_planning
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
option(BUILD_PYTHON_BINDINGS "Build the roboplan_cartesian_planning Python bindings" ON)
if(BUILD_PYTHON_BINDINGS)
  add_subdirectory(bindings)
endif()

# If ament_cmake is detected, export the package to be visible in ROS 2.
if ( ament_cmake_FOUND )
    message(STATUS "ament_cmake found. Exporting for ROS 2.")
    ament_package()
endif()
