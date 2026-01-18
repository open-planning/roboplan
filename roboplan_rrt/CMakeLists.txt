cmake_minimum_required(VERSION 3.22)
project(roboplan_rrt VERSION 0.1.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies.
find_package(roboplan REQUIRED)

# Ament CMake is not immediately required, but we include it at the top so that
# if it is found we can use it in build testing before requiring a
# find_package(ament_cmake_test REQUIRED). Otherwise this can cause infinite recursion
# issues with symlink installs.
find_package(ament_cmake QUIET)

# Add libraries
add_library(roboplan_rrt SHARED
    src/rrt.cpp
)
target_include_directories(roboplan_rrt PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(roboplan_rrt
    roboplan::expected
    roboplan::roboplan
)
set_property(TARGET roboplan_rrt PROPERTY CXX_STANDARD 20)

install(
  TARGETS
  roboplan_rrt
  EXPORT roboplan_rrt-targets
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Generate the package configuration file
configure_file(
    "cmake/roboplan_rrtConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/roboplan_rrtConfig.cmake"
    @ONLY
)

# Create and install the configuration files
include(CMakePackageConfigHelpers)
configure_package_config_file(
    cmake/roboplan_rrtConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/roboplan_rrtConfig.cmake
    INSTALL_DESTINATION lib/cmake/roboplan_rrt
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/roboplan_rrtConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/roboplan_rrtConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/roboplan_rrtConfigVersion.cmake"
    DESTINATION
        "lib/cmake/roboplan_rrt"
)

# Install the headers and targets
install(DIRECTORY
            include/roboplan_rrt
        DESTINATION lib/cmake/)

install(EXPORT roboplan_rrt-targets
        FILE roboplan_rrtTargets.cmake
        NAMESPACE roboplan_rrt::
        DESTINATION lib/cmake/roboplan_rrt)

install(TARGETS roboplan_rrt
        EXPORT roboplan_rrt
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
        )

# Add the unit tests, if enabled.
option(BUILD_TESTING "" ON)
if (BUILD_TESTING)
    enable_testing()
    add_subdirectory(test)
endif()

# If ament_cmake is detected, export the package to be visible in ROS 2.
if ( ament_cmake_FOUND )
    message(STATUS "ament_cmake found. Exporting for ROS 2.")
    ament_package()
endif()
