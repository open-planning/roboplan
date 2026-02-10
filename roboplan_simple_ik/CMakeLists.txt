cmake_minimum_required(VERSION 3.22)
project(roboplan_simple_ik VERSION 0.1.0)

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
add_library(roboplan_simple_ik SHARED
    src/simple_ik.cpp
)
target_include_directories(roboplan_simple_ik PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(roboplan_simple_ik
    roboplan::roboplan
)
set_property(TARGET roboplan_simple_ik PROPERTY CXX_STANDARD 20)

install(
  TARGETS
  roboplan_simple_ik
  EXPORT roboplan_simple_ik-targets
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Generate the package configuration file
configure_file(
    "cmake/roboplan_simple_ikConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/roboplan_simple_ikConfig.cmake"
    @ONLY
)

# Create and install the configuration files
include(CMakePackageConfigHelpers)
configure_package_config_file(
    cmake/roboplan_simple_ikConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/roboplan_simple_ikConfig.cmake
    INSTALL_DESTINATION lib/cmake/roboplan_simple_ik
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/roboplan_simple_ikConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/roboplan_simple_ikConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/roboplan_simple_ikConfigVersion.cmake"
    DESTINATION
        "lib/cmake/roboplan_simple_ik"
)

# Install the headers and targets
install(DIRECTORY
            include/roboplan_simple_ik
        DESTINATION lib/cmake/)

install(EXPORT roboplan_simple_ik-targets
        FILE roboplan_simple_ikTargets.cmake
        NAMESPACE roboplan_simple_ik::
        DESTINATION lib/cmake/roboplan_simple_ik)

install(TARGETS roboplan_simple_ik
        EXPORT roboplan_simple_ik
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
        )

# If ament_cmake is detected, export the package to be visible in ROS 2.
if ( ament_cmake_FOUND )
    message(STATUS "ament_cmake found. Exporting for ROS 2.")
    ament_package()
endif()
