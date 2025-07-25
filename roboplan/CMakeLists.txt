cmake_minimum_required(VERSION 3.22)
project(roboplan VERSION 0.0.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies.
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)

# Add libraries
add_library(expected INTERFACE)
target_include_directories(expected
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

add_library(roboplan SHARED
    src/core/path_utils.cpp
    src/core/scene.cpp
    src/core/types.cpp
)
target_include_directories(roboplan PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(roboplan
    Eigen3::Eigen
    pinocchio::pinocchio
)
set_property(TARGET roboplan PROPERTY CXX_STANDARD 20)

install(
  TARGETS
  expected
  roboplan
  EXPORT roboplan-targets
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Generate the package configuration file
configure_file(
    "cmake/roboplanConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/roboplanConfig.cmake"
    @ONLY
)

# Create and install the configuration files
include(CMakePackageConfigHelpers)
configure_package_config_file(
    cmake/roboplanConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/roboplanConfig.cmake
    INSTALL_DESTINATION lib/cmake/roboplan
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/roboplanConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/roboplanConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/roboplanConfigVersion.cmake"
    DESTINATION
        "lib/cmake/roboplan"
)

# Install the headers and targets
install(DIRECTORY
            include/roboplan
        DESTINATION lib/cmake/)

install(EXPORT roboplan-targets
        FILE roboplanTargets.cmake
        NAMESPACE roboplan::
        DESTINATION lib/cmake/roboplan)

install(TARGETS roboplan
        EXPORT roboplan
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
find_package(ament_cmake QUIET)
if ( ament_cmake_FOUND )
    message(STATUS "ament_cmake found. Exporting for ROS 2.")
    ament_package()
endif()
