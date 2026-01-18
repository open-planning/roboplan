cmake_minimum_required(VERSION 3.22)
project(roboplan_toppra VERSION 0.1.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies.
find_package(roboplan REQUIRED)
find_package(toppra REQUIRED)

# Ament CMake is not immediately required, but we include it at the top so that
# if it is found we can use it in build testing before requiring a
# find_package(ament_cmake_test REQUIRED). Otherwise this can cause infinite recursion
# issues with symlink installs.
find_package(ament_cmake QUIET)

# Add libraries
add_library(roboplan_toppra SHARED
    src/toppra.cpp
)
target_include_directories(roboplan_toppra PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(roboplan_toppra
    roboplan::expected
    roboplan::roboplan
    toppra::toppra
)
set_property(TARGET roboplan_toppra PROPERTY CXX_STANDARD 20)

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

# If ament_cmake is detected, export the package to be visible in ROS 2.
if ( ament_cmake_FOUND )
    message(STATUS "ament_cmake found. Exporting for ROS 2.")
    ament_package()
endif()
