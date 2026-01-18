cmake_minimum_required(VERSION 3.22)
project(roboplan VERSION 0.1.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies.
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(yaml-cpp REQUIRED)

# Ament CMake is not immediately required, but we include it at the top so that
# if it is found we can use it in build testing before requiring a
# find_package(ament_cmake_test REQUIRED). Otherwise this can cause infinite recursion
# issues with symlink installs.
find_package(ament_cmake QUIET)

# Add libraries
add_library(expected INTERFACE)
target_include_directories(expected
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# ROS already has a TinyXML2 vendor, so add this only in non-ROS contexts.
find_package(tinyxml2_vendor QUIET)
find_package(TinyXML2 QUIET)
if ( NOT tinyxml2_vendor_FOUND AND NOT TinyXML2_FOUND )
    add_library(tinyxml2
        include/tinyxml2/tinyxml2.cpp
        include/tinyxml2/tinyxml2.h)
    add_library(tinyxml2::tinyxml2 ALIAS tinyxml2)
endif()

add_library(roboplan SHARED
    src/core/path_utils.cpp
    src/core/scene.cpp
    src/core/scene_utils.cpp
    src/core/types.cpp
)
target_include_directories(roboplan PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(roboplan
    PUBLIC
    Eigen3::Eigen
    pinocchio::pinocchio
    tinyxml2::tinyxml2
)
if (yaml-cpp_VERSION VERSION_LESS "0.8")  # for Ubuntu 22.04
    target_link_libraries(roboplan PRIVATE yaml-cpp)
else()
    target_link_libraries(roboplan PRIVATE yaml-cpp::yaml-cpp)
endif()
set_property(TARGET roboplan PROPERTY CXX_STANDARD 20)

if ( NOT tinyxml2_vendor_FOUND AND NOT TinyXML2_FOUND )
    set(ROBOPLAN_TARGETS expected tinyxml2 roboplan)
else()
    set(ROBOPLAN_TARGETS expected roboplan)
endif()

install(
  TARGETS
  ${ROBOPLAN_TARGETS}
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
if ( ament_cmake_FOUND )
    message(STATUS "ament_cmake found. Exporting for ROS 2.")
    ament_package()
endif()
