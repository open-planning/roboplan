cmake_minimum_required(VERSION 3.22)
project(roboplan VERSION 0.0.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies.
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(pinocchio REQUIRED)

# Add libraries
add_library (roboplan SHARED
    src/scene.cpp
    src/types.cpp
)
target_include_directories(roboplan PRIVATE include)
target_link_libraries(roboplan
    Eigen3::Eigen
    pinocchio::pinocchio
)
set_property(TARGET roboplan PROPERTY CXX_STANDARD 20)

install(
  TARGETS
  roboplan
  EXPORT roboplan-targets
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Build and install the executable.
add_executable(test_scene examples/test_scene.cpp)
target_include_directories(test_scene PRIVATE include)
target_link_libraries(test_scene
    Eigen3::Eigen
    pinocchio::pinocchio
    roboplan
)

install(
  TARGETS
  test_scene
  DESTINATION lib/roboplan
)

# TEST EXPORT

# Generate the package configuration file
configure_file(
    "cmake/roboplanConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/roboplanConfig.cmake"
    @ONLY
)

# Create and install the config files
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

# Install the configuration files
install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/roboplanConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/roboplanConfigVersion.cmake"
    DESTINATION
        "lib/cmake/roboplan"
)

# Install the headers and targets
install(FILES
            include/roboplan/scene.hpp
            include/roboplan/types.hpp
        DESTINATION lib/cmake/roboplan)

install(EXPORT roboplan-targets
        FILE roboplanTargets.cmake
        NAMESPACE roboplan::
        DESTINATION lib/cmake/roboplan)

install(TARGETS roboplan
        EXPORT roboplan
        LIBRARY DESTINATION lib
        INCLUDES DESTINATION include
        )

# If ament_cmake is detected, export the package to be visible in ROS 2.
find_package(ament_cmake QUIET)
if ( ament_cmake_FOUND )

    message(STATUS "ament_cmake found. Exporting for ROS 2.")
    ament_package()

endif()
