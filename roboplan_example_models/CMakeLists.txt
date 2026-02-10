cmake_minimum_required(VERSION 3.22)
project(roboplan_example_models VERSION 0.1.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies.

# Ament CMake is not immediately required, but we include it at the top so that
# if it is found we can use it in build testing before requiring a
# find_package(ament_cmake_test REQUIRED). Otherwise this can cause infinite recursion
# issues with symlink installs.
find_package(ament_cmake QUIET)

# Build and install the library containing the resource paths.
add_library(roboplan_example_models SHARED
    src/resources.cpp
    src/anchor.cpp
)
target_include_directories(roboplan_example_models PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
set_property(TARGET roboplan_example_models PROPERTY CXX_STANDARD 20)

install(DIRECTORY include/ DESTINATION include)
install(
  TARGETS
  roboplan_example_models
  EXPORT roboplan_example_models-targets
  LIBRARY DESTINATION lib
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
        INCLUDES DESTINATION include
        )

# Install all the models.
install(
  DIRECTORY
  models
  DESTINATION ${CMAKE_INSTALL_PREFIX}/share/roboplan_example_models
)

# If ament_cmake is detected, export the package to be visible in ROS 2.
if ( ament_cmake_FOUND )
    message(STATUS "ament_cmake found. Exporting for ROS 2.")
    ament_package()
endif()
