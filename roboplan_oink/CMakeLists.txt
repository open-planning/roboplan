cmake_minimum_required(VERSION 3.22)
project(roboplan_oink VERSION 0.1.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Include FetchContent for downloading dependencies
include(FetchContent)

# Find dependencies.
find_package(roboplan REQUIRED)

# Try to find OsqpEigen
find_package(OsqpEigen QUIET)

if(NOT OsqpEigen_FOUND)
  # When building OsqpEigen from source, we need OSQP >= 1.0.0
  find_package(osqp QUIET)

  if(NOT osqp_FOUND OR osqp_VERSION VERSION_LESS "1.0.0")
    message(STATUS "OSQP not found or version < 1.0.0. Fetching from GitHub...")

    # Disable OSQP tests and demos
    set(OSQP_BUILD_DEMO OFF CACHE INTERNAL "Disable OSQP demo")
    set(OSQP_BUILD_UNITTESTS OFF CACHE INTERNAL "Disable OSQP tests")

    # Fetch OSQP from GitHub
    FetchContent_Declare(
      osqp
      GIT_REPOSITORY https://github.com/osqp/osqp.git
      GIT_TAG v1.0.0
      GIT_SHALLOW TRUE
    )
    FetchContent_MakeAvailable(osqp)

    # OSQP creates non-namespaced targets when built via FetchContent
    # Create namespaced aliases so OsqpEigen can find them
    if(TARGET osqp AND NOT TARGET osqp::osqp)
      add_library(osqp::osqp ALIAS osqp)
    endif()
    if(TARGET osqpstatic AND NOT TARGET osqp::osqpstatic)
      add_library(osqp::osqpstatic ALIAS osqpstatic)
    endif()

    # Set OSQP version detection variables to skip OsqpEigen's try_compile tests
    # (try_compile doesn't work with ALIAS targets from FetchContent)
    set(OSQP_IS_V1 TRUE CACHE BOOL "OSQP is version 1.0+" FORCE)
    set(OSQP_IS_V1_FINAL TRUE CACHE BOOL "OSQP is version 1.0.0 final" FORCE)
  endif()

  message(STATUS "OsqpEigen not found. Fetching from GitHub...")

  set(OSQPEIGEN_COMPILE_tests OFF CACHE INTERNAL "Disable OsqpEigen tests")

  # Fetch osqp-eigen from GitHub (using gbionics fork as per original .gitmodules)
  FetchContent_Declare(
    osqp-eigen
    GIT_REPOSITORY https://github.com/gbionics/osqp-eigen.git
    GIT_TAG v0.11.0
    GIT_SHALLOW TRUE
  )
  FetchContent_MakeAvailable(osqp-eigen)
endif()

# Ament CMake is not immediately required, but we include it at the top so that
# if it is found we can use it in build testing before requiring a
# find_package(ament_cmake_test REQUIRED). Otherwise this can cause infinite recursion
# issues with symlink installs.
find_package(ament_cmake QUIET)

# Add libraries
add_library(roboplan_oink SHARED
    src/optimal_ik.cpp
    src/tasks/frame.cpp
    src/tasks/configuration.cpp
    src/constraints/position_limit.cpp
    src/constraints/velocity_limit.cpp
)
target_include_directories(roboplan_oink PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(roboplan_oink
    roboplan::roboplan
    OsqpEigen::OsqpEigen
)
set_property(TARGET roboplan_oink PROPERTY CXX_STANDARD 20)

install(
  TARGETS
  roboplan_oink
  EXPORT roboplan_oink-targets
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Generate the package configuration file
configure_file(
    "cmake/roboplan_oinkConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/roboplan_oinkConfig.cmake"
    @ONLY
)

# Create and install the configuration files
include(CMakePackageConfigHelpers)
configure_package_config_file(
    cmake/roboplan_oinkConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/roboplan_oinkConfig.cmake
    INSTALL_DESTINATION lib/cmake/roboplan_oink
)
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/roboplan_oinkConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
)
install(
    FILES
        "${CMAKE_CURRENT_BINARY_DIR}/roboplan_oinkConfig.cmake"
        "${CMAKE_CURRENT_BINARY_DIR}/roboplan_oinkConfigVersion.cmake"
    DESTINATION
        "lib/cmake/roboplan_oink"
)

# Install the headers and targets
install(DIRECTORY
            include/roboplan_oink
        DESTINATION lib/cmake/)

install(EXPORT roboplan_oink-targets
        FILE roboplan_oinkTargets.cmake
        NAMESPACE roboplan_oink::
        DESTINATION lib/cmake/roboplan_oink)

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
