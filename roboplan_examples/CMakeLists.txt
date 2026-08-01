cmake_minimum_required(VERSION 3.22)
project(roboplan_examples VERSION 0.6.0)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
  # clang-cl remaps plain -Wall to MSVC /Wall (clang's -Weverything); use the
  # /clang: prefix to pass the GNU-style flags through unchanged.
  add_compile_options(/clang:-Wall /clang:-Wextra /clang:-Wpedantic)
elseif(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies.
find_package(roboplan REQUIRED)
find_package(roboplan_example_models REQUIRED)
find_package(roboplan_simple_ik REQUIRED)
find_package(roboplan_oink REQUIRED)
find_package(roboplan_rrt REQUIRED)
find_package(roboplan_toppra REQUIRED)
find_package(roboplan_cartesian_planning REQUIRED)

# Ament CMake is not immediately required, but we include it at the top so that
# if it is found we can use it in build testing before requiring a
# find_package(ament_cmake_test REQUIRED). Otherwise this can cause infinite recursion
# issues with symlink installs.
find_package(ament_cmake QUIET)

# Build and install the examples.
add_subdirectory(cpp)
add_subdirectory(python)

# If ament_cmake is detected, export the package to be visible in ROS 2.
if ( ament_cmake_FOUND )
    message(STATUS "ament_cmake found. Exporting for ROS 2.")
    if(BUILD_TESTING)
        add_subdirectory(test)
    endif()
    ament_package()
endif()
