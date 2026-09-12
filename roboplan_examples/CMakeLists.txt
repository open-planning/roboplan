cmake_minimum_required(VERSION 3.22)
project(roboplan_examples VERSION 0.6.1)

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND CMAKE_CXX_COMPILER_FRONTEND_VARIANT
                                             STREQUAL "MSVC")
  # clang-cl remaps plain -Wall to MSVC /Wall (clang's -Weverything); use the
  # /clang: prefix to pass the GNU-style flags through unchanged.
  add_compile_options(/clang:-Wall /clang:-Wextra /clang:-Wpedantic)
elseif(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies.
if(NOT TARGET roboplan::roboplan)
  find_package(roboplan REQUIRED)
endif()
if(NOT TARGET roboplan_example_models::roboplan_example_models)
  find_package(roboplan_example_models REQUIRED)
endif()
if(NOT TARGET roboplan_simple_ik::roboplan_simple_ik)
  find_package(roboplan_simple_ik REQUIRED)
endif()
if(NOT TARGET roboplan_oink::roboplan_oink)
  find_package(roboplan_oink REQUIRED)
endif()
if(NOT TARGET roboplan_rrt::roboplan_rrt)
  find_package(roboplan_rrt REQUIRED)
endif()
if(NOT TARGET roboplan_toppra::roboplan_toppra)
  find_package(roboplan_toppra REQUIRED)
endif()
if(NOT TARGET roboplan_cartesian_planning::roboplan_cartesian_planning)
  find_package(roboplan_cartesian_planning REQUIRED)
endif()

# Ament CMake is not immediately required, but we include it at the top so that
# if it is found we can use it in build testing before requiring a
# find_package(ament_cmake_test REQUIRED). Otherwise this can cause infinite
# recursion issues with symlink installs.
find_package(ament_cmake QUIET)

# Build and install the examples.
add_subdirectory(cpp)
add_subdirectory(python)

# If ament_cmake is detected, export the package to be visible in ROS 2.
if(ament_cmake_FOUND)
  message(STATUS "ament_cmake found. Exporting for ROS 2.")
  if(BUILD_TESTING)
    add_subdirectory(test)
  endif()
  ament_package()
endif()
