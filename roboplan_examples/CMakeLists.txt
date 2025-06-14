cmake_minimum_required(VERSION 3.22)
project(roboplan_examples VERSION 0.0.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies.
find_package(roboplan REQUIRED)
find_package(roboplan_simple_ik REQUIRED)

# Build and install the examples.
add_executable(test_scene examples/test_scene.cpp)
target_link_libraries(test_scene
    roboplan::roboplan
)
set_property(TARGET test_scene PROPERTY CXX_STANDARD 20)

add_executable(test_ik examples/test_ik.cpp)
target_link_libraries(test_ik
    roboplan::roboplan
    roboplan_simple_ik::roboplan_simple_ik
)
set_property(TARGET test_ik PROPERTY CXX_STANDARD 20)

install(
  TARGETS
  test_scene
  test_ik
  DESTINATION lib/roboplan_examples
)

# If ament_cmake is detected, export the package to be visible in ROS 2.
find_package(ament_cmake QUIET)
if ( ament_cmake_FOUND )

    message(STATUS "ament_cmake found. Exporting for ROS 2.")
    ament_package()

endif()
