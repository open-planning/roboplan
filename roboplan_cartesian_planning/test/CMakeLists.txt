find_package(GTest REQUIRED)
if(NOT TARGET roboplan_example_models::roboplan_example_models)
  find_package(roboplan_example_models REQUIRED)
endif()

# Build tests
add_executable(test_cartesian_path_planner test_cartesian_path_planner.cpp)
set_property(TARGET test_cartesian_path_planner PROPERTY CXX_STANDARD 20)
target_link_libraries(
  test_cartesian_path_planner roboplan_cartesian_planning
  roboplan_example_models::roboplan_example_models GTest::gtest_main)

# If ament_gtest is detected, make the tests visible in ROS 2.
find_package(ament_cmake_gtest QUIET)
if(ament_cmake_gtest_FOUND)
  ament_add_gtest_test(test_cartesian_path_planner)
else()
  include(GoogleTest)
  gtest_add_tests(TARGET test_cartesian_path_planner)
endif()
