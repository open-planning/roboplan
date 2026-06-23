find_package(GTest REQUIRED)
find_package(roboplan_example_models REQUIRED)

# Build tests
add_executable(test_simple_ik test_simple_ik.cpp)
set_property(TARGET test_simple_ik PROPERTY CXX_STANDARD 20)
target_link_libraries(
  test_simple_ik
  roboplan_simple_ik
  roboplan::roboplan
  roboplan_example_models::roboplan_example_models
  GTest::gtest_main
)

# If ament_gtest is detected, make the tests visible in ROS 2.
find_package(ament_cmake_gtest QUIET)
if ( ament_cmake_gtest_FOUND )
    ament_add_gtest_test(test_simple_ik)
else()
    include(GoogleTest)
    gtest_add_tests(TARGET test_simple_ik)
endif()
