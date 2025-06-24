find_package(GTest REQUIRED)
find_package(roboplan_example_models REQUIRED)

# Build tests
add_executable(test_rrt test_rrt.cpp)
target_link_libraries(
  test_rrt
  roboplan_rrt
  roboplan::roboplan
  roboplan_example_models::roboplan_example_models
  GTest::gtest_main
)

# TODO: Define tests as a variable

# If ament_gtest is detected, make the tests visible in ROS 2.
find_package(ament_cmake_gtest QUIET)
if ( ament_cmake_gtest_FOUND )
    # See https://github.com/ament/ament_cmake/issues/588
    find_package(ament_cmake_test REQUIRED)

    ament_add_gtest_test(test_rrt)
else()
    include(GoogleTest)
    gtest_discover_tests(test_rrt)
endif()
