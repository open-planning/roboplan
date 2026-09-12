find_package(GTest REQUIRED)
if(NOT TARGET roboplan_example_models::roboplan_example_models)
  find_package(roboplan_example_models REQUIRED)
endif()

# Build tests
add_executable(test_toppra test_toppra.cpp)
set_property(TARGET test_toppra PROPERTY CXX_STANDARD 20)
target_link_libraries(
  test_toppra
  PRIVATE roboplan_toppra roboplan_example_models::roboplan_example_models
          GTest::gtest_main)

# TODO: Define tests as a variable

# If ament_gtest is detected, make the tests visible in ROS 2.
find_package(ament_cmake_gtest QUIET)
if(ament_cmake_gtest_FOUND)
  ament_add_gtest_test(test_toppra)
else()
  include(GoogleTest)
  gtest_add_tests(TARGET test_toppra)
endif()
