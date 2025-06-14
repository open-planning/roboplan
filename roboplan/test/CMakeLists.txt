enable_testing()
find_package(GTest REQUIRED)

add_executable(
  test_types
  test_types.cpp
)
target_include_directories(test_types PRIVATE ../include)
target_link_libraries(
  test_types
  roboplan
  GTest::gtest_main
)

include(GoogleTest)
gtest_discover_tests(test_types)

# If ament_gtest is detected, make the tests visible in ROS 2.
find_package(ament_cmake_gtest QUIET)
if ( ament_cmake_gtest_FOUND )
    ament_add_gtest_test(test_types)
endif()
