find_package(GTest REQUIRED)
if(NOT TARGET roboplan_example_models::roboplan_example_models)
  find_package(roboplan_example_models REQUIRED)
endif()

add_executable(test_vex_qp_backend test_qp_backend.cpp)
set_property(TARGET test_vex_qp_backend PROPERTY CXX_STANDARD 20)
target_link_libraries(test_vex_qp_backend PRIVATE roboplan_vex
                                                  GTest::gtest_main)

# If ament_gtest is detected, make the tests visible in ROS 2.
find_package(ament_cmake_gtest QUIET)
if(ament_cmake_gtest_FOUND)
  ament_add_gtest_test(test_vex_qp_backend)
else()
  include(GoogleTest)
  gtest_add_tests(TARGET test_vex_qp_backend)
endif()
