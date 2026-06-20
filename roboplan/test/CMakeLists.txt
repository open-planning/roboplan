find_package(GTest REQUIRED)
find_package(roboplan_example_models REQUIRED)

# Build tests
add_executable(test_types test_types.cpp)
set_property(TARGET test_types PROPERTY CXX_STANDARD 20)
target_link_libraries(
  test_types
  roboplan
  GTest::gtest_main
)

add_executable(test_scene test_scene.cpp)
set_property(TARGET test_scene PROPERTY CXX_STANDARD 20)
target_link_libraries(
  test_scene
  roboplan
  GTest::gmock_main
  GTest::gtest_main
  roboplan_example_models::roboplan_example_models
)

add_executable(test_joints test_joints.cpp)
set_property(TARGET test_joints PROPERTY CXX_STANDARD 20)
target_link_libraries(
  test_joints
  roboplan
  GTest::gmock_main
  GTest::gtest_main
)

add_executable(test_path_utils test_path_utils.cpp)
set_property(TARGET test_path_utils PROPERTY CXX_STANDARD 20)
target_link_libraries(
  test_path_utils
  roboplan
  GTest::gtest_main
  roboplan_example_models::roboplan_example_models
)

add_executable(test_se3_low_pass_filter test_se3_low_pass_filter.cpp)
set_property(TARGET test_se3_low_pass_filter PROPERTY CXX_STANDARD 20)
target_link_libraries(
  test_se3_low_pass_filter
  filters
  GTest::gtest_main
)

add_executable(test_urdf_extended_limits test_urdf_extended_limits.cpp)
set_property(TARGET test_urdf_extended_limits PROPERTY CXX_STANDARD 20)
target_link_libraries(
  test_urdf_extended_limits
  roboplan
  GTest::gtest_main
)

# TODO: Define tests as a variable

# If ament_gtest is detected, make the tests visible in ROS 2.
find_package(ament_cmake_gtest QUIET)
if ( ament_cmake_gtest_FOUND )
    ament_add_gtest_test(test_types)
    ament_add_gtest_test(test_scene)
    ament_add_gtest_test(test_joints)
    ament_add_gtest_test(test_path_utils)
    ament_add_gtest_test(test_se3_low_pass_filter)
    ament_add_gtest_test(test_urdf_extended_limits)
else()
    include(GoogleTest)
    gtest_add_tests(TARGET test_types)
    gtest_add_tests(TARGET test_scene)
    gtest_add_tests(TARGET test_joints)
    gtest_add_tests(TARGET test_path_utils)
    gtest_add_tests(TARGET test_se3_low_pass_filter)
    gtest_add_tests(TARGET test_urdf_extended_limits)
endif()
