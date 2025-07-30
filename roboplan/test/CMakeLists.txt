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

add_executable(test_path_utils test_path_utils.cpp)
set_property(TARGET test_path_utils PROPERTY CXX_STANDARD 20)
target_link_libraries(
  test_path_utils
  roboplan
  GTest::gtest_main
  roboplan_example_models::roboplan_example_models
)

# TODO: Define tests as a variable

# If ament_gtest is detected, make the tests visible in ROS 2.
find_package(ament_cmake_gtest QUIET)
if ( ament_cmake_gtest_FOUND )
    # See https://github.com/ament/ament_cmake/issues/588
    find_package(ament_cmake_test REQUIRED)

    ament_add_gtest_test(test_types)
    ament_add_gtest_test(test_scene)
    ament_add_gtest_test(test_path_utils)
else()
    include(GoogleTest)
    gtest_discover_tests(test_types test_scene test_path_utils)
endif()
