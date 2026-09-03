find_package(GTest REQUIRED)
find_package(roboplan_example_models REQUIRED)

# Each entry is a test executable built from test_<name>.cpp. The ReducedGroupModel test needs
# the internal src/ detail header on its include path (that header is intentionally not
# installed).
set(ROBOPLAN_ALIGATOR_TESTS
    test_reduced_group_model
    test_types
    test_dynamics
    test_trajectory_optimizer
    test_costs
    test_constraints
)

foreach(test_name ${ROBOPLAN_ALIGATOR_TESTS})
  add_executable(${test_name} ${test_name}.cpp)
  set_property(TARGET ${test_name} PROPERTY CXX_STANDARD 20)
  target_include_directories(${test_name} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../src)
  target_link_libraries(
    ${test_name}
    roboplan_aligator
    roboplan::roboplan
    roboplan_example_models::roboplan_example_models
    GTest::gtest_main
  )

  # If ament_gtest is detected, make the tests visible in ROS 2.
  find_package(ament_cmake_gtest QUIET)
  if(ament_cmake_gtest_FOUND)
    ament_add_gtest_test(${test_name})
  else()
    include(GoogleTest)
    gtest_add_tests(TARGET ${test_name})
  endif()
endforeach()

# test_dynamics finite-differences the discretized aligator dynamics directly (through the
# internal problem_builder header), so unlike the other tests it needs aligator's headers and
# symbols explicitly rather than only via the public surface. aligator is linked PUBLIC to
# roboplan_aligator and is available transitively, but we link it here too for clarity (the
# aligator::aligator target is created in the parent CMakeLists before this subdirectory is added).
target_link_libraries(test_dynamics aligator::aligator)

# test_costs builds aligator costs/residuals directly (through the internal cost_factory and
# frame_axis_residual headers), so it likewise needs aligator's headers + symbols.
target_link_libraries(test_costs aligator::aligator)

# test_constraints builds aligator residuals/constraint sets directly (through the internal
# constraint_factory + problem_builder headers), so it likewise needs aligator's headers + symbols.
target_link_libraries(test_constraints aligator::aligator)
