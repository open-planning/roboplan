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
