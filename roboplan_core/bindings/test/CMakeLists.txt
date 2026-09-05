# Register the Python binding tests (colcon picks these up via ament). They run
# against the installed package, so the roboplan namespace is importable.
find_package(ament_cmake_pytest REQUIRED)
set(BINDINGS_TESTS_PY
  test_import
  test_scene
  test_filters
)
foreach(test_name ${BINDINGS_TESTS_PY})
  ament_add_pytest_test(${test_name} ${test_name}.py TIMEOUT 300)
endforeach()
