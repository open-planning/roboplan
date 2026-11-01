# Register the Python binding tests (colcon picks these up via ament). They run
# against the installed package, so the roboplan namespace is importable.
find_package(ament_cmake_pytest REQUIRED)
ament_add_pytest_test(test_ompl test_ompl.py TIMEOUT 300)
