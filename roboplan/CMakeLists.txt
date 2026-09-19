cmake_minimum_required(VERSION 3.22)
project(roboplan VERSION 0.7.0)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  ament_package()
else()
  # Give non-ament builds an install rule to run. Under ament this must be
  # skipped: its symlink-install wrapper forwards the empty CODE string
  # unquoted, which makes CMake fail.
  install(CODE "")
endif()
