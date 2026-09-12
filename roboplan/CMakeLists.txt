cmake_minimum_required(VERSION 3.22)
project(roboplan VERSION 0.6.1)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  ament_package()
endif()

install(CODE "")
