cmake_minimum_required(VERSION 3.5)
project(roboplan_example_resources VERSION 0.0.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_library(roboplan_example_resources SHARED
  src/example_resources.cpp
)
target_include_directories(roboplan_example_resources PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(roboplan_example_resources dl)

install(
  TARGETS
  roboplan_example_resources
  EXPORT roboplan_example_resourcesTargets
  INCLUDES DESTINATION include
)

install(DIRECTORY include/ DESTINATION include)

install(EXPORT roboplan_example_resourcesTargets
    FILE roboplan_example_resourcesTargets.cmake
    DESTINATION lib/cmake/roboplan_example_resources
)

file(WRITE
  ${CMAKE_CURRENT_BINARY_DIR}/roboplan_example_resourcesConfig.cmake
  "include(\"\${CMAKE_CURRENT_LIST_DIR}/roboplan_example_resourcesTargets.cmake\")
")

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/roboplan_example_resourcesConfig.cmake
  DESTINATION lib/cmake/roboplan_example_resources
)

install(
  DIRECTORY
  ur_robot_model
  DESTINATION share/roboplan_example_resources
)

# If ament_cmake is detected, export the package to be visible in ROS 2.
find_package(ament_cmake QUIET)
if ( ament_cmake_FOUND )
    message(STATUS "ament_cmake found. Exporting for ROS 2.")
    ament_package()
endif()
