cmake_minimum_required(VERSION 3.22)

set(ROBOPLAN_PACKAGING_VERSION "0.0.0")
if(DEFINED SKBUILD_PROJECT_VERSION)
  set(ROBOPLAN_PACKAGING_VERSION "${SKBUILD_PROJECT_VERSION}")
endif()
project(roboplan VERSION "${ROBOPLAN_PACKAGING_VERSION}")

get_filename_component(ROBOPLAN_REPOSITORY_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." ABSOLUTE)

include(cmake/roboplan_python_packaging.cmake)

if(SKBUILD)
  roboplan_configure_scikit_build_prefix()
endif()

roboplan_register_build_tree_packages()

add_subdirectory("${ROBOPLAN_REPOSITORY_ROOT}/roboplan_example_models" "roboplan_example_models")
add_subdirectory("${ROBOPLAN_REPOSITORY_ROOT}/roboplan" "roboplan")
add_subdirectory("${ROBOPLAN_REPOSITORY_ROOT}/roboplan_simple_ik" "roboplan_simple_ik")
add_subdirectory("${ROBOPLAN_REPOSITORY_ROOT}/roboplan_oink" "roboplan_oink")
add_subdirectory("${ROBOPLAN_REPOSITORY_ROOT}/roboplan_rrt" "roboplan_rrt")
add_subdirectory("${ROBOPLAN_REPOSITORY_ROOT}/roboplan_toppra" "roboplan_toppra")
add_subdirectory("${ROBOPLAN_REPOSITORY_ROOT}/roboplan_aligator" "roboplan_aligator")
add_subdirectory("${ROBOPLAN_REPOSITORY_ROOT}/roboplan_cartesian_planning" "roboplan_cartesian_planning")

if(SKBUILD)
  roboplan_configure_unified_python_wheel()
endif()
