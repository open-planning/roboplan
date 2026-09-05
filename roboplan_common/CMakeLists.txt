cmake_minimum_required(VERSION 3.22)
project(roboplan_common VERSION 0.6.1)

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  set(AMENT_BUILD TRUE)
  find_package(ament_cmake_python REQUIRED)
  find_package(Python 3.8 COMPONENTS Interpreter Development REQUIRED)
else()
  set(AMENT_BUILD FALSE)
endif()

include("${CMAKE_CURRENT_SOURCE_DIR}/cmake/RoboplanPythonInstall.cmake")
roboplan_get_python_install_dir(PYTHON_DESTINATION)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/roboplan_commonConfigVersion.cmake"
  VERSION "${PROJECT_VERSION}"
  COMPATIBILITY SameMajorVersion
)
configure_package_config_file(
  cmake/roboplan_commonConfig.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/roboplan_commonConfig.cmake"
  INSTALL_DESTINATION lib/cmake/roboplan_common
)

# Make the helper available to nanobind stub generation in sibling packages.
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/python/roboplan_common"
  DESTINATION "${CMAKE_BINARY_DIR}/pystage")
install(DIRECTORY python/roboplan_common DESTINATION "${PYTHON_DESTINATION}")
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/roboplan_commonConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/roboplan_commonConfigVersion.cmake"
  cmake/RoboplanPythonInstall.cmake
  DESTINATION lib/cmake/roboplan_common
)

if(ament_cmake_FOUND)
  ament_package()
endif()
