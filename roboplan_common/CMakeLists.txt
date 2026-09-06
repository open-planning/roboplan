cmake_minimum_required(VERSION 3.22)
project(roboplan_common VERSION 0.6.1)

find_package(ament_cmake QUIET)
if(COMMAND roboplan_get_python_install_dir)
  roboplan_get_python_install_dir(PYTHON_DESTINATION)
elseif(ament_cmake_FOUND)
  find_package(ament_cmake_python REQUIRED)
  ament_get_python_install_dir(PYTHON_DESTINATION)
  if(PYTHON_DESTINATION MATCHES "dist-packages")
    find_package(Python 3.8 COMPONENTS Interpreter REQUIRED)
    set(PYTHON_DESTINATION
        "lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages")
  endif()
elseif(SKBUILD)
  # scikit-build collects files installed at the wheel root.
  set(PYTHON_DESTINATION ".")
else()
  find_package(Python REQUIRED COMPONENTS Interpreter)
  file(TO_CMAKE_PATH "${Python_SITEARCH}" PYTHON_DESTINATION)
endif()

# Make the helper available to nanobind stub generation in sibling packages.
file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/python/roboplan_common"
  DESTINATION "${CMAKE_BINARY_DIR}/pystage")
install(DIRECTORY python/roboplan_common DESTINATION "${PYTHON_DESTINATION}")

if(ament_cmake_FOUND)
  ament_package()
endif()
