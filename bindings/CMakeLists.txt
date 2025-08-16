cmake_minimum_required(VERSION 3.22)

project(roboplan_bindings LANGUAGES CXX)

if (NOT SKBUILD)
  message(WARNING "\
  This CMake file is meant to be executed using 'scikit-build'. Running
  it directly will almost certainly not produce the desired result. If
  you are a user trying to install this package, please use the command
  below, which will install all necessary build dependencies, compile
  the package in an isolated environment, and then install it.
  =====================================================================
   $ pip install .
  =====================================================================
  If you are a software developer, and this is your own package, then
  it is usually much more efficient to install the build dependencies
  in your environment once and use the following command that avoids
  a costly creation of a new virtual environment at every compilation:
  =====================================================================
   $ pip install nanobind scikit-build-core[pyproject]
   $ pip install --no-build-isolation -ve .
  =====================================================================
  You may optionally add -Ceditable.rebuild=true to auto-rebuild when
  the package is imported. Otherwise, you need to re-run the above
  after editing C++ files.")
endif()

find_package(Python
  REQUIRED COMPONENTS Interpreter Development.Module
  OPTIONAL_COMPONENTS Development.SABIModule)

find_package(nanobind CONFIG REQUIRED)
find_package(roboplan REQUIRED)
find_package(roboplan_example_models REQUIRED)
find_package(roboplan_rrt REQUIRED)
find_package(roboplan_simple_ik REQUIRED)
find_package(roboplan_toppra REQUIRED)

# Compile the extension module
nanobind_add_module(
  roboplan_ext
  STABLE_ABI
  NB_STATIC
  src/roboplan_ext.cpp
  src/modules/core.cpp
  src/modules/example_models.cpp
  src/modules/rrt.cpp
  src/modules/simple_ik.cpp
  src/modules/toppra.cpp
)
target_include_directories(roboplan_ext PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
set_property(TARGET roboplan_ext PROPERTY CXX_STANDARD 20)
target_link_libraries(roboplan_ext PRIVATE
  roboplan::expected
  roboplan::roboplan
  roboplan_example_models::roboplan_example_models
  roboplan_rrt::roboplan_rrt
  roboplan_simple_ik::roboplan_simple_ik
  roboplan_toppra::roboplan_toppra
)

# Install directive for scikit-build-core
install(TARGETS roboplan_ext LIBRARY DESTINATION roboplan)
