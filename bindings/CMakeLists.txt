cmake_minimum_required(VERSION 3.22)

project(roboplan_bindings LANGUAGES CXX)

# Determine if we are compiling with ament or scikit
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  set(AMENT_BUILD TRUE)
  message(STATUS "Building with ament_cmake")
else()
  set(AMENT_BUILD FALSE)
  message(STATUS "Building with scikit-build")
endif()

# If neither build tool is found issue a warning
if (NOT AMENT_BUILD AND NOT SKBUILD)
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

# Configure nanobind for either ament or scikit
if(AMENT_BUILD)
  find_package(Python 3.8 COMPONENTS Interpreter Development REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  include(FetchContent)
  FetchContent_Declare(
    nanobind
    GIT_REPOSITORY https://github.com/wjakob/nanobind.git
    GIT_TAG v2.9.2
  )
  FetchContent_MakeAvailable(nanobind)
else()
  find_package(Python
    REQUIRED COMPONENTS Interpreter Development.Module
    OPTIONAL_COMPONENTS Development.SABIModule)
  find_package(nanobind CONFIG REQUIRED)
endif()

find_package(roboplan REQUIRED)
find_package(roboplan_example_models REQUIRED)
find_package(roboplan_rrt REQUIRED)
find_package(roboplan_simple_ik REQUIRED)
find_package(roboplan_toppra REQUIRED)

# Roboplan binding sources
set(ROBOPLAN_EXT_SOURCES
  src/roboplan_ext.cpp
  src/modules/core.cpp
  src/modules/example_models.cpp
  src/modules/rrt.cpp
  src/modules/simple_ik.cpp
  src/modules/toppra.cpp
)

# Compile the extension module
if(AMENT_BUILD)
  nanobind_add_module(
    roboplan_ext_target
    NB_STATIC
    ${ROBOPLAN_EXT_SOURCES}
  )
  set_target_properties(roboplan_ext_target PROPERTIES
    OUTPUT_NAME roboplan_ext
  )
  set(EXT_TARGET roboplan_ext_target)
else()
  nanobind_add_module(
    roboplan_ext
    STABLE_ABI
    NB_STATIC
    ${ROBOPLAN_EXT_SOURCES}
  )
  set(EXT_TARGET roboplan_ext)
endif()

# Configure the extension module
target_include_directories(${EXT_TARGET} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
set_property(TARGET ${EXT_TARGET} PROPERTY CXX_STANDARD 20)
target_link_libraries(${EXT_TARGET} PRIVATE
  roboplan::expected
  roboplan::roboplan
  roboplan_example_models::roboplan_example_models
  roboplan_rrt::roboplan_rrt
  roboplan_simple_ik::roboplan_simple_ik
  roboplan_toppra::roboplan_toppra
)

if(AMENT_BUILD)
  ament_get_python_install_dir(PYTHON_INSTALL_DIR)
  install(TARGETS ${EXT_TARGET}
    LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/roboplan"
  )
  install(DIRECTORY src/roboplan/
    DESTINATION "${PYTHON_INSTALL_DIR}/roboplan"
    FILES_MATCHING PATTERN "*.py"
  )
  ament_package()
else()
  install(TARGETS ${EXT_TARGET} LIBRARY DESTINATION roboplan)
endif()
