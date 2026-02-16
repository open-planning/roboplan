cmake_minimum_required(VERSION 3.22)

project(roboplan_bindings LANGUAGES CXX VERSION 0.1.0)

add_compile_definitions(ROBOPLAN_VERSION="${PROJECT_VERSION}")

# Compilation options
option(GENERATE_PYTHON_STUBS "Generate the Python stubs associated to the Python library" ON)

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
  # Detect the installed nanobind package and import it into CMake
  execute_process(
    COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT)
  find_package(nanobind CONFIG REQUIRED)
else()
  find_package(Python
    REQUIRED COMPONENTS Interpreter Development.Module
    OPTIONAL_COMPONENTS Development.SABIModule)
  find_package(nanobind CONFIG REQUIRED)
endif()

find_package(roboplan REQUIRED)
find_package(roboplan_example_models REQUIRED)
find_package(roboplan_oink REQUIRED)
find_package(roboplan_rrt REQUIRED)
find_package(roboplan_simple_ik REQUIRED)
find_package(roboplan_toppra REQUIRED)

# Interface library for the utility functions
add_library(roboplan_bindings_utils INTERFACE)
target_include_directories(roboplan_bindings_utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(DIRECTORY include/utils
  DESTINATION include
  FILES_MATCHING PATTERN "*.hpp"
)
install(TARGETS roboplan_bindings_utils
  EXPORT roboplan_bindings_utilsTargets
)
if(AMENT_BUILD)
  ament_export_targets(roboplan_bindings_utilsTargets HAS_LIBRARY_TARGET)
  ament_export_dependencies()
else()
  include(CMakePackageConfigHelpers)

  install(EXPORT roboplan_bindings_utilsTargets
    FILE roboplan_bindings_utilsTargets.cmake
    NAMESPACE roboplan_bindings::
    DESTINATION lib/cmake/roboplan_bindings_utils
  )
  configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/roboplan_bindings_utilsConfig.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/roboplan_bindings_utilsConfig.cmake"
    INSTALL_DESTINATION lib/cmake/roboplan_bindings_utils
  )
  install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/roboplan_bindings_utilsConfig.cmake"
    DESTINATION lib/cmake/roboplan_bindings_utils
  )
endif()

# Roboplan binding sources
set(ROBOPLAN_EXT_SOURCES
  src/roboplan_ext.cpp
  src/modules/core.cpp
  src/modules/example_models.cpp
  src/modules/optimal_ik.cpp
  src/modules/rrt.cpp
  src/modules/simple_ik.cpp
  src/modules/toppra.cpp
)

# Compile the extension module, we use STABLE_ABI even for ament, though that
# may be unnecessary.
nanobind_add_module(
  roboplan_ext
  STABLE_ABI
  NB_STATIC
  ${ROBOPLAN_EXT_SOURCES}
)

# Configure the extension module
target_include_directories(roboplan_ext PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
set_property(TARGET roboplan_ext PROPERTY CXX_STANDARD 20)

# Ensures that the library is always called tl_expected::tl_expected regardless
# of whether it was built from source in roboplan or comes from ROS binaries.
find_package(tl_expected QUIET)
if(NOT TARGET tl_expected::tl_expected AND TARGET roboplan::tl_expected)
    add_library(tl_expected::tl_expected INTERFACE IMPORTED)
    set_target_properties(tl_expected::tl_expected PROPERTIES
        INTERFACE_LINK_LIBRARIES roboplan::tl_expected
    )
endif()

target_link_libraries(roboplan_ext PRIVATE
  roboplan::roboplan
  roboplan_example_models::roboplan_example_models
  roboplan_oink::roboplan_oink
  roboplan_rrt::roboplan_rrt
  roboplan_simple_ik::roboplan_simple_ik
  roboplan_toppra::roboplan_toppra
  tl_expected::tl_expected
)

if(AMENT_BUILD)
  ament_get_python_install_dir(PYTHON_INSTALL_DIR)
  install(TARGETS roboplan_ext
    LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/roboplan"
  )
  ament_python_install_package(roboplan
    PACKAGE_DIR src/roboplan
  )

  if(BUILD_TESTING)
    find_package(ament_cmake_pytest REQUIRED)
    set(BINDINGS_TESTS_PY
      test_import
      test_rrt
      test_scene
    )
    foreach(test_name ${BINDINGS_TESTS_PY})
      ament_add_pytest_test(${test_name} test/${test_name}.py
        APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
        TIMEOUT 300
      )
    endforeach()
  endif()

  ament_package()
else()
  install(TARGETS roboplan_ext LIBRARY DESTINATION roboplan)
endif()

# Generate Python stub files from the bindings.
if(GENERATE_PYTHON_STUBS)
  nanobind_add_stub(
    roboplan_ext_stub
    MODULE roboplan_ext
    RECURSIVE
    OUTPUT roboplan_ext.pyi
    MARKER_FILE py.typed
    PYTHON_PATH $<TARGET_FILE_DIR:roboplan_ext>
    DEPENDS roboplan_ext
  )

  if(PYTHON_INSTALL_DIR)
      set(PYTHON_STUB_INSTALL_DIR "${PYTHON_INSTALL_DIR}/roboplan")
  else()
      set(PYTHON_STUB_INSTALL_DIR "roboplan")
  endif()
  install(
    DIRECTORY "${CMAKE_BINARY_DIR}/roboplan_ext/"
    DESTINATION "${PYTHON_STUB_INSTALL_DIR}"
    # Copy the stub files to the source folder for docs and versioning.
    # For now, this step has to be run manually when submitting changes.
    # It might be good to make a CI action that generates this and finds mismatches,
    # prompting the user to regenerate it on their own.
    DESTINATION "${CMAKE_SOURCE_DIR}/src/roboplan"
  )
endif()
