# Python bindings for the roboplan_rrt package.
#
# Added via `add_subdirectory(bindings)` from the parent CMakeLists.txt when
# BUILD_PYTHON_BINDINGS is ON. The roboplan_rrt target and the roboplan::roboplan
# import target (which carries the installed <roboplan_bindings/...> headers) are
# already available from the parent configure.
#
# NOTE: the ament/scikit/nanobind detection below mirrors the core package and
# should be replaced by a shared CMake helper once the migration is complete.

find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  set(AMENT_BUILD TRUE)
  find_package(Python 3.8 COMPONENTS Interpreter Development REQUIRED)
  find_package(ament_cmake_python REQUIRED)
else()
  set(AMENT_BUILD FALSE)
  find_package(Python
    REQUIRED COMPONENTS Interpreter Development.Module
    OPTIONAL_COMPONENTS Development.SABIModule)
endif()

execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT)
find_package(nanobind CONFIG REQUIRED)

nanobind_add_module(
  _rrt_ext
  STABLE_ABI
  NB_STATIC
  src/rrt_ext.cpp
  src/rrt.cpp
)
target_include_directories(_rrt_ext PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_definitions(_rrt_ext PRIVATE ROBOPLAN_VERSION="${PROJECT_VERSION}")
set_property(TARGET _rrt_ext PROPERTY CXX_STANDARD 20)
target_link_libraries(_rrt_ext PRIVATE
  roboplan_rrt
  roboplan::roboplan
)

# Resolve the Python install destination per build context:
#   - colcon (ament): the site-packages dir (relative to the package's install
#     prefix) that colcon's PYTHONPATH hook adds. We set this directly rather
#     than via ament_get_python_install_dir(), which returns
#     lib/python3/dist-packages on Debian-based ROS -- a path the hook omits.
#   - scikit-build wheel: the wheel root (".").
#   - plain CMake into a prefix (pixi/conda, the Docker images): the
#     interpreter's site-packages (absolute), so the package is importable.
if(AMENT_BUILD)
  set(PYTHON_DESTINATION
      "lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages")
elseif(SKBUILD)
  set(PYTHON_DESTINATION ".")
else()
  set(PYTHON_DESTINATION "${Python_SITEARCH}")
endif()

install(TARGETS _rrt_ext LIBRARY DESTINATION "${PYTHON_DESTINATION}/roboplan/rrt")

# Generate Python type stubs (.pyi) into the source tree so they are versioned
# and shipped by the python install below. Stub generation imports the module,
# which imports roboplan.core, so roboplan-core must be importable at build time
# (it is, via site-packages in the wheel/colcon build). Older nanobind versions
# (from rosdep on some distros) do not provide nanobind_add_stub.
option(GENERATE_PYTHON_STUBS "Generate .pyi stubs for the Python bindings" ON)
if(GENERATE_PYTHON_STUBS AND NOT COMMAND nanobind_add_stub)
  message(WARNING
    "Skipping Python stub generation: the installed nanobind version does not "
    "provide nanobind_add_stub. Upgrade nanobind to generate stubs.")
endif()
if(GENERATE_PYTHON_STUBS AND COMMAND nanobind_add_stub)
  nanobind_add_stub(
    _rrt_ext_stub
    MODULE _rrt_ext
    OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/python/roboplan/rrt/_rrt_ext.pyi"
    MARKER_FILE "${CMAKE_CURRENT_SOURCE_DIR}/python/roboplan/rrt/py.typed"
    PYTHON_PATH "$<TARGET_FILE_DIR:_rrt_ext>"
    DEPENDS _rrt_ext
  )
endif()

install(DIRECTORY python/roboplan/ DESTINATION "${PYTHON_DESTINATION}/roboplan")

# Register the Python binding tests (colcon picks these up via ament). They run
# against the installed package, so the roboplan namespace is importable.
if(AMENT_BUILD AND BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_rrt test/test_rrt.py TIMEOUT 300)
endif()
