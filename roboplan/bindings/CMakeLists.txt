# Python bindings for the roboplan core package.
#
# This file is added via `add_subdirectory(bindings)` from the parent
# roboplan/CMakeLists.txt only when BUILD_PYTHON_BINDINGS is ON, so the C++
# targets (roboplan, filters) already exist in this configure and do not need
# to be found via find_package.
#
# NOTE: The ament/scikit-build/nanobind detection boilerplate below is
# duplicated per package for now; it should be factored into a shared CMake
# helper installed by this (core) package once the migration is complete.

# Determine if we are compiling with ament or scikit-build.
find_package(ament_cmake QUIET)
if(ament_cmake_FOUND)
  set(AMENT_BUILD TRUE)
  message(STATUS "roboplan bindings: building with ament_cmake")
else()
  set(AMENT_BUILD FALSE)
  message(STATUS "roboplan bindings: building with scikit-build / vanilla CMake")
endif()

# Configure nanobind for either ament or scikit.
if(AMENT_BUILD)
  find_package(Python 3.8 COMPONENTS Interpreter Development REQUIRED)
  find_package(ament_cmake_python REQUIRED)
else()
  find_package(Python
    REQUIRED COMPONENTS Interpreter Development.Module
    OPTIONAL_COMPONENTS Development.SABIModule)
endif()

# Always resolve nanobind from the active Python interpreter so we pick up the
# environment's (pixi/wheel build-env) nanobind rather than a stray system
# install (e.g. /usr/share/nanobind from apt, which lacks bundled headers).
execute_process(
  COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
  OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE nanobind_ROOT)
find_package(nanobind CONFIG REQUIRED)

# NOTE: the shared binding utility header <roboplan_bindings/expected.hpp> lives
# in this package's main include tree (roboplan/include/roboplan_bindings/) and
# is installed unconditionally by the top-level CMakeLists, so dependent packages
# (e.g. roboplan_rrt, roboplan_toppra) can find it via find_package(roboplan)
# even when this package is built without the Python bindings. The core.hpp /
# filters.hpp init-declaration headers under bindings/include are private.

# Compile the core extension module.
nanobind_add_module(
  _core_ext
  STABLE_ABI
  NB_STATIC
  src/core_ext.cpp
  src/core.cpp
)
target_include_directories(_core_ext PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_definitions(_core_ext PRIVATE ROBOPLAN_VERSION="${PROJECT_VERSION}")
set_property(TARGET _core_ext PROPERTY CXX_STANDARD 20)
target_link_libraries(_core_ext PRIVATE
  roboplan
)

# PEP 420 namespace package: no roboplan/__init__.py is shipped. Resolve the
# Python install destination per build context:
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

# Compile the filters extension module (the `filters` C++ target is part of
# this core package, so no cross-package import is needed at runtime).
nanobind_add_module(
  _filters_ext
  STABLE_ABI
  NB_STATIC
  src/filters_ext.cpp
  src/filters.cpp
)
target_include_directories(_filters_ext PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_definitions(_filters_ext PRIVATE ROBOPLAN_VERSION="${PROJECT_VERSION}")
set_property(TARGET _filters_ext PROPERTY CXX_STANDARD 20)
target_link_libraries(_filters_ext PRIVATE
  filters
)

install(TARGETS _core_ext LIBRARY DESTINATION "${PYTHON_DESTINATION}/roboplan/core")
install(TARGETS _filters_ext LIBRARY DESTINATION "${PYTHON_DESTINATION}/roboplan/filters")

# Generate Python type stubs (.pyi) for the compiled extensions. They are
# written into the source tree (python/roboplan/<sub>/) so they are versioned
# in git and shipped by the python install below; regenerate and commit them
# when the C++ API changes (CI checks they are up to date). Older nanobind
# versions (from rosdep on some distros) do not provide nanobind_add_stub.
option(GENERATE_PYTHON_STUBS "Generate .pyi stubs for the Python bindings" ON)
if(GENERATE_PYTHON_STUBS AND NOT COMMAND nanobind_add_stub)
  message(WARNING
    "Skipping Python stub generation: the installed nanobind version does not "
    "provide nanobind_add_stub. Upgrade nanobind to generate stubs.")
endif()
if(GENERATE_PYTHON_STUBS AND COMMAND nanobind_add_stub)
  nanobind_add_stub(
    _core_ext_stub
    MODULE _core_ext
    OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/python/roboplan/core/_core_ext.pyi"
    MARKER_FILE "${CMAKE_CURRENT_SOURCE_DIR}/python/roboplan/core/py.typed"
    PYTHON_PATH "$<TARGET_FILE_DIR:_core_ext>"
    DEPENDS _core_ext
  )
  nanobind_add_stub(
    _filters_ext_stub
    MODULE _filters_ext
    OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/python/roboplan/filters/_filters_ext.pyi"
    MARKER_FILE "${CMAKE_CURRENT_SOURCE_DIR}/python/roboplan/filters/py.typed"
    PYTHON_PATH "$<TARGET_FILE_DIR:_filters_ext>"
    DEPENDS _filters_ext
  )
endif()

# Ships the roboplan/<sub>/ packages and the generated .pyi/py.typed stubs.
install(DIRECTORY python/roboplan/ DESTINATION "${PYTHON_DESTINATION}/roboplan")

# Register the Python binding tests (colcon picks these up via ament). They run
# against the installed package, so the roboplan namespace is importable.
if(AMENT_BUILD AND BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  set(BINDINGS_TESTS_PY
    test_import
    test_scene
    test_filters
  )
  foreach(test_name ${BINDINGS_TESTS_PY})
    ament_add_pytest_test(${test_name} test/${test_name}.py TIMEOUT 300)
  endforeach()
endif()
