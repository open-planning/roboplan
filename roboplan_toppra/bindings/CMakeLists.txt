# Python bindings for the roboplan_toppra package.
#
# Added via `add_subdirectory(bindings)` from the parent CMakeLists.txt when
# BUILD_PYTHON_BINDINGS is ON. The roboplan_toppra target and the
# roboplan::roboplan import target (which carries the installed
# <roboplan_bindings/...> headers) are already available from the parent
# configure.
#
# NOTE: the ament/scikit/nanobind detection below is intentionally duplicated
# per package for clarity rather than factored into a shared helper.

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
  _toppra_ext
  STABLE_ABI
  NB_STATIC
  src/toppra_ext.cpp
  src/toppra.cpp
)
target_include_directories(_toppra_ext PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_definitions(_toppra_ext PRIVATE ROBOPLAN_VERSION="${PROJECT_VERSION}")
set_property(TARGET _toppra_ext PROPERTY CXX_STANDARD 20)
target_link_libraries(_toppra_ext PRIVATE
  roboplan_toppra
  roboplan::roboplan
)

# Resolve the Python install destination per build context:
#   - colcon (ament): the dir (relative to the package's install prefix) that
#     colcon's PYTHONPATH hook adds. Use ament_get_python_install_dir() so the
#     layout matches on Windows (Lib/site-packages), falling back to
#     site-packages on Debian-based ROS, where ament returns
#     lib/python3/dist-packages -- a path the hook omits.
#   - scikit-build wheel: the wheel root (".").
#   - plain CMake into a prefix (pixi/conda, the Docker images): the
#     interpreter's site-packages (absolute), so the package is importable.
if(NOT COMMAND roboplan_get_python_install_dir AND DEFINED ENV{CMEEL_BUILD})
  find_package(roboplan_common CONFIG REQUIRED)
endif()

if(COMMAND roboplan_get_python_install_dir)
  roboplan_get_python_install_dir(PYTHON_DESTINATION)
elseif(DEFINED ROBOPLAN_PYTHON_INSTALL_DIR)
  set(PYTHON_DESTINATION "${ROBOPLAN_PYTHON_INSTALL_DIR}")
elseif(AMENT_BUILD)
  ament_get_python_install_dir(_ament_python_dir)
  if(_ament_python_dir MATCHES "dist-packages")
    set(PYTHON_DESTINATION
        "lib/python${Python_VERSION_MAJOR}.${Python_VERSION_MINOR}/site-packages")
  else()
    set(PYTHON_DESTINATION "${_ament_python_dir}")
  endif()
elseif(SKBUILD)
  set(PYTHON_DESTINATION ".")
else()
  # TO_CMAKE_PATH converts Windows backslashes, which would otherwise become
  # invalid escape sequences in the generated cmake_install.cmake.
  file(TO_CMAKE_PATH "${Python_SITEARCH}" PYTHON_DESTINATION)
endif()

# Ensure the extension can find workspace dylibs via @rpath on macOS, as it seems that
# SIP will strip DYLD_LIBRARY_PATH from subprocesses, which will break stub generation.
if(APPLE)
  file(RELATIVE_PATH _ext_to_lib
    "${CMAKE_INSTALL_PREFIX}/${PYTHON_DESTINATION}/roboplan/toppra"
    "${CMAKE_INSTALL_PREFIX}/lib"
  )
  set_target_properties(_toppra_ext PROPERTIES
    INSTALL_RPATH "@loader_path/${_ext_to_lib}"
  )
endif()

install(TARGETS _toppra_ext
  LIBRARY DESTINATION "${PYTHON_DESTINATION}/roboplan/toppra")

# Generate Python type stubs (.pyi) into the source tree so they are versioned
# and shipped by the python install below. Stub generation imports the module,
# which imports roboplan.core, so roboplan-core must be importable at build time
# (it is, via site-packages in the wheel/colcon build). Older nanobind versions
# (from rosdep on some distros) do not provide nanobind_add_stub.
# Default OFF on Windows: stub generation imports the extension from the build
# tree, where the DLLs it depends on are not yet on the loader search path.
# The committed stubs are generated on Unix.
if(WIN32)
  set(generate_stubs_default OFF)
else()
  set(generate_stubs_default ON)
endif()
option(GENERATE_PYTHON_STUBS "Generate .pyi stubs for the Python bindings" ${generate_stubs_default})
if(GENERATE_PYTHON_STUBS AND NOT COMMAND nanobind_add_stub)
  message(WARNING
    "Skipping Python stub generation: the installed nanobind version does not "
    "provide nanobind_add_stub. Upgrade nanobind to generate stubs.")
endif()
if(GENERATE_PYTHON_STUBS AND COMMAND nanobind_add_stub)
  # roboplan.toppra imports roboplan.core; use the staged copy if a sibling
  # build already produced one.
  set(_extra_stub_python_path)
  set(_extra_stub_depends)
  if(TARGET _core_ext)
    list(APPEND _extra_stub_python_path "${CMAKE_BINARY_DIR}/pystage")
    list(APPEND _extra_stub_depends _core_ext)
  endif()
  nanobind_add_stub(
    _toppra_ext_stub
    MODULE _toppra_ext
    OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/python/roboplan/toppra/_toppra_ext.pyi"
    MARKER_FILE "${CMAKE_CURRENT_SOURCE_DIR}/python/roboplan/toppra/py.typed"
    PYTHON_PATH "$<TARGET_FILE_DIR:_toppra_ext>" ${_extra_stub_python_path}
    DEPENDS _toppra_ext ${_extra_stub_depends}
  )

  # Stage this extension under pystage/, for any sibling package that might need it.
  set_target_properties(_toppra_ext PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/pystage/roboplan/toppra")
  file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/python/roboplan/toppra/__init__.py"
    DESTINATION "${CMAKE_BINARY_DIR}/pystage/roboplan/toppra")
endif()

install(DIRECTORY python/roboplan/ DESTINATION "${PYTHON_DESTINATION}/roboplan")
