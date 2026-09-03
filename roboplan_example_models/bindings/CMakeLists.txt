# Python bindings for the roboplan_example_models package.
#
# Added via `add_subdirectory(bindings)` from the parent CMakeLists.txt when
# BUILD_PYTHON_BINDINGS is ON. The roboplan_example_models target is already
# available from the parent configure. This package is standalone (no core dep).
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
  _example_models_ext
  STABLE_ABI
  NB_STATIC
  src/example_models_ext.cpp
  src/example_models.cpp
)
target_include_directories(_example_models_ext PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_definitions(_example_models_ext PRIVATE ROBOPLAN_VERSION="${PROJECT_VERSION}")
set_property(TARGET _example_models_ext PROPERTY CXX_STANDARD 20)
target_link_libraries(_example_models_ext PRIVATE
  roboplan_example_models
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
if(AMENT_BUILD)
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
    "${CMAKE_INSTALL_PREFIX}/${PYTHON_DESTINATION}/roboplan/example_models"
    "${CMAKE_INSTALL_PREFIX}/lib"
  )
  set_target_properties(_example_models_ext PROPERTIES
    INSTALL_RPATH "@loader_path/${_ext_to_lib}"
  )
endif()

install(TARGETS _example_models_ext
  LIBRARY DESTINATION "${PYTHON_DESTINATION}/roboplan/example_models")

# Generate Python type stubs (.pyi) into the source tree so they are versioned
# and shipped by the python install below. Older nanobind versions (from rosdep
# on some distros) do not provide nanobind_add_stub.
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
  nanobind_add_stub(
    _example_models_ext_stub
    MODULE _example_models_ext
    OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/python/roboplan/example_models/_example_models_ext.pyi"
    MARKER_FILE "${CMAKE_CURRENT_SOURCE_DIR}/python/roboplan/example_models/py.typed"
    PYTHON_PATH "$<TARGET_FILE_DIR:_example_models_ext>"
    DEPENDS _example_models_ext
  )

  # Stage this extension under pystage/ (see roboplan/bindings/CMakeLists.txt).
  set_target_properties(_example_models_ext PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/pystage/roboplan/example_models")
  file(COPY "${CMAKE_CURRENT_SOURCE_DIR}/python/roboplan/example_models/__init__.py"
    DESTINATION "${CMAKE_BINARY_DIR}/pystage/roboplan/example_models")
endif()

install(DIRECTORY python/roboplan/ DESTINATION "${PYTHON_DESTINATION}/roboplan")
