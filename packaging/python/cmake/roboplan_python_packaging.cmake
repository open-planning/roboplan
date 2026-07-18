function(roboplan_configure_scikit_build_prefix)
  find_package(Python COMPONENTS Interpreter REQUIRED)
  execute_process(
    COMMAND
      "${Python_EXECUTABLE}"
      -c
      "import pathlib, sysconfig; prefix = pathlib.Path(sysconfig.get_path('purelib')) / 'cmeel.prefix'; print(prefix if prefix.exists() else '')"
    OUTPUT_VARIABLE ROBOPLAN_CMEEL_PREFIX
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  set(ROBOPLAN_CMEEL_PREFIX "${ROBOPLAN_CMEEL_PREFIX}" PARENT_SCOPE)
  if(ROBOPLAN_CMEEL_PREFIX)
    list(PREPEND CMAKE_PREFIX_PATH "${ROBOPLAN_CMEEL_PREFIX}")
    set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH}" PARENT_SCOPE)
  endif()
endfunction()

function(roboplan_register_build_tree_package package_name)
  set(options)
  set(one_value_args)
  set(multi_value_args ALIASES)
  cmake_parse_arguments(ARG "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

  set(_package_dir "${PROJECT_BINARY_DIR}/packaging-python-package-configs/${package_name}")
  file(MAKE_DIRECTORY "${_package_dir}")
  set(_config_file "${_package_dir}/${package_name}Config.cmake")
  file(WRITE "${_config_file}" "# Generated for the packaging/python build tree.\n")

  foreach(alias_pair IN LISTS ARG_ALIASES)
    string(REPLACE "=" ";" alias_parts "${alias_pair}")
    list(GET alias_parts 0 namespaced_target)
    list(GET alias_parts 1 local_target)
    file(APPEND "${_config_file}"
      "if(TARGET ${local_target} AND NOT TARGET ${namespaced_target})\n"
      "  add_library(${namespaced_target} ALIAS ${local_target})\n"
      "endif()\n"
    )
  endforeach()

  set(${package_name}_DIR "${_package_dir}" CACHE PATH "Build-tree ${package_name} package config" FORCE)
endfunction()

function(roboplan_register_build_tree_packages)
  roboplan_register_build_tree_package(roboplan_example_models
    ALIASES roboplan_example_models::roboplan_example_models=roboplan_example_models)
  roboplan_register_build_tree_package(roboplan
    ALIASES roboplan::roboplan=roboplan roboplan::filters=filters)
  roboplan_register_build_tree_package(roboplan_simple_ik
    ALIASES roboplan_simple_ik::roboplan_simple_ik=roboplan_simple_ik)
  roboplan_register_build_tree_package(roboplan_oink
    ALIASES roboplan_oink::roboplan_oink=roboplan_oink)
  roboplan_register_build_tree_package(roboplan_rrt
    ALIASES roboplan_rrt::roboplan_rrt=roboplan_rrt)
  roboplan_register_build_tree_package(roboplan_toppra
    ALIASES roboplan_toppra::roboplan_toppra=roboplan_toppra)
  roboplan_register_build_tree_package(roboplan_cartesian_planning
    ALIASES roboplan_cartesian_planning::roboplan_cartesian_planning=roboplan_cartesian_planning)
endfunction()

function(roboplan_install_matching_libraries pattern)
  foreach(search_root IN ITEMS
      "$ENV{CONDA_PREFIX}/lib"
      "${ROBOPLAN_CMEEL_PREFIX}/lib")
    if(search_root)
      file(GLOB matched_libraries LIST_DIRECTORIES false "${search_root}/${pattern}")
      foreach(matched_library IN LISTS matched_libraries)
        install(FILES "${matched_library}" DESTINATION lib)
        file(REAL_PATH "${matched_library}" matched_real_library)
        install(FILES "${matched_real_library}" DESTINATION lib)
      endforeach()
    endif()
  endforeach()
  foreach(search_prefix IN LISTS CMAKE_PREFIX_PATH)
    if(search_prefix)
      file(GLOB matched_libraries LIST_DIRECTORIES false "${search_prefix}/lib/${pattern}")
      foreach(matched_library IN LISTS matched_libraries)
        install(FILES "${matched_library}" DESTINATION lib)
        file(REAL_PATH "${matched_library}" matched_real_library)
        install(FILES "${matched_real_library}" DESTINATION lib)
      endforeach()
    endif()
  endforeach()
endfunction()

function(roboplan_configure_unified_python_wheel)
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_program(ROBOPLAN_UNIFIED_PATCHELF patchelf REQUIRED)
  elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    find_program(ROBOPLAN_UNIFIED_INSTALL_NAME_TOOL install_name_tool REQUIRED)
  endif()

  if(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(ROBOPLAN_UNIFIED_LIBRARY_INSTALL_RPATH "@loader_path")
    set(ROBOPLAN_UNIFIED_EXTENSION_INSTALL_RPATH "@loader_path/../../lib")
  else()
    set(ROBOPLAN_UNIFIED_LIBRARY_INSTALL_RPATH "$ORIGIN")
    set(ROBOPLAN_UNIFIED_EXTENSION_INSTALL_RPATH "$ORIGIN/../../lib")
  endif()

  foreach(target IN ITEMS
      roboplan_example_models
      roboplan
      filters
      roboplan_simple_ik
      roboplan_oink
      roboplan_rrt
      roboplan_toppra
      roboplan_cartesian_planning)
    if(TARGET ${target})
      set_property(TARGET ${target} PROPERTY INSTALL_RPATH "${ROBOPLAN_UNIFIED_LIBRARY_INSTALL_RPATH}")
    endif()
  endforeach()

  foreach(target IN ITEMS
      _example_models_ext
      _core_ext
      _filters_ext
      _simple_ik_ext
      _optimal_ik_ext
      _rrt_ext
      _toppra_ext
      _cartesian_ext)
    if(TARGET ${target})
      set_property(TARGET ${target} PROPERTY INSTALL_RPATH "${ROBOPLAN_UNIFIED_EXTENSION_INSTALL_RPATH}")
    endif()
  endforeach()

  foreach(library IN ITEMS
      boost_atomic
      boost_filesystem
      boost_serialization
      boost_system
      coal
      console_bridge
      gz-math
      gz-utils
      octomap
      octomath
      pinocchio_collision
      pinocchio_default
      pinocchio_extra
      pinocchio_parsers
      proxsuite
      qdldl
      qhull_r
      sdformat
      tinyxml2
      toppra
      urdfdom_model
      urdfdom_sensor
      urdfdom_world
      yaml-cpp)
    find_library(ROBOPLAN_UNIFIED_${library}_LIBRARY NAMES ${library})
    if(ROBOPLAN_UNIFIED_${library}_LIBRARY)
      install(FILES "${ROBOPLAN_UNIFIED_${library}_LIBRARY}" DESTINATION lib)
      file(REAL_PATH "${ROBOPLAN_UNIFIED_${library}_LIBRARY}" ROBOPLAN_UNIFIED_${library}_REAL_LIBRARY)
      install(FILES "${ROBOPLAN_UNIFIED_${library}_REAL_LIBRARY}" DESTINATION lib)
    endif()
  endforeach()

  foreach(library_pattern IN ITEMS
      "libassimp.so.*"
      "libboost_atomic.so.*"
      "libboost_filesystem.so.*"
      "libboost_serialization.so.*"
      "libboost_system.so.*"
      "libconsole_bridge.so.*"
      "libgz-math.so.*"
      "libgz-utils.so.*"
      "liboctomap.so.*"
      "liboctomath.so.*"
      "libqhull_r.so.*"
      "libsdformat.so.*"
      "libtinyxml2.so.*"
      "liburdfdom_model.so.*"
      "liburdfdom_sensor.so.*"
      "liburdfdom_world.so.*"
      "libyaml-cpp.so.*"
      "libassimp.*.dylib"
      "libboost_atomic.*.dylib"
      "libboost_filesystem.*.dylib"
      "libboost_serialization.*.dylib"
      "libboost_system.*.dylib"
      "libconsole_bridge.*.dylib"
      "libgz-math.*.dylib"
      "libgz-utils.*.dylib"
      "liboctomap.*.dylib"
      "liboctomath.*.dylib"
      "libqhull_r.*.dylib"
      "libsdformat.*.dylib"
      "libtinyxml2.*.dylib"
      "liburdfdom_model.*.dylib"
      "liburdfdom_sensor.*.dylib"
      "liburdfdom_world.*.dylib"
      "libyaml-cpp.*.dylib")
    roboplan_install_matching_libraries("${library_pattern}")
  endforeach()

  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    configure_file(
      "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/repair_unified_rpaths.cmake.in"
      "${PROJECT_BINARY_DIR}/roboplan_repair_unified_rpaths.cmake"
      @ONLY
    )
    install(SCRIPT "${PROJECT_BINARY_DIR}/roboplan_repair_unified_rpaths.cmake")
  elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    configure_file(
      "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/repair_unified_macos_rpaths.cmake.in"
      "${PROJECT_BINARY_DIR}/roboplan_repair_unified_macos_rpaths.cmake"
      @ONLY
    )
    install(SCRIPT "${PROJECT_BINARY_DIR}/roboplan_repair_unified_macos_rpaths.cmake")
  endif()
endfunction()
