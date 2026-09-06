# Shared superbuild helpers for configuring all RoboPlan packages in one
# add_subdirectory() tree.
#
# Each package resolves siblings via find_package(<name> REQUIRED), expecting
# an installed package, so it stays buildable standalone. To make that work
# in an uninstalled build tree, we pre-register a near-empty fake
# "<name>Config.cmake" for each package and point <name>_DIR at it.
# find_package(CONFIG) checks <name>_DIR before CMAKE_PREFIX_PATH, so this
# makes find_package(<name> REQUIRED) succeed without a real install.
#
# No ALIAS creation needed here: each package's own CMakeLists.txt already
# aliases its target next to add_library(), so it works the same way via
# add_subdirectory() or find_package(). (CMake >= 3.24's
# FetchContent_Declare(... OVERRIDE_FIND_PACKAGE) would only replace the
# <name>_DIR half, not aliasing, and isn't an option anyway: ROS Humble
# ships CMake 3.22.)

function(roboplan_register_build_tree_package package_name)
  set(_package_dir "${PROJECT_BINARY_DIR}/superbuild-package-configs/${package_name}")
  file(MAKE_DIRECTORY "${_package_dir}")
  file(WRITE "${_package_dir}/${package_name}Config.cmake"
    "# Generated for a RoboPlan CMake superbuild. ${package_name} already\n"
    "# aliases its own target, so this file just needs to exist for\n"
    "# find_package() to succeed without a real install.\n"
  )

  set(${package_name}_DIR "${_package_dir}" CACHE PATH "Build-tree ${package_name} package config" FORCE)
endfunction()

function(roboplan_register_build_tree_packages)
  foreach(package_name IN ITEMS
      roboplan_example_models
      roboplan
      roboplan_simple_ik
      roboplan_oink
      roboplan_rrt
      roboplan_toppra
      roboplan_cartesian_planning)
    roboplan_register_build_tree_package(${package_name})
  endforeach()
endfunction()
