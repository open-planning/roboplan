#!/bin/bash

# Helper script that builds all the packages using vanilla CMake.

set -e  # Needed to ensure cmake failures propagate up

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
REPO_ROOT_DIR="${SCRIPT_DIR}/.."
pushd ${REPO_ROOT_DIR} || exit

# TODO: First install Pinocchio via https://stack-of-tasks.github.io/pinocchio/download.html
# This whole block should probably just be in a non-ROS Dockerfile.
export PATH=/opt/openrobots/bin:$PATH
export PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH
export CMAKE_PREFIX_PATH=/opt/openrobots:$CMAKE_PREFIX_PATH

# Build all the packages with CMake
# rm -rf build install  # If you want a clean build
mkdir -p build install
export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:${PWD}/install/

# TODO: Make into reusable function?
cmake roboplan_example_models/CMakeLists.txt \
    -Bbuild/roboplan_example_models \
    -DCMAKE_INSTALL_PREFIX=${PWD}/install/roboplan_example_models
cmake --build build/roboplan_example_models
cmake --install build/roboplan_example_models

cmake roboplan/CMakeLists.txt \
  -Bbuild/roboplan \
  -DCMAKE_INSTALL_PREFIX=${PWD}/install/roboplan
cmake --build build/roboplan
cmake --install build/roboplan

cmake roboplan_simple_ik/CMakeLists.txt \
  -Bbuild/roboplan_simple_ik \
  -DCMAKE_INSTALL_PREFIX=${PWD}/install/roboplan_simple_ik
cmake --build build/roboplan_simple_ik
cmake --install build/roboplan_simple_ik

cmake roboplan_rrt/CMakeLists.txt \
  -Bbuild/roboplan_rrt \
  -DCMAKE_INSTALL_PREFIX=${PWD}/install/roboplan_rrt
cmake --build build/roboplan_rrt
cmake --install build/roboplan_rrt

cmake roboplan_oink/CMakeLists.txt \
  -Bbuild/roboplan_oink \
  -DCMAKE_INSTALL_PREFIX=${PWD}/install/roboplan_oink
cmake --build build/roboplan_oink
cmake --install build/roboplan_oink

cmake external/toppra/cpp/CMakeLists.txt \
  -Bbuild/toppra \
  -DCMAKE_INSTALL_PREFIX=${PWD}/install/toppra \
  -DBUILD_TESTS=OFF \
  -DPYTHON_BINDINGS=OFF
cmake --build build/toppra
cmake --install build/toppra

cmake roboplan_toppra/CMakeLists.txt \
  -Bbuild/roboplan_toppra \
  -DCMAKE_INSTALL_PREFIX=${PWD}/install/roboplan_toppra
cmake --build build/roboplan_toppra
cmake --install build/roboplan_toppra

cmake roboplan_examples/CMakeLists.txt \
  -Bbuild/roboplan_examples \
  -DCMAKE_INSTALL_PREFIX=${PWD}/install/roboplan_examples
cmake --build build/roboplan_examples
cmake --install build/roboplan_examples

echo "
=======================
CMake build complete...
=======================
"

popd > /dev/null || exit
