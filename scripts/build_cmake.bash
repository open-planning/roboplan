#!/bin/bash

# Helper script that builds all the packages using vanilla CMake.

set -e  # Needed to ensure cmake failures propagate up

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
REPO_ROOT_DIR="${SCRIPT_DIR}/.."
pushd ${REPO_ROOT_DIR} || exit

# Helper function to build a CMake project.
build_project() {
  local PROJECT_NAME=$1

  if [ -z "$PROJECT_NAME" ]; then
    echo "Error: No project name provided."
    return 1
  fi

  cmake "${PROJECT_NAME}/CMakeLists.txt" \
    -B"build/${PROJECT_NAME}" \
    -DCMAKE_INSTALL_PREFIX="${PWD}/install/${PROJECT_NAME}"
  cmake --build "build/${PROJECT_NAME}"
  cmake --install "build/${PROJECT_NAME}"
}

# Build all the packages with CMake
# rm -rf build install  # If you want a clean build
mkdir -p build install
export CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:${PWD}/install/

build_project roboplan_example_models
build_project roboplan
build_project roboplan_simple_ik
build_project roboplan_oink
build_project roboplan_toppra
build_project roboplan_rrt
build_project roboplan_examples

echo "
=======================
CMake build complete...
=======================
"

popd > /dev/null || exit
