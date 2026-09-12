#!/bin/bash

# Builds all packages via vanilla CMake, using the superbuild in
# ../superbuild and its CMakePresets.json "default" preset.

set -e  # Needed to ensure cmake failures propagate up

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
REPO_ROOT_DIR="${SCRIPT_DIR}/.."
pushd "${REPO_ROOT_DIR}" || exit

INSTALL_PREFIX="${PWD}/install"

cmake -S superbuild --preset default -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}"
cmake --build build --parallel "$(nproc)"
cmake --install build

echo "
=======================
CMake build complete...
=======================
"

popd > /dev/null || exit
