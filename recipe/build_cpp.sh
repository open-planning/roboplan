#!/usr/bin/env bash
set -euxo pipefail

# Build and install the example models library
cmake -S "${SRC_DIR}/roboplan_example_models" -B build/roboplan_example_models \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$PREFIX \
  -DCMAKE_FIND_FRAMEWORK=LAST \
  -DCMAKE_OSX_SYSROOT=${CONDA_BUILD_SYSROOT:-}

cmake --build build/roboplan_example_models
cmake --install build/roboplan_example_models

# Build and install the roboplan core library
cmake -S "${SRC_DIR}/roboplan" -B build/roboplan \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$PREFIX \
  -DCMAKE_FIND_FRAMEWORK=LAST \
  -DCMAKE_OSX_SYSROOT=${CONDA_BUILD_SYSROOT:-}

cmake --build build/roboplan
cmake --install build/roboplan

# Install the roboplan RRT library
cmake -S "${SRC_DIR}/roboplan_rrt" -B build/roboplan_rrt \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$PREFIX \
  -DCMAKE_FIND_FRAMEWORK=LAST \
  -DCMAKE_OSX_SYSROOT=${CONDA_BUILD_SYSROOT:-}

cmake --build build/roboplan_rrt
cmake --install build/roboplan_rrt

# Install the roboplan simple IK library
cmake -S "${SRC_DIR}/roboplan_simple_ik" -B build/roboplan_simple_ik \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$PREFIX \
  -DCMAKE_FIND_FRAMEWORK=LAST \
  -DCMAKE_OSX_SYSROOT=${CONDA_BUILD_SYSROOT:-}

cmake --build build/roboplan_simple_ik
cmake --install build/roboplan_simple_ik

# Install the roboplan TOPP-RA library
cmake -S "${SRC_DIR}/external/toppra/cpp" -B build/toppra \
  -G Ninja \
  -DBUILD_TESTS=OFF \
  -DPYTHON_BINDINGS=OFF \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$PREFIX \
  -DCMAKE_FIND_FRAMEWORK=LAST \
  -DCMAKE_OSX_SYSROOT=${CONDA_BUILD_SYSROOT:-}

cmake --build build/toppra
cmake --install build/toppra

cmake -S "${SRC_DIR}/roboplan_toppra" -B build/roboplan_toppra \
  -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=$PREFIX \
  -DCMAKE_FIND_FRAMEWORK=LAST \
  -DCMAKE_OSX_SYSROOT=${CONDA_BUILD_SYSROOT:-}

cmake --build build/roboplan_toppra
cmake --install build/roboplan_toppra
