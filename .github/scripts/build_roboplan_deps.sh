#!/bin/bash
# Runs as CIBW_BEFORE_ALL inside the manylinux container.
# Installs C++ dependencies via micromamba and builds all roboplan sub-packages
# so that the bindings wheel can find them via CMAKE_PREFIX_PATH.
set -euxo pipefail

PROJECT_DIR="${1:-/project}"
CONDA_ENV="/opt/cibw_env"
INSTALL_PREFIX="/opt/roboplan_deps"

# ---------------------------------------------------------------------------
# 1. Install micromamba (static binary, works in any Linux container)
# ---------------------------------------------------------------------------
curl -Ls https://micro.mamba.pm/api/micromamba/linux-64/latest \
  | tar -xvj -C /usr/local bin/micromamba

# ---------------------------------------------------------------------------
# 2. Create conda environment with all C++ build + runtime dependencies.
#    - osqp-eigen / libtoppra are NOT listed here; roboplan_oink and
#      roboplan_toppra fall back to FetchContent when they are absent, so
#      cmake will download and build them automatically.
# ---------------------------------------------------------------------------
/usr/local/bin/micromamba create \
  --prefix "$CONDA_ENV" \
  --channel conda-forge \
  --yes \
  "cmake>=3.22" ninja \
  "gcc>=11,<12" "gxx>=11,<12" \
  "eigen>=3.4" \
  "pinocchio>=3.9.0,<4" \
  "yaml-cpp>=0.8"

CMAKE="$CONDA_ENV/bin/cmake"
GCC="$CONDA_ENV/bin/gcc"
GXX="$CONDA_ENV/bin/g++"

# ---------------------------------------------------------------------------
# 3. Build and install every roboplan sub-package.
#    Order mirrors pixi.toml build_all task.
# ---------------------------------------------------------------------------
build_and_install() {
  local NAME="$1"
  "$CMAKE" \
    -GNinja \
    -S "$PROJECT_DIR/$NAME" \
    -B "/tmp/build/$NAME" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTING=OFF \
    "-DCMAKE_INSTALL_PREFIX=$INSTALL_PREFIX" \
    "-DCMAKE_PREFIX_PATH=$CONDA_ENV;$INSTALL_PREFIX" \
    "-DCMAKE_C_COMPILER=$GCC" \
    "-DCMAKE_CXX_COMPILER=$GXX" \
    "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
  "$CMAKE" --build "/tmp/build/$NAME" --parallel "$(nproc)"
  "$CMAKE" --install "/tmp/build/$NAME"
}

build_and_install roboplan_example_models
build_and_install roboplan
build_and_install roboplan_simple_ik
build_and_install roboplan_oink
build_and_install roboplan_toppra
build_and_install roboplan_rrt
