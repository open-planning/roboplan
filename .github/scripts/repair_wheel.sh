#!/bin/bash
# Repair a wheel with auditwheel, excluding the pinocchio ecosystem shared
# libraries. These are provided at runtime by the "pin" PyPI package and must
# not be bundled into the wheel.
#
# Usage: repair_wheel.sh <wheel> <dest_dir>
set -euxo pipefail

WHEEL="$1"
DEST_DIR="$2"
CONDA_ENV="/opt/cibw_env"

# Build --exclude flags for every pinocchio-ecosystem .so in the conda env.
# Everything else (yaml-cpp, osqp, toppra, roboplan sub-packages, etc.) is
# bundled normally by auditwheel.
EXCLUDES=$(find "$CONDA_ENV/lib" -maxdepth 1 \
  \( \
    -name "libpinocchio*.so*"    \
    -o -name "libcoal*.so*"      \
    -o -name "libhpp-fcl*.so*"   \
    -o -name "liburdfdom*.so*"   \
    -o -name "libconsole_bridge*.so*" \
    -o -name "libassimp*.so*"    \
    -o -name "libboost*.so*"     \
    -o -name "libcasadi*.so*"    \
    -o -name "liboctomap*.so*"   \
    -o -name "liboctomath*.so*"  \
  \) \
  -printf "--exclude %f\n" 2>/dev/null \
  | sort -u | tr '\n' ' ')

echo "Pinocchio ecosystem excluded from bundling: $EXCLUDES"
# shellcheck disable=SC2086
auditwheel repair -w "$DEST_DIR" $EXCLUDES "$WHEEL"
