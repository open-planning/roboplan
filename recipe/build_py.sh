#!/usr/bin/env bash
set -euxo pipefail

# Build and install the bindings.
cd "${SRC_DIR}/bindings"
${PYTHON} -m pip install . --no-build-isolation --no-deps -vv
