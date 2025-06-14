#!/bin/bash

# Runs all unit tests

EXIT_CODE=0
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

if [[ -z "${COLCON_PREFIX_PATH}" ]]
then
    echo "Did not find COLCON_PREFIX_PATH. Make sure your workspace is sourced."
    return 1
fi

echo "
=======================
Running C++ tests...
=======================
"
pushd "${COLCON_PREFIX_PATH}/../"
colcon test \
    --event-handlers console_cohesion+ \
    --return-code-on-test-failure || EXIT_CODE=$?
echo ""
colcon test-result --verbose
popd || exit

echo "
=======================
Running Python tests...
=======================
"
pushd "${SCRIPT_DIR}/../bindings"
python3 -m pytest . || EXIT_CODE=$?
popd || exit

echo "
=======================
Tests completed!
=======================
"
exit ${EXIT_CODE}
