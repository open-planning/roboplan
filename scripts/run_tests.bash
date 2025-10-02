#!/bin/bash

# Helper script that runs all unit tests and displays results.

EXIT_CODE=0
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

if command -v ros2 >/dev/null 2>&1;
then
    echo "
=======================
Running ROS C++ tests...
=======================
"
    if [[ -z "${COLCON_PREFIX_PATH}" ]]
    then
        echo "Did not find COLCON_PREFIX_PATH. Make sure your workspace is sourced."
        return 1
    fi

    pushd "${COLCON_PREFIX_PATH}/../" > /dev/null || exit
    colcon test \
        --event-handlers console_cohesion+ \
        --return-code-on-test-failure || EXIT_CODE=$?
    echo ""
    colcon test-result --verbose
    popd > /dev/null || exit
else
    echo "
=======================
Running C++ tests...
=======================
"
    pushd "${SCRIPT_DIR}/../build" > /dev/null || exit
    for PACKAGE in */test;
    do
        pushd ${PACKAGE} > /dev/null || exit
        ctest -V || EXIT_CODE=$?
        popd > /dev/null || exit
    done
    popd > /dev/null || exit
fi


echo "
=======================
Running Python tests...
=======================
"
pushd "${SCRIPT_DIR}/../bindings" >> /dev/null || exit
python3 -m pytest test/ || EXIT_CODE=$?
popd > /dev/null || exit

echo "
=======================
Tests completed!
=======================
"
exit ${EXIT_CODE}
