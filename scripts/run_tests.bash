#!/bin/bash

# Helper script that runs all unit tests and displays results.

EXIT_CODE=0
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# The RoboPlan packages to test. Add new packages here so both the C++ and
# Python test runners below pick them up.
PACKAGES=(
    roboplan
    roboplan_rrt
    roboplan_simple_ik
    roboplan_oink
    roboplan_toppra
    roboplan_cartesian_planning
)

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
    for PACKAGE in "${PACKAGES[@]}";
    do
        [[ -d "${PACKAGE}/test" ]] || continue
        pushd "${PACKAGE}/test" > /dev/null || exit
        ctest -V || EXIT_CODE=$?
        popd > /dev/null || exit
    done
    popd > /dev/null || exit

    echo "
=======================
Running Python tests...
=======================
"
    pushd "${SCRIPT_DIR}/.." > /dev/null || exit
    PYTEST_DIRS=()
    for PACKAGE in "${PACKAGES[@]}";
    do
        [[ -d "${PACKAGE}/bindings/test" ]] && PYTEST_DIRS+=("${PACKAGE}/bindings/test")
    done
    python3 -m pytest "${PYTEST_DIRS[@]}" || EXIT_CODE=$?
    popd > /dev/null || exit
fi

echo "
=======================
Tests completed!
=======================
"
exit ${EXIT_CODE}
