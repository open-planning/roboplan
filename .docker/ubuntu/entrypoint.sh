#!/bin/bash

# Set the library paths
pushd "/roboplan_ws/install" > /dev/null || exit
for PACKAGE in */lib;
do
    export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:/roboplan_ws/install/${PACKAGE}"
done
popd > /dev/null || exit
echo "Set LD_LIBRARY_PATH to installed packages."

# Execute the command passed into this entrypoint
exec "$@"
