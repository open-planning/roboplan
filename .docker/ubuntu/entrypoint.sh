#!/bin/bash

# Set the library paths for installed packages
source /roboplan_env.sh
echo "Set CMAKE_PREFIX_PATH and LD_LIBRARY_PATH for roboplan."

# Execute the command passed into this entrypoint
exec "$@"
