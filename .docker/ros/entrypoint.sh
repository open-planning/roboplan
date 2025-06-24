#!/bin/bash

# Source ROS and the RoboPlan workspace
echo -e "Sourced ROS ${ROS_DISTRO}"
source /opt/ros/${ROS_DISTRO}/setup.bash

if [ -f /workspace/roboplan_ws/install/setup.bash ]
then
  echo "Sourced RoboPlan workspace"
  source /workspace/roboplan_ws/install/setup.bash
fi

# Execute the command passed into this entrypoint
exec "$@"
