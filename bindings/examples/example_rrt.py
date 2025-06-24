from pathlib import Path

import numpy as np

from roboplan import (
    get_package_share_dir,
    JointConfiguration,
    Scene,
    RRTOptions,
    RRT,
)


if __name__ == "__main__":

    roboplan_examples_dir = Path(get_package_share_dir())
    urdf_path = roboplan_examples_dir / "ur_robot_model" / "ur5_gripper.urdf"
    srdf_path = roboplan_examples_dir / "ur_robot_model" / "ur5_gripper.srdf"
    package_paths = [roboplan_examples_dir]

    scene = Scene("test_scene", urdf_path, srdf_path, package_paths)

    options = RRTOptions()
    options.max_connection_distance = 1.0
    options.collision_check_step_size = 0.05
    rrt = RRT(scene, options)

    q_start = JointConfiguration()
    while True:
        q_start.positions = scene.randomPositions()
        if not scene.hasCollisions(q_start.positions):
            break

    q_goal = JointConfiguration()
    while True:
        q_goal.positions = scene.randomPositions()
        if not scene.hasCollisions(q_goal.positions):
            break

    result = rrt.plan(q_start, q_goal)
