from pathlib import Path

import numpy as np

from utils import get_example_resources_directory, get_package_path

from roboplan import (
    Scene,
    JointConfiguration,
    CartesianConfiguration,
    SimpleIkOptions,
    SimpleIk,
)

if __name__ == "__main__":

    roboplan_examples_dir = Path(get_example_resources_directory())
    urdf_path = roboplan_examples_dir / "ur5_gripper.urdf"
    srdf_path = roboplan_examples_dir / "ur5_gripper.srdf"
    package_paths = [get_package_path()]

    scene = Scene("test_scene", urdf_path, srdf_path, package_paths)

    options = SimpleIkOptions()
    options.step_size = 0.25
    ik_solver = SimpleIk(scene, options)

    start = JointConfiguration()
    start.positions = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    solution = JointConfiguration()

    goal = CartesianConfiguration()
    goal.base_frame = "base"
    goal.tip_frame = "tool0"
    goal.tform = np.array(
        [
            [-0.934556, -0.0993347, -0.341668, 0.781603],
            [-0.342415, -0.00996671, 0.939496, 0.268262],
            [-0.0967298, 0.995004, -0.0246992, 0.0321088],
            [0.0, 0.0, 0.0, 1.0],
        ]
    )  # TODO: get from FK

    result = ik_solver.solveIk(goal, start, solution)

    if not result:
        print("Failed to solve IK")
    else:
        print("Solved IK with solution:")
        print(solution.positions)
