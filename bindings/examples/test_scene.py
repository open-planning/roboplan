from pathlib import Path

import numpy as np

from utils import get_example_resources_directory, get_package_path

import roboplan

if __name__ == "__main__":

    jc = roboplan.JointConfiguration(
        ["joint_1", "joint_2", "joint_3"],
        np.array([0.1, 0.2, 0.3]),
    )
    print(f"Config names: {jc.joint_names}")
    print(f"Config positions: {jc.positions}")
    print("")

    roboplan_examples_dir = Path(get_example_resources_directory())
    urdf_path = roboplan_examples_dir / "ur5_gripper.urdf"
    srdf_path = roboplan_examples_dir / "ur5_gripper.srdf"
    package_paths = [get_package_path()]

    scene = roboplan.Scene("test_scene", urdf_path, srdf_path, package_paths)
    scene.print()
