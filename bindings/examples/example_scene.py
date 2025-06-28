from pathlib import Path

import numpy as np

import roboplan
from roboplan import get_package_share_dir


if __name__ == "__main__":

    jc = roboplan.JointConfiguration(
        ["joint_1", "joint_2", "joint_3"],
        np.array([0.1, 0.2, 0.3]),
    )
    print(f"Config names: {jc.joint_names}")
    print(f"Config positions: {jc.positions}")
    print("")

    roboplan_examples_dir = Path(get_package_share_dir())
    urdf_path = roboplan_examples_dir / "ur_robot_model" / "ur5_gripper.urdf"
    srdf_path = roboplan_examples_dir / "ur_robot_model" / "ur5_gripper.srdf"
    package_paths = [roboplan_examples_dir]

    scene = roboplan.Scene("test_scene", urdf_path, srdf_path, package_paths)
    print(scene)
