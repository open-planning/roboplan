from pathlib import Path

import numpy as np

import roboplan

if __name__ == "__main__":

    jc = roboplan.JointConfiguration(
        ["joint_1", "joint_2", "joint_3"],
        np.array([0.1, 0.2, 0.3]),
    )
    print(f"Config names: {jc.joint_names}")
    print(f"Config positions: {jc.positions}")
    print("")

    urdf_path = Path(
        "/home/sebastian/workspace/roboplan_ws/src/roboplan/roboplan_examples/ur_robot_model/ur5_gripper.urdf"
    )

    srdf_path = Path(
        "/home/sebastian/workspace/roboplan_ws/src/roboplan/roboplan_examples/ur_robot_model/ur5_gripper.srdf"
    )

    package_paths = [Path("/home/sebastian/workspace/roboplan_ws/src/roboplan/")]

    scene = roboplan.Scene("test_scene", urdf_path, srdf_path, package_paths)
    scene.print()
