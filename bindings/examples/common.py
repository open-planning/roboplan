from pathlib import Path
from roboplan import get_package_share_dir


ROBOPLAN_EXAMPLES_DIR = Path(get_package_share_dir())

# Dictionary of supported example models and their relevant parameters.
# Entries are a list of:
#     - The URDF path.
#     - The SRDF path.
#     - The YAML config file path.
#     - The end-effector name.
#     - The robot's base link.
#     - The starting joint configuration of the robot.
MODELS = {
    "ur5": [
        ROBOPLAN_EXAMPLES_DIR / "ur_robot_model" / "ur5_gripper.urdf",
        ROBOPLAN_EXAMPLES_DIR / "ur_robot_model" / "ur5_gripper.srdf",
        ROBOPLAN_EXAMPLES_DIR / "ur_robot_model" / "ur5_config.yaml",
        "tool0",
        "base",
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ],
    "franka": [
        ROBOPLAN_EXAMPLES_DIR / "franka_robot_model" / "fr3.urdf",
        ROBOPLAN_EXAMPLES_DIR / "franka_robot_model" / "fr3.srdf",
        ROBOPLAN_EXAMPLES_DIR / "franka_robot_model" / "fr3_config.yaml",
        "fr3_hand",
        "fr3_link0",
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ],
}
