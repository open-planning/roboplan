from pathlib import Path
from roboplan import get_package_share_dir


ROBOPLAN_EXAMPLES_DIR = Path(get_package_share_dir())
MODELS = {
    "ur5": [
        ROBOPLAN_EXAMPLES_DIR / "ur_robot_model" / "ur5_gripper.urdf",
        ROBOPLAN_EXAMPLES_DIR / "ur_robot_model" / "ur5_gripper.srdf",
        "tool0",
        "base",
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ],
    "franka": [
        ROBOPLAN_EXAMPLES_DIR / "franka_robot_model" / "fr3.urdf",
        ROBOPLAN_EXAMPLES_DIR / "franka_robot_model" / "fr3.srdf",
        "fr3_hand",
        "fr3_link0",
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ],
}
