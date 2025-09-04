from dataclasses import dataclass
from pathlib import Path
from typing import List
from roboplan import get_package_share_dir


@dataclass
class RobotModelConfig:
    """
    Configuration for a robot model including file paths and parameters.

    Entries:
      - The URDF path.
      - The SRDF path.
      - The YAML config file path.
      - The end-effector name.
      - The robot's base link.
      - The starting joint configuration of the robot.
    """

    urdf_path: Path
    srdf_path: Path
    yaml_config_path: Path
    ee_names: List[str]
    base_link: str
    starting_joint_config: List[float]


# Base directory for all robot models
ROBOPLAN_EXAMPLES_DIR = Path(get_package_share_dir())

MODELS = {
    "ur5": RobotModelConfig(
        urdf_path=ROBOPLAN_EXAMPLES_DIR / "ur_robot_model" / "ur5_gripper.urdf",
        srdf_path=ROBOPLAN_EXAMPLES_DIR / "ur_robot_model" / "ur5_gripper.srdf",
        yaml_config_path=ROBOPLAN_EXAMPLES_DIR / "ur_robot_model" / "ur5_config.yaml",
        ee_names=["tool0"],
        base_link="base",
        starting_joint_config=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ),
    "franka": RobotModelConfig(
        urdf_path=ROBOPLAN_EXAMPLES_DIR / "franka_robot_model" / "fr3.urdf",
        srdf_path=ROBOPLAN_EXAMPLES_DIR / "franka_robot_model" / "fr3.srdf",
        yaml_config_path=ROBOPLAN_EXAMPLES_DIR
        / "franka_robot_model"
        / "fr3_config.yaml",
        ee_names=["fr3_hand"],
        base_link="fr3_link0",
        starting_joint_config=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ),
    "dual": RobotModelConfig(
        urdf_path=ROBOPLAN_EXAMPLES_DIR / "franka_robot_model" / "dual_fr3.urdf",
        srdf_path=ROBOPLAN_EXAMPLES_DIR / "franka_robot_model" / "dual_fr3.srdf",
        yaml_config_path=ROBOPLAN_EXAMPLES_DIR
        / "franka_robot_model"
        / "dual_fr3_config.yaml",
        ee_names=["left_fr3_hand", "right_fr3_hand"],
        base_link="left_fr3_link0",
        starting_joint_config=[0.0] * 18,
    ),
}
