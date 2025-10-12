from dataclasses import dataclass
from pathlib import Path
from typing import List
from roboplan import get_install_prefix


@dataclass
class RobotModelConfig:
    """
    Configuration for a robot model including file paths and parameters.

    Entries:
      - The URDF path.
      - The SRDF path.
      - The YAML config file path.
      - The default joint group name.
      - The end-effector name.
      - The robot's base link.
      - The starting joint configuration of the robot.
    """

    urdf_path: Path
    srdf_path: Path
    yaml_config_path: Path
    default_joint_group: str
    ee_names: List[str]
    base_link: str
    starting_joint_config: List[float]


# Base directory for all robot models
ROBOPLAN_EXAMPLES_DIR = Path(get_install_prefix()) / "share"
ROBOPLAN_MODELS_DIR = ROBOPLAN_EXAMPLES_DIR / "roboplan_example_models" / "models"

MODELS = {
    "ur5": RobotModelConfig(
        urdf_path=ROBOPLAN_MODELS_DIR / "ur_robot_model" / "ur5_gripper.urdf",
        srdf_path=ROBOPLAN_MODELS_DIR / "ur_robot_model" / "ur5_gripper.srdf",
        yaml_config_path=ROBOPLAN_MODELS_DIR / "ur_robot_model" / "ur5_config.yaml",
        default_joint_group="arm",
        ee_names=["tool0"],
        base_link="base",
        starting_joint_config=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ),
    "franka": RobotModelConfig(
        urdf_path=ROBOPLAN_MODELS_DIR / "franka_robot_model" / "fr3.urdf",
        srdf_path=ROBOPLAN_MODELS_DIR / "franka_robot_model" / "fr3.srdf",
        yaml_config_path=ROBOPLAN_MODELS_DIR / "franka_robot_model" / "fr3_config.yaml",
        default_joint_group="fr3_arm",
        ee_names=["fr3_hand"],
        base_link="fr3_link0",
        starting_joint_config=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
    ),
    "dual": RobotModelConfig(
        urdf_path=ROBOPLAN_MODELS_DIR / "franka_robot_model" / "dual_fr3.urdf",
        srdf_path=ROBOPLAN_MODELS_DIR / "franka_robot_model" / "dual_fr3.srdf",
        yaml_config_path=ROBOPLAN_MODELS_DIR
        / "franka_robot_model"
        / "dual_fr3_config.yaml",
        default_joint_group="dual_fr3_arm",
        ee_names=["left_fr3_hand", "right_fr3_hand"],
        base_link="left_fr3_link0",
        starting_joint_config=[0.0] * 18,
    ),
    "kinova": RobotModelConfig(
        urdf_path=ROBOPLAN_MODELS_DIR / "kinova_robot_model" / "kinova_robotiq.urdf",
        srdf_path=ROBOPLAN_MODELS_DIR / "kinova_robot_model" / "kinova_robotiq.srdf",
        yaml_config_path=ROBOPLAN_MODELS_DIR
        / "kinova_robot_model"
        / "kinova_robotiq_config.yaml",
        default_joint_group="manipulator",
        ee_names=["robotiq_85_base_link"],
        base_link="base_link",
        starting_joint_config=[
            1.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],
    ),
}
