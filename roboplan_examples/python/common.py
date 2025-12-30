from dataclasses import dataclass
from pathlib import Path
from typing import List

import hppfcl
import numpy as np
from numpy.typing import NDArray
import pinocchio as pin

from roboplan.core import Box, Scene, Sphere
from roboplan.example_models import get_package_models_dir


@dataclass
class ObstacleConfig:
    """Configuration for obstacles in an example."""

    name: str  # Name of the obstacle.
    geom: hppfcl.ShapeBase  # The obstacle geometry.
    parent_frame: str  # The name of the parent frame.
    tform: NDArray  # The transform from parent frame to the geometry.
    color: NDArray  # The geometry color.
    disabled_collisions: list[str] | None = (
        None  # Optional list of disabled collision bodies.
    )

    def addToScene(self, scene: Scene) -> None:
        """Helper function to add the obstacle to the scene."""
        if isinstance(self.geom, hppfcl.Box):
            x, y, z = self.geom.halfSide * 2.0
            scene.addBoxGeometry(
                self.name,
                self.parent_frame,
                Box(x, y, z),
                self.tform,
                self.color,
            )
        elif isinstance(self.geom, hppfcl.Sphere):
            scene.addSphereGeometry(
                self.name,
                self.parent_frame,
                Sphere(self.geom.radius),
                self.tform,
                self.color,
            )
        else:
            raise TypeError(f"Unsupported geometry type: {type(self.geom)}")

        if self.disabled_collisions is not None:
            for body in self.disabled_collisions:
                scene.setCollisions(self.name, body, False)

    def addToPinocchioModels(
        self,
        model: pin.Model,
        collision_model: pin.GeometryModel,
        visual_model: pin.GeometryModel,
    ) -> None:
        """Helper function to add the obstacle to Pinocchio geometry models."""
        geom_obj = pin.GeometryObject(
            self.name,
            model.getFrameId(self.parent_frame),
            pin.SE3(self.tform),
            self.geom,
        )
        geom_obj.meshColor = self.color
        collision_model.addGeometryObject(geom_obj)
        visual_model.addGeometryObject(geom_obj)


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
    obstacles: List[ObstacleConfig]


# Base directory for all robot models
ROBOPLAN_MODELS_DIR = get_package_models_dir()

MODELS = {
    "ur5": RobotModelConfig(
        urdf_path=ROBOPLAN_MODELS_DIR / "ur_robot_model" / "ur5_gripper.urdf",
        srdf_path=ROBOPLAN_MODELS_DIR / "ur_robot_model" / "ur5_gripper.srdf",
        yaml_config_path=ROBOPLAN_MODELS_DIR / "ur_robot_model" / "ur5_config.yaml",
        default_joint_group="arm",
        ee_names=["tool0"],
        base_link="base",
        starting_joint_config=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        obstacles=[
            ObstacleConfig(
                name="test_box",
                geom=hppfcl.Box(0.5, 0.5, 0.5),
                parent_frame="universe",
                tform=pin.SE3(np.eye(3), np.array([0.0, 0.0, 1.2])).homogeneous,
                color=np.array([0.0, 0.0, 1.0, 0.5]),
            ),
            ObstacleConfig(
                name="test_sphere",
                geom=hppfcl.Sphere(0.3),
                parent_frame="universe",
                tform=pin.SE3(np.eye(3), np.array([0.75, 0.0, 0.25])).homogeneous,
                color=np.array([1.0, 0.0, 0.0, 0.5]),
                disabled_collisions=["test_box"],
            ),
            ObstacleConfig(
                name="ground_plane",
                geom=hppfcl.Box(1.5, 1.5, 0.2),
                parent_frame="universe",
                tform=pin.SE3(np.eye(3), np.array([0.0, 0.0, -0.1])).homogeneous,
                color=np.array([0.5, 0.5, 0.5, 0.5]),
                disabled_collisions=["base_link", "test_box", "test_sphere"],
            ),
        ],
    ),
    "franka": RobotModelConfig(
        urdf_path=ROBOPLAN_MODELS_DIR / "franka_robot_model" / "fr3.urdf",
        srdf_path=ROBOPLAN_MODELS_DIR / "franka_robot_model" / "fr3.srdf",
        yaml_config_path=ROBOPLAN_MODELS_DIR / "franka_robot_model" / "fr3_config.yaml",
        default_joint_group="fr3_arm",
        ee_names=["fr3_hand"],
        base_link="fr3_link0",
        starting_joint_config=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        obstacles=[
            ObstacleConfig(
                name="test_box",
                geom=hppfcl.Box(1.0, 1.0, 0.5),
                parent_frame="universe",
                tform=pin.SE3(np.eye(3), np.array([0.0, 0.0, 1.2])).homogeneous,
                color=np.array([0.0, 0.0, 1.0, 0.5]),
            ),
            ObstacleConfig(
                name="test_sphere",
                geom=hppfcl.Sphere(0.3),
                parent_frame="universe",
                tform=pin.SE3(np.eye(3), np.array([0.75, 0.0, 0.25])).homogeneous,
                color=np.array([1.0, 0.0, 0.0, 0.5]),
                disabled_collisions=["test_box"],
            ),
            ObstacleConfig(
                name="ground_plane",
                geom=hppfcl.Box(1.5, 1.5, 0.2),
                parent_frame="universe",
                tform=pin.SE3(np.eye(3), np.array([0.0, 0.0, -0.1])).homogeneous,
                color=np.array([0.5, 0.5, 0.5, 0.5]),
                disabled_collisions=["fr3_link0", "test_box", "test_sphere"],
            ),
        ],
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
        obstacles=[
            ObstacleConfig(
                name="test_box",
                geom=hppfcl.Box(1.0, 1.0, 0.5),
                parent_frame="universe",
                tform=pin.SE3(np.eye(3), np.array([0.0, 0.0, 1.3])).homogeneous,
                color=np.array([0.0, 0.0, 1.0, 0.5]),
            ),
            ObstacleConfig(
                name="ground_plane",
                geom=hppfcl.Box(2.0, 2.0, 0.2),
                parent_frame="universe",
                tform=pin.SE3(np.eye(3), np.array([0.0, 0.0, -0.1])).homogeneous,
                color=np.array([0.5, 0.5, 0.5, 0.5]),
                disabled_collisions=["left_fr3_link0", "right_fr3_link0", "test_box"],
            ),
        ],
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
        obstacles=[
            ObstacleConfig(
                name="test_box",
                geom=hppfcl.Box(1.0, 1.0, 0.5),
                parent_frame="universe",
                tform=pin.SE3(np.eye(3), np.array([0.0, 0.0, 1.3])).homogeneous,
                color=np.array([0.0, 0.0, 1.0, 0.5]),
            ),
            ObstacleConfig(
                name="test_sphere",
                geom=hppfcl.Sphere(0.3),
                parent_frame="universe",
                tform=pin.SE3(np.eye(3), np.array([0.75, 0.0, 0.25])).homogeneous,
                color=np.array([1.0, 0.0, 0.0, 0.5]),
                disabled_collisions=["test_box"],
            ),
            ObstacleConfig(
                name="ground_plane",
                geom=hppfcl.Box(1.5, 1.5, 0.2),
                parent_frame="universe",
                tform=pin.SE3(np.eye(3), np.array([0.0, 0.0, -0.1])).homogeneous,
                color=np.array([0.5, 0.5, 0.5, 0.5]),
                disabled_collisions=["base_link", "test_box", "test_sphere"],
            ),
        ],
    ),
    "so101": RobotModelConfig(
        urdf_path=ROBOPLAN_MODELS_DIR / "so101_robot_model" / "so101.urdf",
        srdf_path=ROBOPLAN_MODELS_DIR / "so101_robot_model" / "so101.srdf",
        yaml_config_path=ROBOPLAN_MODELS_DIR
        / "so101_robot_model"
        / "so101_config.yaml",
        default_joint_group="arm",
        ee_names=["gripper_link"],
        base_link="base_link",
        starting_joint_config=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        obstacles=[
            ObstacleConfig(
                name="test_box",
                geom=hppfcl.Box(0.25, 0.25, 0.25),
                parent_frame="universe",
                tform=pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.5])).homogeneous,
                color=np.array([0.0, 0.0, 1.0, 0.5]),
            ),
            ObstacleConfig(
                name="ground_plane",
                geom=hppfcl.Box(0.5, 0.5, 0.1),
                parent_frame="universe",
                tform=pin.SE3(np.eye(3), np.array([0.0, 0.0, -0.05])).homogeneous,
                color=np.array([0.5, 0.5, 0.5, 0.5]),
                disabled_collisions=["base_link", "test_box"],
            ),
        ],
    ),
}
