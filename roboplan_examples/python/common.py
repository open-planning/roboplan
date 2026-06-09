from dataclasses import dataclass
from pathlib import Path

try:
    import coal
except ModuleNotFoundError:
    import hppfcl as coal

import numpy as np
from numpy.typing import NDArray
import pinocchio as pin

from roboplan.core import Box, Cylinder, Mesh, Scene, Sphere, OcTree
from roboplan.example_models import get_package_models_dir


@dataclass
class ObstacleConfig:
    """Configuration for obstacles in an example."""

    name: str
    """Name of the obstacle."""

    geom: coal.ShapeBase | Path
    """The obstacle geometry, or a path to a mesh."""

    parent_frame: str
    """The name of the parent frame."""

    tform: NDArray
    """The transform from parent frame to the geometry."""

    color: NDArray
    """The geometry color, in RGBA format."""

    disabled_collisions: list[str] | None = None
    """Optional list of disabled collision bodies."""

    def addToScene(self, scene: Scene) -> None:
        """Helper function to add the obstacle to the scene."""
        if isinstance(self.geom, Path):  # Special case for meshes
            scene.addMeshGeometry(
                self.name,
                self.parent_frame,
                Mesh(self.geom),
                self.tform,
                self.color,
            )
        elif isinstance(self.geom, coal.Box):
            x, y, z = self.geom.halfSide * 2.0
            scene.addBoxGeometry(
                self.name,
                self.parent_frame,
                Box(x, y, z),
                self.tform,
                self.color,
            )
        elif isinstance(self.geom, coal.Sphere):
            scene.addSphereGeometry(
                self.name,
                self.parent_frame,
                Sphere(self.geom.radius),
                self.tform,
                self.color,
            )
        elif isinstance(self.geom, coal.Cylinder):
            scene.addCylinderGeometry(
                self.name,
                self.parent_frame,
                Cylinder(self.geom.radius, self.geom.halfLength * 2.0),
                self.tform,
                self.color,
            )
        elif isinstance(self.geom, coal.OcTree):
            boxes = self.geom.toBoxes()
            resolution = self.geom.getResolution()
            scene.addOcTreeGeometry(
                self.name,
                self.parent_frame,
                OcTree(boxes, resolution),
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
        geom_obj = self.createGeometryObject(model)
        collision_model.addGeometryObject(geom_obj)
        visual_model.addGeometryObject(geom_obj)

    def createGeometryObject(self, model: pin.Model):
        if isinstance(self.geom, Path):  # Special case for meshes
            loader = coal.MeshLoader()
            geom = loader.load(str(self.geom))
        else:
            geom = self.geom

        geom_obj = pin.GeometryObject(
            self.name,
            model.getFrameId(self.parent_frame),
            pin.SE3(self.tform),
            geom,
        )
        geom_obj.meshColor = self.color
        if isinstance(self.geom, Path):
            geom_obj.meshPath = str(self.geom)
        return geom_obj


@dataclass
class RobotModelConfig:
    """Configuration for a robot model including file paths and parameters."""

    urdf_path: Path
    """The path to the URDF file."""

    srdf_path: Path
    """The path to the SRDF file."""

    yaml_config_path: Path
    """The path to the YAML config file."""

    default_joint_group: str
    """The default joint group name."""

    ee_names: list[str]
    """The names of the end effector frames."""

    base_link: str
    """The robot's base link."""

    starting_joint_config: list[float]
    """The starting joint configuration of the robot."""

    obstacles: list[ObstacleConfig]
    """Configurations for the obstacles in the example scene."""


# Base directory for all robot models
ROBOPLAN_MODELS_DIR = get_package_models_dir()


def get_model_data():
    """Returns example model data."""
    return {
        "ur5": RobotModelConfig(
            urdf_path=ROBOPLAN_MODELS_DIR / "ur_robot_model" / "ur5_gripper.urdf",
            srdf_path=ROBOPLAN_MODELS_DIR / "ur_robot_model" / "ur5_gripper.srdf",
            yaml_config_path=ROBOPLAN_MODELS_DIR / "ur_robot_model" / "ur5_config.yaml",
            default_joint_group="arm",
            ee_names=["tool0"],
            base_link="base",
            starting_joint_config=[
                0.0,
                -np.pi / 2,
                np.pi / 2,
                -np.pi / 2,
                -np.pi / 2,
                0.0,
            ],
            obstacles=[
                ObstacleConfig(
                    name="test_box",
                    geom=coal.Box(0.5, 0.5, 0.5),
                    parent_frame="universe",
                    tform=pin.SE3(np.eye(3), np.array([0.0, 0.0, 1.2])).homogeneous,
                    color=np.array([0.0, 0.0, 1.0, 0.5]),
                ),
                ObstacleConfig(
                    name="test_sphere",
                    geom=coal.Sphere(0.3),
                    parent_frame="universe",
                    tform=pin.SE3(np.eye(3), np.array([0.75, 0.0, 0.25])).homogeneous,
                    color=np.array([1.0, 0.0, 0.0, 0.5]),
                    disabled_collisions=["test_box"],
                ),
                ObstacleConfig(
                    name="ground_plane",
                    geom=coal.Box(1.5, 1.5, 0.2),
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
            yaml_config_path=ROBOPLAN_MODELS_DIR
            / "franka_robot_model"
            / "fr3_config.yaml",
            default_joint_group="fr3_arm",
            ee_names=["fr3_hand"],
            base_link="fr3_link0",
            starting_joint_config=[
                # nq=8 with Pinocchio mimic joints (fr3_finger_joint2 mimics joint1 removed).
                0.0,
                -np.pi / 4,
                0.0,
                -3 * np.pi / 4,
                0.0,
                np.pi / 2,
                np.pi / 4,
                0.04,
            ],
            obstacles=[
                ObstacleConfig(
                    name="test_box",
                    geom=coal.Box(1.0, 1.0, 0.5),
                    parent_frame="universe",
                    tform=pin.SE3(np.eye(3), np.array([0.0, 0.0, 1.2])).homogeneous,
                    color=np.array([0.0, 0.0, 1.0, 0.5]),
                ),
                ObstacleConfig(
                    name="test_sphere",
                    geom=coal.Sphere(0.3),
                    parent_frame="universe",
                    tform=pin.SE3(np.eye(3), np.array([0.75, 0.0, 0.25])).homogeneous,
                    color=np.array([1.0, 0.0, 0.0, 0.5]),
                    disabled_collisions=["test_box"],
                ),
                ObstacleConfig(
                    name="ground_plane",
                    geom=coal.Box(1.5, 1.5, 0.2),
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
            starting_joint_config=[
                # nq=16 with Pinocchio mimic joints (fr3_finger_joint2 mimics joint1 removed x2).
                # Left arm
                0.0,
                -np.pi / 4,
                0.0,
                -3 * np.pi / 4,
                0.0,
                np.pi / 2,
                np.pi / 4,
                0.04,
                # Right arm
                0.0,
                -np.pi / 4,
                0.0,
                -3 * np.pi / 4,
                0.0,
                np.pi / 2,
                np.pi / 4,
                0.04,
            ],
            obstacles=[
                ObstacleConfig(
                    name="test_box",
                    geom=coal.Box(1.0, 1.0, 0.5),
                    parent_frame="universe",
                    tform=pin.SE3(np.eye(3), np.array([0.0, 0.0, 1.3])).homogeneous,
                    color=np.array([0.0, 0.0, 1.0, 0.5]),
                ),
                ObstacleConfig(
                    name="ground_plane",
                    geom=coal.Box(2.0, 2.0, 0.2),
                    parent_frame="universe",
                    tform=pin.SE3(np.eye(3), np.array([0.0, 0.0, -0.1])).homogeneous,
                    color=np.array([0.5, 0.5, 0.5, 0.5]),
                    disabled_collisions=[
                        "left_fr3_link0",
                        "right_fr3_link0",
                        "test_box",
                    ],
                ),
            ],
        ),
        "kinova": RobotModelConfig(
            urdf_path=ROBOPLAN_MODELS_DIR
            / "kinova_robot_model"
            / "kinova_robotiq.urdf",
            srdf_path=ROBOPLAN_MODELS_DIR
            / "kinova_robot_model"
            / "kinova_robotiq.srdf",
            yaml_config_path=ROBOPLAN_MODELS_DIR
            / "kinova_robot_model"
            / "kinova_robotiq_config.yaml",
            default_joint_group="manipulator",
            ee_names=["robotiq_85_base_link"],
            base_link="base_link",
            starting_joint_config=[
                # nq=12 with Pinocchio mimic joints (Robotiq finger mimics collapsed).
                # Continuous joints use [cos(theta), sin(theta)]
                1.0,
                0.0,  # joint_1
                0.26179938779914941,  # joint_2
                -1.0,
                0.0,  # joint_3 (pi)
                -2.2689280275926285,  # joint_4
                1.0,
                0.0,  # joint_5
                0.96,  # joint_6
                0.0,
                1.0,  # joint_7 (pi/2)
                0.0,  # robotiq_85_left_knuckle_joint
            ],
            obstacles=[
                ObstacleConfig(
                    name="test_box",
                    geom=coal.Box(1.0, 1.0, 0.5),
                    parent_frame="universe",
                    tform=pin.SE3(np.eye(3), np.array([0.0, 0.0, 1.3])).homogeneous,
                    color=np.array([0.0, 0.0, 1.0, 0.5]),
                ),
                ObstacleConfig(
                    name="test_sphere",
                    geom=coal.Sphere(0.3),
                    parent_frame="universe",
                    tform=pin.SE3(np.eye(3), np.array([0.75, 0.0, 0.25])).homogeneous,
                    color=np.array([1.0, 0.0, 0.0, 0.5]),
                    disabled_collisions=["test_box"],
                ),
                ObstacleConfig(
                    name="ground_plane",
                    geom=coal.Box(1.5, 1.5, 0.2),
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
            starting_joint_config=[0.0, -np.pi / 4, 0.0, -np.pi / 2, 0.0, np.pi / 4],
            obstacles=[
                ObstacleConfig(
                    name="test_box",
                    geom=coal.Box(0.25, 0.25, 0.25),
                    parent_frame="universe",
                    tform=pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.5])).homogeneous,
                    color=np.array([0.0, 0.0, 1.0, 0.5]),
                ),
                ObstacleConfig(
                    name="ground_plane",
                    geom=coal.Box(0.5, 0.5, 0.1),
                    parent_frame="universe",
                    tform=pin.SE3(np.eye(3), np.array([0.0, 0.0, -0.05])).homogeneous,
                    color=np.array([0.5, 0.5, 0.5, 0.5]),
                    disabled_collisions=["base_link", "test_box"],
                ),
            ],
        ),
        "stretch": RobotModelConfig(
            urdf_path=ROBOPLAN_MODELS_DIR
            / "stretch4_robot_model"
            / "stretch4_sg4.urdf",
            srdf_path=ROBOPLAN_MODELS_DIR
            / "stretch4_robot_model"
            / "stretch4_sg4.srdf",
            yaml_config_path=ROBOPLAN_MODELS_DIR
            / "stretch4_robot_model"
            / "stretch4_sg4_config.yaml",
            default_joint_group="stretch4_arm",
            ee_names=["grasp_center_link"],
            base_link="universe",
            starting_joint_config=[
                # nq=17 with Pinocchio mimic joints (arm_l2/l3/l4 collapsed into arm_l1).
                # Planar base (x, y, cos(yaw), sin(yaw)), SRDF home pose.
                0.0,
                0.0,
                1.0,
                0.0,
                0.5,  # lift_joint
                0.065,  # arm_l1_joint
                0.0,  # wrist_yaw_joint
                0.0,  # wrist_pitch_joint
                0.0,  # wrist_roll_joint
                0.0,  # gripper_finger_left_joint
                0.0,  # gripper_finger_right_joint
                1.0,
                0.0,  # wheel_0_joint (cos, sin)
                1.0,
                0.0,  # wheel_1_joint
                1.0,
                0.0,  # wheel_2_joint
            ],
            obstacles=[
                ObstacleConfig(
                    name="test_box",
                    geom=coal.Box(1.0, 1.0, 0.5),
                    parent_frame="universe",
                    tform=pin.SE3(np.eye(3), np.array([0.0, 0.0, 1.2])).homogeneous,
                    color=np.array([0.0, 0.0, 1.0, 0.5]),
                ),
                ObstacleConfig(
                    name="test_sphere",
                    geom=coal.Sphere(0.3),
                    parent_frame="universe",
                    tform=pin.SE3(np.eye(3), np.array([0.75, 0.0, 0.25])).homogeneous,
                    color=np.array([1.0, 0.0, 0.0, 0.5]),
                    disabled_collisions=["test_box"],
                ),
                ObstacleConfig(
                    name="ground_plane",
                    geom=coal.Box(2.0, 2.0, 0.2),
                    parent_frame="universe",
                    tform=pin.SE3(np.eye(3), np.array([0.0, 0.0, -0.1])).homogeneous,
                    color=np.array([0.5, 0.5, 0.5, 0.5]),
                    disabled_collisions=["base_link", "test_box", "test_sphere"],
                ),
            ],
        ),
        "reachback": RobotModelConfig(
            urdf_path=ROBOPLAN_MODELS_DIR / "reachback_robot_model" / "reachback.urdf",
            srdf_path=ROBOPLAN_MODELS_DIR / "reachback_robot_model" / "reachback.srdf",
            yaml_config_path=ROBOPLAN_MODELS_DIR
            / "reachback_robot_model"
            / "reachback_config.yaml",
            default_joint_group="arm_base",
            ee_names=["arm_gripper_tcp"],
            base_link="universe",
            starting_joint_config=[
                # nq=19 with Pinocchio mimic joints (arm_finger_b_joint mimics arm_finger_a_joint removed).
                # Planar base_joint (x, y, cos(yaw), sin(yaw)), 4 continuous wheel joints (cos, sin each),
                # then arm + gripper driver joint.
                0.0,
                0.0,
                1.0,
                0.0,  # base_joint (planar)
                1.0,
                0.0,  # front_left_wheel_joint
                1.0,
                0.0,  # front_right_wheel_joint
                1.0,
                0.0,  # rear_left_wheel_joint
                1.0,
                0.0,  # rear_right_wheel_joint
                0.0,  # arm_joint_1
                0.0,  # arm_joint_2
                1.57,  # arm_joint_3
                -1.57,  # arm_joint_4
                1.57,  # arm_joint_5
                0.0,  # arm_joint_6
                0.0,  # arm_finger_a_joint
            ],
            obstacles=[
                ObstacleConfig(
                    name="test_box",
                    geom=coal.Box(0.5, 0.5, 0.5),
                    parent_frame="universe",
                    tform=pin.SE3(np.eye(3), np.array([1.5, 0.0, 0.75])).homogeneous,
                    color=np.array([0.0, 0.0, 1.0, 0.5]),
                ),
                ObstacleConfig(
                    name="test_sphere",
                    geom=coal.Sphere(0.3),
                    parent_frame="universe",
                    tform=pin.SE3(np.eye(3), np.array([-1.0, 0.75, 0.5])).homogeneous,
                    color=np.array([1.0, 0.0, 0.0, 0.5]),
                    disabled_collisions=["test_box"],
                ),
                ObstacleConfig(
                    name="ground_plane",
                    geom=coal.Box(5.0, 5.0, 0.2),
                    parent_frame="universe",
                    tform=pin.SE3(np.eye(3), np.array([0.0, 0.0, -0.1255])).homogeneous,
                    color=np.array([0.5, 0.5, 0.5, 0.5]),
                    disabled_collisions=[
                        "front_left_wheel_link",
                        "front_right_wheel_link",
                        "rear_left_wheel_link",
                        "rear_right_wheel_link",
                        "test_box",
                        "test_sphere",
                    ],
                ),
            ],
        ),
    }


def load_octree_from_point_cloud(pointcloud_path: Path, voxel_resolution: float = 0.04):
    """
    Loads a point cloud from a PLY file and converts it into an octree structure.

    Args:
        pointcloud_path: The path to the point cloud.
        voxel_resolution: The voxel resolution, in meters.

    Returns:
        An octree data structure representing the hierarchical spatial partitioning
        of the point cloud.
    """
    from plyfile import PlyData  # Lazy import to not require plyfile

    # Read the PLY file
    ply_data = PlyData.read(pointcloud_path)

    # Access vertex data
    vertices = ply_data["vertex"]
    vertex_array = np.array([vertices["x"], vertices["y"], vertices["z"]]).T
    octree = coal.makeOctree(vertex_array, voxel_resolution)

    return octree


def get_octree():
    """Returns example octree."""
    return ObstacleConfig(
        name="octree_cloud",
        geom=load_octree_from_point_cloud(
            ROBOPLAN_MODELS_DIR / "pointclouds" / "example_point_cloud.ply",
            0.04,  # resolution
        ),
        parent_frame="universe",
        tform=pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.0])).homogeneous,
        color=np.array([0.251, 0.878, 0.816, 1.0]),
    )


def get_home_configuration(
    scene: Scene,
    model_data: RobotModelConfig,
) -> np.ndarray:
    """Return the home configuration for the selected model.

    Args:
        scene: Scene used to read current joint positions.
        model_data: Robot model configuration.

    Returns:
        Full starting joint configuration vector.
    """
    q_full = scene.getCurrentJointPositions()
    q_home = np.array(model_data.starting_joint_config)

    if len(q_home) == len(q_full):
        return q_home.copy()

    print(
        f"Warning: starting_joint_config size ({len(q_home)}) does not match "
        f"model configuration size ({len(q_full)}). Using scene default instead."
    )
    return q_full.copy()
