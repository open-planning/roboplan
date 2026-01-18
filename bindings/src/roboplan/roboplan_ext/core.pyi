from collections.abc import Sequence
import enum
import os
from typing import Annotated, overload

import numpy
from numpy.typing import NDArray


class JointConfiguration:
    """Represents a robot joint configuration."""

    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, joint_names: Sequence[str], positions: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> None: ...

    @property
    def joint_names(self) -> list[str]:
        """The names of the joints."""

    @joint_names.setter
    def joint_names(self, arg: Sequence[str], /) -> None: ...

    @property
    def positions(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """The joint positions, in the same order as the names."""

    @positions.setter
    def positions(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def velocities(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """The joint velocities, in the same order as the names."""

    @velocities.setter
    def velocities(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def accelerations(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """The joint accelerations, in the same order as the names."""

    @accelerations.setter
    def accelerations(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

class CartesianConfiguration:
    """Represents a robot Cartesian configuration."""

    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, base_frame: str, tip_frame: str, tform: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]) -> None: ...

    @property
    def base_frame(self) -> str:
        """The name of the base (or reference) frame."""

    @base_frame.setter
    def base_frame(self, arg: str, /) -> None: ...

    @property
    def tip_frame(self) -> str:
        """The name of the tip (or target) frame."""

    @tip_frame.setter
    def tip_frame(self, arg: str, /) -> None: ...

    @property
    def tform(self) -> Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]:
        """The transformation matrix from the base to the tip frame."""

    @tform.setter
    def tform(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')], /) -> None: ...

class JointType(enum.Enum):
    """Enumeration that describes different types of joints."""

    UNKNOWN = 0

    PRISMATIC = 1

    REVOLUTE = 2

    CONTINUOUS = 3

    PLANAR = 4

    FLOATING = 5

class JointLimits:
    """Contains joint limit information."""

    def __init__(self) -> None: ...

    @property
    def min_position(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """The minimum positions of the joint."""

    @min_position.setter
    def min_position(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def max_position(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """The maximum positions of the joint."""

    @max_position.setter
    def max_position(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def max_velocity(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """The maximum (symmetric) velocities of the joint."""

    @max_velocity.setter
    def max_velocity(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def max_acceleration(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """The maximum (symmetric) accelerations of the joint."""

    @max_acceleration.setter
    def max_acceleration(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def max_jerk(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """The maximum (symmetric) jerks of the joint."""

    @max_jerk.setter
    def max_jerk(self, arg: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], /) -> None: ...

class JointMimicInfo:
    """Contains joint mimic information."""

    def __init__(self) -> None: ...

    @property
    def mimicked_joint_name(self) -> str:
        """The name of the joint being mimicked."""

    @mimicked_joint_name.setter
    def mimicked_joint_name(self, arg: str, /) -> None: ...

    @property
    def scaling(self) -> float:
        """The scaling factor for the mimic relationship."""

    @scaling.setter
    def scaling(self, arg: float, /) -> None: ...

    @property
    def offset(self) -> float:
        """The offset for the mimic relationship."""

    @offset.setter
    def offset(self, arg: float, /) -> None: ...

class JointInfo:
    """Contains joint information relevant to motion planning and control."""

    def __init__(self, joint_type: JointType) -> None: ...

    @property
    def type(self) -> JointType:
        """The type of the joint."""

    @property
    def num_position_dofs(self) -> int:
        """The number of positional degrees of freedom."""

    @property
    def num_velocity_dofs(self) -> int:
        """The number of velocity degrees of freedom."""

    @property
    def limits(self) -> JointLimits:
        """The joint limit information for each degree of freedom."""

    @property
    def mimic_info(self) -> JointMimicInfo | None:
        """The joint mimic information."""

class JointGroupInfo:
    """Contains information about a named group of joints."""

    def __init__(self) -> None: ...

    @property
    def joint_names(self) -> list[str]:
        """The joint names that make up the group."""

    @joint_names.setter
    def joint_names(self, arg: Sequence[str], /) -> None: ...

    @property
    def joint_indices(self) -> list[int]:
        """The joint indices in the group."""

    @joint_indices.setter
    def joint_indices(self, arg: Sequence[int], /) -> None: ...

    @property
    def q_indices(self) -> Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')]:
        """The position vector indices in the group."""

    @q_indices.setter
    def q_indices(self, arg: Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def v_indices(self) -> Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')]:
        """The velocity vector indices in the group."""

    @v_indices.setter
    def v_indices(self, arg: Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')], /) -> None: ...

    @property
    def has_continuous_dofs(self) -> bool:
        """Whether the group has any continuous degrees of freedom."""

    @has_continuous_dofs.setter
    def has_continuous_dofs(self, arg: bool, /) -> None: ...

    @property
    def nq_collapsed(self) -> int:
        """The number of collapsed degrees of freedom."""

    @nq_collapsed.setter
    def nq_collapsed(self, arg: int, /) -> None: ...

    def __repr__(self) -> str: ...

class JointPath:
    """Contains a path of joint configurations."""

    def __init__(self) -> None: ...

    @property
    def joint_names(self) -> list[str]:
        """The list of joint names."""

    @joint_names.setter
    def joint_names(self, arg: Sequence[str], /) -> None: ...

    @property
    def positions(self) -> list[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]:
        """The list of joint configuration positions."""

    @positions.setter
    def positions(self, arg: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]], /) -> None: ...

    def __repr__(self) -> str: ...

class JointTrajectory:
    """Contains a trajectory of joint configurations."""

    def __init__(self) -> None: ...

    @property
    def joint_names(self) -> list[str]:
        """The list of joint names."""

    @joint_names.setter
    def joint_names(self, arg: Sequence[str], /) -> None: ...

    @property
    def times(self) -> list[float]:
        """The list of times."""

    @times.setter
    def times(self, arg: Sequence[float], /) -> None: ...

    @property
    def positions(self) -> list[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]:
        """The list of joint positions."""

    @positions.setter
    def positions(self, arg: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]], /) -> None: ...

    @property
    def velocities(self) -> list[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]:
        """The list of joint velocities."""

    @velocities.setter
    def velocities(self, arg: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]], /) -> None: ...

    @property
    def accelerations(self) -> list[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]:
        """The list of joint acceleration."""

    @accelerations.setter
    def accelerations(self, arg: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]], /) -> None: ...

    def __repr__(self) -> str: ...

class Box:
    """Temporary wrapper struct to represent a box geometry."""

    def __init__(self, x: float, y: float, z: float) -> None: ...

class Sphere:
    """Temporary wrapper struct to represent a sphere geometry."""

    def __init__(self, radius: float) -> None: ...

class Scene:
    """Primary scene representation for planning and control."""

    @overload
    def __init__(self, name: str, urdf_path: str | os.PathLike, srdf_path: str | os.PathLike, package_paths: Sequence[str | os.PathLike] = [], yaml_config_path: str | os.PathLike = ...) -> None: ...

    @overload
    def __init__(self, name: str, urdf: str, srdf: str, package_paths: Sequence[str | os.PathLike] = [], yaml_config_path: str | os.PathLike = ...) -> None: ...

    def getName(self) -> str:
        """Gets the scene's name."""

    def getJointNames(self) -> list[str]:
        """Gets the scene's full joint names, including mimic joints."""

    def getActuatedJointNames(self) -> list[str]:
        """Gets the scene's actuated (non-mimic) joint names."""

    def getJointInfo(self, joint_name: str) -> JointInfo:
        """Gets the information for a specific joint."""

    def configurationDistance(self, q_start: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], q_end: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> float:
        """Gets the distance between two joint configurations."""

    def setRngSeed(self, seed: int) -> None:
        """Sets the seed for the random number generator (RNG)."""

    def randomPositions(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Generates random positions for the robot model."""

    def randomCollisionFreePositions(self, max_samples: int = 1000) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | None:
        """Generates random collision-free positions for the robot model."""

    def hasCollisions(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], debug: bool = False) -> bool:
        """Checks collisions at specified joint positions."""

    def isValidPose(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> bool:
        """
        Checks if the specified joint positions are valid with respect to joint limits.
        """

    def applyMimics(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Applies mimic joint relationships to a position vector."""

    def toFullJointPositions(self, group_name: str, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Converts partial joint positions to full joint positions."""

    def interpolate(self, q_start: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], q_end: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], fraction: float) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Interpolates between two joint configurations."""

    def forwardKinematics(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], frame_name: str) -> Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]:
        """Calculates forward kinematics for a specific frame."""

    def getFrameId(self, name: str) -> int:
        """Get the Pinocchio model ID of a frame by its name."""

    def getJointGroupInfo(self, name: str) -> JointGroupInfo:
        """Get the joint group information of a scene by its name."""

    def getCurrentJointPositions(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get the current joint positions for the full robot state."""

    def setJointPositions(self, positions: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> None:
        """Set the joint positions for the full robot state."""

    def getJointPositionIndices(self, joint_names: Sequence[str]) -> Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')]:
        """Get the joint position indices for a set of joint names."""

    def addBoxGeometry(self, name: str, parent_frame: str, box: Box, tform: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')], color: Annotated[NDArray[numpy.float64], dict(shape=(4), order='C')]) -> None:
        """Adds a box geometry to the scene."""

    def addSphereGeometry(self, name: str, parent_frame: str, sphere: Sphere, tform: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')], color: Annotated[NDArray[numpy.float64], dict(shape=(4), order='C')]) -> None:
        """Adds a sphere geometry to the scene."""

    def updateGeometryPlacement(self, name: str, parent_frame: str, tform: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]) -> None:
        """Updates the placement of an object geometry in the scene."""

    def removeGeometry(self, name: str) -> None:
        """Removes a geometry from the scene."""

    def getCollisionGeometryIDs(self, body: str) -> list[int]:
        """
        Gets a list of collision geometry IDs corresponding to a specified body.
        """

    def setCollisions(self, body1: str, body2: str, enable: bool) -> None:
        """Sets the allowable collisions for a pair of bodies in the model."""

    def __repr__(self) -> str: ...

def computeFramePath(scene: Scene, q_start: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], q_end: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], frame_name: str, max_step_size: float) -> list[Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]]:
    """Computes the Cartesian path of a specified frame."""

def hasCollisionsAlongPath(scene: Scene, q_start: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], q_end: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], max_step_size: float) -> bool:
    """Checks collisions along a specified configuration space path."""

class PathShortcutter:
    """Shortcuts joint paths with random sampling and checking connections."""

    def __init__(self, scene: Scene, group_name: str) -> None: ...

    def shortcut(self, path: JointPath, max_step_size: float, max_iters: int = 100, seed: int = 0) -> JointPath:
        """Attempts to shortcut a specified path."""

    def getPathLengths(self, path: JointPath) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """
        Computes configuration distances from the start to each pose in a path.
        """

    def getNormalizedPathScaling(self, path: JointPath) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Computes length-normalized scaling values along a JointPath."""

    def getConfigurationfromNormalizedPathScaling(self, path: JointPath, path_scalings: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], value: float) -> "std::pair<Eigen::Matrix<double, -1, 1, 0, -1, 1>, unsigned long>":
        """Gets joint configurations from a path with normalized joint scalings."""

def collapseContinuousJointPositions(scene: Scene, group_name: str, q_orig: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
    """
    Collapses a joint position vector's continuous joints for downstream algorithms.
    """

def expandContinuousJointPositions(scene: Scene, group_name: str, q_orig: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
    """
    Expands a joint position vector's continuous joints from downstream algorithms.
    """
