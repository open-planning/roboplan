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
        """
        The name of the base (or reference) frame. An empty frame name means the world frame.
        """

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
    def link_names(self) -> list[str]:
        """The link (body) names that make up the group."""

    @link_names.setter
    def link_names(self, arg: Sequence[str], /) -> None: ...

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

class CartesianPath:
    """Contains a path of Cartesian configurations."""

    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, base_frames: Sequence[str], tip_frames: Sequence[str], tforms: Sequence[Sequence[Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]]]) -> None: ...

    @property
    def base_frames(self) -> list[str]:
        """
        The names of the base frames. An empty frame name means the world frame.
        """

    @base_frames.setter
    def base_frames(self, arg: Sequence[str], /) -> None: ...

    @property
    def tip_frames(self) -> list[str]:
        """The names of the tip frames."""

    @tip_frames.setter
    def tip_frames(self, arg: Sequence[str], /) -> None: ...

    @property
    def tforms(self) -> list[list[Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]]]:
        """The Cartesian transforms from each base frame to each tip frame."""

    @tforms.setter
    def tforms(self, arg: Sequence[Sequence[Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]]], /) -> None: ...

    def __repr__(self) -> str: ...

class CartesianTrajectory:
    """Contains a trajectory of Cartesian configurations."""

    @overload
    def __init__(self) -> None: ...

    @overload
    def __init__(self, base_frames: Sequence[str], tip_frames: Sequence[str], times: Sequence[float], tforms: Sequence[Sequence[Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]]]) -> None: ...

    @property
    def base_frames(self) -> list[str]:
        """
        The names of the base frames. An empty frame name means the world frame.
        """

    @base_frames.setter
    def base_frames(self, arg: Sequence[str], /) -> None: ...

    @property
    def tip_frames(self) -> list[str]:
        """The names of the tip frames."""

    @tip_frames.setter
    def tip_frames(self, arg: Sequence[str], /) -> None: ...

    @property
    def times(self) -> list[float]:
        """The list of times."""

    @times.setter
    def times(self, arg: Sequence[float], /) -> None: ...

    @property
    def tforms(self) -> list[list[Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]]]:
        """The Cartesian transforms from each base frame to each tip frame."""

    @tforms.setter
    def tforms(self, arg: Sequence[Sequence[Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]]], /) -> None: ...

    def __repr__(self) -> str: ...

class Box:
    """Temporary wrapper struct to represent a box geometry."""

    def __init__(self, x: float, y: float, z: float) -> None: ...

class Sphere:
    """Temporary wrapper struct to represent a sphere geometry."""

    def __init__(self, radius: float) -> None: ...

class Cylinder:
    """Temporary wrapper struct to represent a cylinder geometry."""

    def __init__(self, radius: float, length: float) -> None: ...

class Mesh:
    """Temporary wrapper struct to represent a triangle mesh geometry."""

    def __init__(self, filename: str | os.PathLike, scale: Annotated[NDArray[numpy.float64], dict(shape=(3), order='C')] = ...) -> None: ...

class OcTree:
    """Temporary wrapper struct to represent an octree geometry."""

    def __init__(self, boxes: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(6), order='C')]], resolution: float) -> None: ...

class PinocchioSceneDescription:
    """Pinocchio model and collision geometry."""

def loadTextFile(path: str | os.PathLike) -> str:
    """Reads a text file from disk."""

class YamlNode:
    """Parsed YAML document."""

def loadJointLimitsConfig(path: str | os.PathLike) -> YamlNode:
    """Loads a joint-limits config from disk."""

def loadUrdfSceneDescriptionFromXml(urdf_xml: str, package_paths: Sequence[str | os.PathLike] = []) -> PinocchioSceneDescription:
    """
    Builds a PinocchioSceneDescription from URDF XML. `package_paths` resolve `package://` mesh paths.
    """

def loadUrdfSceneDescription(urdf_path: str | os.PathLike, package_paths: Sequence[str | os.PathLike] = []) -> PinocchioSceneDescription:
    """
    Loads a URDF file into a PinocchioSceneDescription. `package_paths` resolve `package://` mesh paths.
    """

def loadMjcfModel(mjcf_path: str | os.PathLike) -> PinocchioSceneDescription:
    """Loads an MJCF file into a PinocchioSceneDescription."""

class Scene:
    """Primary scene representation for planning and control."""

    def __init__(self, name: str, description: PinocchioSceneDescription) -> None: ...

    def getName(self) -> str:
        """Gets the scene's name."""

    def getJointNames(self) -> list[str]:
        """Gets the scene's actuated joint names (non-mimic joints only)."""

    def getJointNamesWithMimics(self) -> list[str]:
        """Gets the scene's full joint names, including mimic joints."""

    def getJointInfo(self, joint_name: str) -> JointInfo:
        """Gets the information for a specific joint."""

    def configurationDistance(self, q_start: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], q_end: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> float:
        """Gets the distance between two joint configurations."""

    def setRngSeed(self, seed: int) -> None:
        """Sets the seed for the random number generator (RNG)."""

    def randomPositions(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Generates random positions for the robot model."""

    @overload
    def randomCollisionFreePositions(self, max_samples: int = 1000) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | None:
        """Generates random collision-free positions for the robot model."""

    @overload
    def randomCollisionFreePositions(self, joint_names: Sequence[str], q_reference: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], max_samples: int = 1000) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | None:
        """
        Generates random collision-free positions, randomizing only the specified joints and taking all others from the reference configuration.
        """

    def hasCollisions(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], debug: bool = False) -> bool:
        """Checks collisions at specified joint positions."""

    def isValidConfiguration(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> bool:
        """
        Checks if the specified joint positions are valid with respect to joint limits.
        """

    def clampToValidConfiguration(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Clamps the specified joint positions to valid joint limits."""

    def toFullJointPositions(self, group_name: str, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Converts partial joint positions to full joint positions."""

    def toFullJointVelocities(self, group_name: str, v: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Converts partial joint velocities to full joint velocities."""

    def interpolate(self, q_start: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], q_end: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], fraction: float) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Interpolates between two joint configurations."""

    def integrate(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], v: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """
        Integrates a velocity vector from a configuration using Lie group operations.
        """

    def difference(self, q_start: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], q_end: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """
        Computes the velocity vector taking one configuration to another, using Lie group operations. The inverse of integrate().
        """

    def forwardKinematics(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], frame_name: str, base_frame: str = '') -> Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]:
        """Calculates forward kinematics for a specific frame."""

    def computeFrameJacobian(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], frame_name: str, local: bool = True) -> Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='F')]:
        """
        Computes the frame Jacobian (6 x nv): LOCAL frame if `local` is true, else WORLD.
        """

    def computeRelativeFrameJacobian(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], frame_name: str, base_frame: str, local: bool = True) -> Annotated[NDArray[numpy.float64], dict(shape=(None, None), order='F')]:
        """Computes the Jacobian of a frame's velocity relative to a base frame."""

    def getFrameId(self, name: str) -> int:
        """Get the Pinocchio model ID of a frame by its name."""

    def getJointGroupInfo(self, name: str) -> JointGroupInfo:
        """Get the joint group information of a scene by its name."""

    def importSrdf(self, srdf_xml: str) -> None:
        """Applies groups and disabled collision pairs from an SRDF document."""

    def importJointLimitsFromConfig(self, yaml_config: YamlNode) -> None:
        """Overrides joint limits from a parsed configuration."""

    def addGroupFromChain(self, name: str, base_link: str, tip_link: str) -> None:
        """Adds a joint group defined by a kinematic chain."""

    def addGroupFromGroups(self, name: str, group_names: Sequence[str]) -> None:
        """Adds a joint group by concatenating existing groups."""

    def addGroup(self, name: str, joint_names: Sequence[str], extra_link_names: Sequence[str] = []) -> None:
        """Adds a joint group from an explicit list of joints."""

    def getCurrentJointPositions(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Get the current Pinocchio configuration vector (model.nq)."""

    def getCurrentJointPositionsWithMimics(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """
        Get current joint positions in getJointNamesWithMimics() order, including mimic values.
        """

    def setJointPositions(self, positions: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> None:
        """Set the joint positions for the full robot state."""

    def getJointPositionIndices(self, joint_names: Sequence[str]) -> Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')]:
        """Get the joint position indices for a set of joint names."""

    def getJointVelocityIndices(self, joint_names: Sequence[str]) -> Annotated[NDArray[numpy.int32], dict(shape=(None,), order='C')]:
        """Get the joint velocity indices for a set of joint names."""

    def getPositionLimitVectors(self, group_name: str = '', collapsed: bool = False) -> tuple[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]:
        """Get the joint position limit vectors for a specified group."""

    def getVelocityLimitVectors(self, group_name: str = '') -> tuple[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]:
        """Get the joint velocity limit vectors for a specified group."""

    def getAccelerationLimitVectors(self, group_name: str = '') -> tuple[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]:
        """Get the joint acceleration limit vectors for a specified group."""

    def getJerkLimitVectors(self, group_name: str = '') -> tuple[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]]:
        """Get the joint jerk limit vectors for a specified group."""

    def addBoxGeometry(self, name: str, parent_frame: str, box: Box, tform: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')], color: Annotated[NDArray[numpy.float64], dict(shape=(4), order='C')]) -> None:
        """Adds a box geometry to the scene."""

    def addSphereGeometry(self, name: str, parent_frame: str, sphere: Sphere, tform: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')], color: Annotated[NDArray[numpy.float64], dict(shape=(4), order='C')]) -> None:
        """Adds a sphere geometry to the scene."""

    def addCylinderGeometry(self, name: str, parent_frame: str, cylinder: Cylinder, tform: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')], color: Annotated[NDArray[numpy.float64], dict(shape=(4), order='C')]) -> None:
        """Adds a cylinder geometry to the scene."""

    def addMeshGeometry(self, name: str, parent_frame: str, mesh: Mesh, tform: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')], color: Annotated[NDArray[numpy.float64], dict(shape=(4), order='C')]) -> None:
        """Adds a triangle mesh geometry to the scene."""

    def addOcTreeGeometry(self, name: str, parent_frame: str, octree: OcTree, tform: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')], color: Annotated[NDArray[numpy.float64], dict(shape=(4), order='C')]) -> None:
        """Adds an octree geometry to the scene."""

    def updateGeometryPlacement(self, name: str, parent_frame: str, tform: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]) -> None:
        """Updates the placement of an object geometry in the scene."""

    def removeGeometry(self, name: str) -> None:
        """Removes a geometry from the scene."""

    def attachObject(self, object_name: str, parent_frame: str, allowed_collision_bodies: Sequence[str] = [], tform: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')] | None = None) -> None:
        """
        Attaches an object in the scene to a parent frame. Disables collisions between the object and all geometry objects attached to the parent frame, as well as the object and all geometries or frames specified in allowed_collision_bodies.
        """

    def detachObject(self, object_name: str) -> None:
        """Detaches an object from its parent frame, returning it to the world."""

    def reparentAttachedObject(self, object_name: str, parent_frame: str, allowed_collision_bodies: Sequence[str] = [], tform: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')] | None = None) -> None:
        """
        Moves an attached object to a new parent frame, such as when handing it over.
        """

    def isObjectAttached(self, object_name: str) -> bool:
        """Checks whether an object is attached via attachObject."""

    def getCollisionGeometryIDs(self, body: str) -> list[int]:
        """
        Gets a list of collision geometry IDs corresponding to a specified body.
        """

    def getRobotCollisionGeometryIds(self) -> list[int]:
        """
        Gets the collision geometry IDs belonging to the robot model itself (excluding objects added to the scene).
        """

    @overload
    def setCollisions(self, body1: str, body2: str, enable: bool) -> None:
        """Sets the allowable collisions for a pair of bodies in the model."""

    @overload
    def setCollisions(self, pairs: Sequence[tuple[str, str]], enable: bool) -> None:
        """
        Sets the allowable collisions for many body pairs, rebuilding collision data once.
        """

    def allowAdjacentLinkCollisions(self) -> None:
        """
        Allows collisions between every parent-child link pair in the kinematic tree.
        """

    def __repr__(self) -> str: ...

class SceneContext:
    """
    Per-thread scratch for scene queries: Pinocchio data, geometry data, the broadphase tree, a random number generator, and a current configuration.

    Each method is the Scene query of the same name, run against this context's private scratch; give each thread its own. Adding or removing geometry, or changing collision pairs, leaves that scratch stale and the collision queries report the mismatch, so build a new context after such a change.
    """

    def __init__(self, scene: Scene) -> None:
        """Builds a context over `scene`'s current collision geometry."""

    def hasCollisions(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], debug: bool = False) -> bool:
        """Checks collisions at the given joint positions."""

    def computeDistances(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], broadphase_margin: float | None = None) -> None:
        """
        Computes the distance for every active collision pair into this context's data.
        """

    def forwardKinematics(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], frame_name: str, base_frame: str = '') -> Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]:
        """Calculates forward kinematics for a specific frame."""

    def updateFramePlacements(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> None:
        """Runs forward kinematics and refreshes every frame placement."""

    def computeJointJacobians(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> None:
        """Computes the joint Jacobians for every joint."""

    def setRngSeed(self, seed: int) -> None:
        """Sets the seed of this context's random number generator."""

    def randomPositions(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """Generates random positions using this context's RNG."""

    @overload
    def randomCollisionFreePositions(self, max_samples: int = 1000) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | None:
        """
        Generates random collision-free positions using this context's RNG and scratch.
        """

    @overload
    def randomCollisionFreePositions(self, joint_names: Sequence[str], q_reference: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], max_samples: int = 1000) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | None:
        """
        Generates random collision-free positions using this context's RNG and scratch, randomizing only the specified joints and taking all others from the reference configuration.
        """

    def getJointPositions(self) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """This context's current joint positions."""

    def setJointPositions(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> None:
        """Sets this context's current joint positions."""

    def toFullJointPositions(self, group_name: str, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """
        Converts partial joint positions to full ones, filling non-group joints from this context's current configuration.
        """

    def isGeometryCurrent(self) -> bool:
        """
        Whether the scene's collision geometry is still the one this context was built from.
        """

    def getScene(self) -> Scene:
        """The Scene this context was built from."""

class Constraint:
    """
    Base class for kinematic constraints that planners enforce by projection.
    """

    def dimension(self) -> int:
        """The number of residual coordinates this constraint contributes."""

    def getGroupName(self) -> str:
        """
        The joint group whose degrees of freedom a projection is allowed to move.
        """

    def computeError(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """
        Computes the constraint residual at a configuration, which is zero where the constraint is satisfied and otherwise the signed overshoot past the nearest bound.
        """

    def satisfies(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> bool:
        """Checks whether a configuration satisfies this constraint."""

class PoseConstraint(Constraint):
    """
    Constrains the pose of a robot frame to a box of allowed task space displacements.
    """

    def __init__(self, scene: Scene, group_name: str, frame_name: str, lower_bounds: Annotated[NDArray[numpy.float64], dict(shape=(6), order='C')] = ..., upper_bounds: Annotated[NDArray[numpy.float64], dict(shape=(6), order='C')] = ..., tform: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')] = ..., reference_frame: str = '', position_tolerance: float = 0.001, orientation_tolerance: float = 0.005) -> None: ...

    def computeDisplacement(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> Annotated[NDArray[numpy.float64], dict(shape=(6), order='C')]:
        """
        Computes the constrained frame's [x, y, z, roll, pitch, yaw] displacement from the region frame.
        """

    @property
    def frame_name(self) -> str:
        """The name of the constrained frame."""

    @property
    def reference_frame(self) -> str:
        """The name of the frame the region transform is expressed in."""

    @property
    def tform(self) -> Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]:
        """The pose of the region frame relative to the reference frame."""

    @property
    def lower_bounds(self) -> Annotated[NDArray[numpy.float64], dict(shape=(6), order='C')]:
        """The lower bounds on the frame's task space displacement."""

    @property
    def upper_bounds(self) -> Annotated[NDArray[numpy.float64], dict(shape=(6), order='C')]:
        """The upper bounds on the frame's task space displacement."""

    @property
    def position_tolerance(self) -> float:
        """
        The position residual norm, in meters, below which the constraint is satisfied.
        """

    @property
    def orientation_tolerance(self) -> float:
        """
        The orientation residual norm, in radians, below which the constraint is satisfied.
        """

class ConstraintProjectorOptions:
    """Options struct for constraint projection."""

    def __init__(self, path_step_size: float = 0.1, max_iters: int = 50, correction_step_size: float = 1.0, damping: float = 1e-06, convergence_ratio: float = 0.1) -> None: ...

    @property
    def path_step_size(self) -> float:
        """The configuration-space step size taken between projections."""

    @path_step_size.setter
    def path_step_size(self, arg: float, /) -> None: ...

    @property
    def max_iters(self) -> int:
        """The maximum number of projection iterations before giving up."""

    @max_iters.setter
    def max_iters(self, arg: int, /) -> None: ...

    @property
    def correction_step_size(self) -> float:
        """
        The fraction of each computed correction to apply per projection iteration.
        """

    @correction_step_size.setter
    def correction_step_size(self, arg: float, /) -> None: ...

    @property
    def damping(self) -> float:
        """Damping value for the Jacobian pseudoinverse."""

    @damping.setter
    def damping(self, arg: float, /) -> None: ...

    @property
    def convergence_ratio(self) -> float:
        """
        The fraction of each constraint's tolerance the projection converges to, leaving headroom for the interpolation between projected configurations. Must be in (0, 1].
        """

    @convergence_ratio.setter
    def convergence_ratio(self, arg: float, /) -> None: ...

class ConstraintProjector:
    """
    Projects joint configurations onto the intersection of a set of constraints.
    """

    def __init__(self, scene: Scene, group_name: str, constraints: Sequence[Constraint], options: ConstraintProjectorOptions = ...) -> None: ...

    def satisfies(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> bool:
        """Checks whether a configuration satisfies every constraint."""

    def project(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | None:
        """
        Projects a configuration onto the constraints, returning None if the projection did not converge.
        """

    def satisfiesAlongPath(self, q_start: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], q_end: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], max_step_size: float, check_endpoints: bool = True) -> bool:
        """
        Checks whether a straight-line configuration-space path stays inside the constraints.
        """

    def empty(self) -> bool:
        """Whether the projector holds no constraints, and is therefore a no-op."""

@overload
def computeFramePath(scene: Scene, q_start: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], q_end: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], frame_name: str, max_step_size: float) -> list[Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]]:
    """
    Computes the Cartesian path of a specified frame by interpolating sparse positions.
    """

@overload
def computeFramePath(scene: Scene, q_vec: Sequence[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]], frame_name: str) -> list[Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]]:
    """
    Computes the Cartesian path of a specified frame using a vector of provided points.
    """

def hasCollisionsAlongPath(scene: Scene, q_start: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], q_end: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], max_step_size: float, bisection: bool = False, check_endpoints: bool = True) -> bool:
    """
    Checks collisions along a specified configuration space path. Uses the Scene's own collision scratch, so it is not safe to call concurrently on one Scene.
    """

def computePathLength(scene: Scene, group_name: str, path: JointPath) -> float:
    """Computes the total configuration-space length of a joint path."""

class PathShortcuttingOptions:
    """Options struct for path shortcutting."""

    def __init__(self, group_name: str = '', max_step_size: float = 0.05, max_iters: int = 100, seed: int = 0, max_convergence_iters: int = 20, redundant_removal_iters: int = 20, constraint_projection: ConstraintProjectorOptions = ..., constrained_min_detour_ratio: float = 1.1) -> None: ...

    @property
    def group_name(self) -> str:
        """The joint group name to be used for path shortcutting."""

    @group_name.setter
    def group_name(self, arg: str, /) -> None: ...

    @property
    def max_step_size(self) -> float:
        """
        Maximum step size used in collision checking, and the minimum separable distance between points in a shortcut.
        """

    @max_step_size.setter
    def max_step_size(self, arg: float, /) -> None: ...

    @property
    def max_iters(self) -> int:
        """Maximum number of iterations of random sampling."""

    @max_iters.setter
    def max_iters(self, arg: int, /) -> None: ...

    @property
    def seed(self) -> int:
        """Seed for the random generator. If < 0, a random seed is used."""

    @seed.setter
    def seed(self, arg: int, /) -> None: ...

    @property
    def max_convergence_iters(self) -> int:
        """
        Stop early once this many consecutive iterations fail to apply a shortcut. A value of 0 disables early stopping.
        """

    @max_convergence_iters.setter
    def max_convergence_iters(self, arg: int, /) -> None: ...

    @property
    def redundant_removal_iters(self) -> int:
        """
        Cadence (in iterations) at which to interleave the redundant-vertex removal pass that cleans up the micro-segments introduced by shortcutting. Must be greater than 0.
        """

    @redundant_removal_iters.setter
    def redundant_removal_iters(self, arg: int, /) -> None: ...

    @property
    def constraint_projection(self) -> ConstraintProjectorOptions:
        """Options for the projection used when shortcutting with constraints."""

    @constraint_projection.setter
    def constraint_projection(self, arg: ConstraintProjectorOptions, /) -> None: ...

    @property
    def constrained_min_detour_ratio(self) -> float:
        """
        With constraints, the ratio of a section's length to the straight distance between its ends below which no projected shortcut is attempted.
        """

    @constrained_min_detour_ratio.setter
    def constrained_min_detour_ratio(self, arg: float, /) -> None: ...

class PathShortcutter:
    """Shortcuts joint paths with random sampling and checking connections."""

    def __init__(self, scene: Scene, options: PathShortcuttingOptions) -> None: ...

    def shortcut(self, path: JointPath, constraints: Sequence[Constraint] = []) -> JointPath:
        """
        Attempts to shortcut a specified path, optionally keeping it on a set of constraints.
        """

    def getPathLengths(self, path: JointPath) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
        """
        Computes configuration distances from the start to each pose in a path.
        """

    def getConfigurationAtPathLength(self, path: JointPath, path_lengths: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], length: float) -> tuple[Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], int]:
        """Gets the joint configuration at a given length along a path."""

def poseError(a: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')], b: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]) -> tuple[float, float]:
    """
    Computes the (position error [m], orientation error [rad]) between two SE(3) transforms expressed in the same frame.
    """

def interpolatePose(start: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')], end: Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')], fraction: float) -> Annotated[NDArray[numpy.float64], dict(shape=(4, 4), order='F')]:
    """
    Interpolates between two SE(3) transforms: linear in position, SLERP in orientation.
    """

def collapseContinuousJointPositions(scene: Scene, group_name: str, q_orig: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
    """
    Collapses a joint position vector's continuous joints for downstream algorithms.
    """

def expandContinuousJointPositions(scene: Scene, group_name: str, q_orig: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]) -> Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')]:
    """
    Expands a joint position vector's continuous joints from downstream algorithms.
    """

class RobotBodyFilterMethod(enum.Enum):
    """
    The test used by RobotBodyFilter to classify points near the robot geometry.
    """

    Narrowphase = 0
    """
    Exact for every geometry type, including meshes: each candidate point gets a Coal narrowphase query against the padded geometry (one GJK/BVH query per point).
    """

    PaddedObb = 1
    """
    Conservative: each candidate point is checked against the geometry's padded oriented bounding box (OBB). Much faster, but over-removes points near box corners, so it always removes a superset of Narrowphase's points.
    """

class RobotBodyFilterOptions:
    """Options struct for the robot body filter."""

    def __init__(self, padding: float, method: RobotBodyFilterMethod = RobotBodyFilterMethod.Narrowphase, num_threads: int = 0) -> None: ...

    @property
    def padding(self) -> float:
        """
        Distance, in meters, around the robot's collision geometry within which points are considered part of the robot body. Must be non-negative.
        """

    @padding.setter
    def padding(self, arg: float, /) -> None: ...

    @property
    def method(self) -> RobotBodyFilterMethod:
        """The classification test to use."""

    @method.setter
    def method(self, arg: RobotBodyFilterMethod, /) -> None: ...

    @property
    def num_threads(self) -> int:
        """
        Number of threads used to classify points, or 0 to use all hardware threads. At most one thread is spawned per block of points, so small clouds run serially.
        """

    @num_threads.setter
    def num_threads(self, arg: int, /) -> None: ...

class RobotBodyFilter:
    """
    Filters points that lie on or near the robot's own collision geometry.

    Removes the robot's body from a sensor point cloud (or octree cells) so it does not see itself as an obstacle. Both methods share a broadphase cull against the padded world-frame AABB of every robot collision geometry and differ only in the test run on the surviving candidates; see RobotBodyFilterMethod.

    Thread safety and lifetime: the filter owns private Pinocchio scratch, so distinct filters may run concurrently on one Scene, but one filter must not be shared across threads. Only the robot's own collision geometry is filtered against, and it is copied at construction, so adding or removing scene objects does not require a rebuild.
    """

    def __init__(self, scene: Scene, options: RobotBodyFilterOptions) -> None:
        """Constructs a filter over the scene's current robot collision geometry."""

    def computeMask(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], points: Annotated[NDArray[numpy.float64], dict(shape=(None, 3), writable=False)], extra_padding: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | None = None) -> Annotated[NDArray[numpy.bool_], dict(shape=(None,), order='C')]:
        """
        Classifies each point against the padded robot geometry at a joint configuration.
        """

    def filterPoints(self, q: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')], points: Annotated[NDArray[numpy.float64], dict(shape=(None, 3), writable=False)], extra_padding: Annotated[NDArray[numpy.float64], dict(shape=(None,), order='C')] | None = None) -> Annotated[NDArray[numpy.float64], dict(shape=(None, 3), order='C')]:
        """
        Returns only the points outside the padded robot body at a joint configuration.
        """

    def getOptions(self) -> RobotBodyFilterOptions:
        """The filter options."""
