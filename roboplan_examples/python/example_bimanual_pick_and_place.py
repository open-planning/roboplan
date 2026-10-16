#!/usr/bin/env python3

import itertools
import threading
import time
from dataclasses import dataclass

try:
    import coal
except ModuleNotFoundError:
    import hppfcl as coal

import numpy as np
import pinocchio as pin
import tyro
import xacro
from common import ObstacleConfig, attach_object, detach_object, get_model_data
from pinocchio.visualize import ViserVisualizer

from roboplan.cartesian_planning import (
    CartesianPathPlanner,
    CartesianPlannerComponents,
    CartesianPlannerOptions,
)
from roboplan.core import (
    CartesianConfiguration,
    CartesianPath,
    JointConfiguration,
    JointPath,
    JointTrajectory,
    PathShortcutter,
    PathShortcuttingOptions,
    Scene,
    hasCollisionsAlongPath,
    loadJointLimitsConfig,
    loadUrdfSceneDescriptionFromXml,
)
from roboplan.example_models import get_package_share_dir
from roboplan.optimal_ik import (
    ConfigurationTask,
    ConfigurationTaskOptions,
    FrameTask,
    FrameTaskOptions,
    IterativeSolveOptions,
    Oink,
    PositionLimit,
    RelativePoseConstraint,
    VelocityLimit,
)
from roboplan.rrt import (
    RRT,
    ConstraintProjectorOptions,
    PoseConstraint,
    RRTOptions,
)
from roboplan.toppra import PathParameterizerTOPPRA, SplineFittingMode, TOPPRAOptions
from roboplan.visualization import visualizePath

MODEL_NAME = "tiago_pro"

OBJECT_NAME = "object"
LEFT_GRASP_FRAME = "gripper_left_grasping_link"
RIGHT_GRASP_FRAME = "gripper_right_grasping_link"
GRASP_FRAMES = [LEFT_GRASP_FRAME, RIGHT_GRASP_FRAME]
HAND_LINKS = [
    f"gripper_{side}_{link}_link"
    for side in ("left", "right")
    for link in (
        "base",
        "base_finger_left",
        "base_finger_right",
        "inner_finger_left",
        "inner_finger_right",
        "outer_finger_left",
        "outer_finger_right",
        "fingertip_left",
        "fingertip_right",
    )
]
FINGER_JOINTS = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
BASE_LINKS = [
    "base_link",
    "base_dock_link",
    *[
        f"{link}_{side}_link"
        for link in ("wheel", "suspension")
        for side in ("front_left", "front_right", "rear_left", "rear_right")
    ],
]

# The object is a long bar, held at each end by one hand, top-down, with the fingers closing
# across its width. The grasping frame's x axis points out of the gripper, and its fingers close
# along its y axis.
OBJECT_SIZE = (0.04, 0.30, 0.08)
GRASP_ROTATION = pin.utils.rotate("y", np.pi / 2) @ pin.utils.rotate("x", -np.pi / 2)
OBJECT_T_LEFT_TCP = pin.SE3(GRASP_ROTATION, np.array([0.0, 0.11, 0.025])).homogeneous
OBJECT_T_RIGHT_TCP = pin.SE3(GRASP_ROTATION, np.array([0.0, -0.11, 0.025])).homogeneous
FINGER_OPEN = 0.07
FINGER_CLOSED = 0.04

# Two tables of different heights in front of the robot.
TABLE_XYS = [(0.9, 0.6), (0.9, -0.6)]
TABLE_SIZES_XY = [(0.4, 0.6), (0.4, 0.6)]
TABLE_HEIGHTS = [0.45, 0.7]
BASE_JOINT = "base_joint"
LIFT_GROUP = "base_arms"

# Each hand can grasp the bar either way round its approach axis.
# A pick is solved with a goal for each hand, from a seed with the base BASE_STANDOFF
# behind the object, facing it, and the rest of the joints random.
HAND_FLIPS = list(itertools.product((False, True), repeat=2))
BASE_STANDOFF = 0.65
JOINT_LIMIT_MARGIN = 0.04
IK_ATTEMPTS = 20
POSTURE_WEIGHT = 0.05

APPROACH_DISTANCE = 0.1
CARRY_SPEED = 0.15
COLLISION_CHECK_STEP_SIZE = 0.02
# Straight-line motions run close to the object, so they are checked more finely.
STRAIGHT_CHECK_STEP_SIZE = COLLISION_CHECK_STEP_SIZE / 4.0
TRAJ_DT = 0.01


def get_object_pose_on_table(table: int) -> np.ndarray:
    """Returns the world pose of the object resting on a table."""
    z = TABLE_HEIGHTS[table] + OBJECT_SIZE[2] / 2.0 + 0.002
    return pin.SE3(np.eye(3), np.array([*TABLE_XYS[table], z])).homogeneous


def above(world_T_object: np.ndarray) -> np.ndarray:
    """Returns a pose APPROACH_DISTANCE above another."""
    world_T_above = world_T_object.copy()
    world_T_above[2, 3] += APPROACH_DISTANCE
    return world_T_above


def get_object_T_tcps(hand_flips: tuple[bool, bool]) -> tuple[np.ndarray, np.ndarray]:
    """
    Returns the left and right hands' grasp poses on the object, each optionally turned a half
    turn about its approach axis.
    """
    flip = pin.SE3(pin.utils.rotate("x", np.pi), np.zeros(3)).homogeneous
    return tuple(
        object_T_tcp @ flip if flipped else object_T_tcp
        for object_T_tcp, flipped in zip(
            (OBJECT_T_LEFT_TCP, OBJECT_T_RIGHT_TCP), hand_flips
        )
    )


@dataclass
class Grasp:
    hand_flips: tuple[bool, bool]
    """Whether each hand is turned a half turn about its approach axis."""

    q_grasp: np.ndarray
    """The joint group positions grasping the object."""

    q_above: np.ndarray
    """The joint group positions for the pre-grasp above the object."""


def make_grasp_constraint(scene: Scene, group_name: str) -> PoseConstraint:
    """Returns a constraint holding the right hand at its current pose relative to the left."""
    left_T_right = scene.forwardKinematics(
        scene.getCurrentJointPositions(), RIGHT_GRASP_FRAME, LEFT_GRASP_FRAME
    )
    return PoseConstraint(
        scene,
        group_name,
        RIGHT_GRASP_FRAME,
        lower_bounds=np.zeros(6),
        upper_bounds=np.zeros(6),
        tform=left_T_right,
        reference_frame=LEFT_GRASP_FRAME,
    )


def make_cartesian_planner(
    scene: Scene, left_T_right: np.ndarray, q_start: np.ndarray
) -> CartesianPathPlanner:
    """
    Builds a Cartesian planner for LIFT_GROUP with the default setup + a RelativePoseConstraint
    between the hands.
    """
    options = CartesianPlannerOptions(
        group_name=LIFT_GROUP,
        dt=TRAJ_DT,
        max_linear_speed=CARRY_SPEED,
        position_tolerance=0.001,
        orientation_tolerance=0.005,
    )
    _, v_max = scene.getVelocityLimitVectors(LIFT_GROUP)
    lift_oink = Oink(scene, LIFT_GROUP)
    goal = CartesianConfiguration(  # In the world frame
        "", LEFT_GRASP_FRAME, scene.forwardKinematics(q_start, LEFT_GRASP_FRAME)
    )
    components = CartesianPlannerComponents()
    components.oink = lift_oink
    components.tracking_tasks = [
        FrameTask(
            lift_oink,
            scene,
            goal,
            FrameTaskOptions(
                position_cost=options.position_cost,
                orientation_cost=options.orientation_cost,
                task_gain=options.task_gain,
                lm_damping=options.lm_damping,
                priority=1,
            ),
        )
    ]
    components.extra_tasks = [
        ConfigurationTask(
            lift_oink,
            q_start[lift_oink.q_indices],
            np.full(lift_oink.num_variables, options.config_task_weight),
            ConfigurationTaskOptions(priority=2),
        )
    ]
    components.constraints = [
        VelocityLimit(lift_oink, options.dt, np.abs(v_max)),
        PositionLimit(lift_oink, options.position_limit_gain),
        RelativePoseConstraint(
            lift_oink, scene, LEFT_GRASP_FRAME, RIGHT_GRASP_FRAME, left_T_right
        ),
    ]
    return CartesianPathPlanner(scene, options, components)


def set_fingers(
    scene: Scene, viz: ViserVisualizer, position: float, duration: float = 0.5
):
    """Opens or closes both grippers, animating the motion."""
    finger_indices = scene.getJointPositionIndices(FINGER_JOINTS)
    q_start = scene.getCurrentJointPositions()
    q = q_start.copy()
    for alpha in np.linspace(0.0, 1.0, int(duration / TRAJ_DT)):
        q[finger_indices] = (1.0 - alpha) * q_start[finger_indices] + alpha * position
        viz.display(q)
        time.sleep(TRAJ_DT)
    scene.setJointPositions(q)


class GraspSolver:
    """Solves for ways of picking up the object with both hands and putting it down."""

    def __init__(self, scene: Scene, group_name: str):
        self.scene = scene
        self.group_name = group_name
        group_info = scene.getJointGroupInfo(group_name)
        self.q_indices = group_info.q_indices

        # The planar base joint's positions are (x, y, cos(yaw), sin(yaw)). Pick seeds randomize
        # every joint but the base.
        self.base_indices = scene.getJointPositionIndices([BASE_JOINT])
        self.base_q = np.flatnonzero(np.isin(self.q_indices, self.base_indices))
        self.seed_joint_names = [
            name for name in group_info.joint_names if name != BASE_JOINT
        ]

        self.ik_options = IterativeSolveOptions()
        self.ik_options.max_iters = 200
        self.ik_options.max_time = 0.5
        self.ik_options.max_restarts = 0
        self.ik_options.max_stagnant_iters = 20
        self.ik_options.check_collisions = True

        self.oink = Oink(scene, group_name)
        lower, upper = scene.getPositionLimitVectors(group_name, collapsed=True)
        margin = JOINT_LIMIT_MARGIN * (upper - lower)
        finite = np.isfinite(margin)  # Leave unlimited joints unlimited.
        lower[finite] += margin[finite]
        upper[finite] -= margin[finite]
        self.position_limit = PositionLimit(
            self.oink,
            joint_lower_limits=lower,
            joint_upper_limits=upper,
            max_pullback=0.2,
        )

    def solve_ik(
        self,
        world_T_tcps: list[np.ndarray],
        left_T_right: np.ndarray | None,
        q_seed: np.ndarray,
    ) -> np.ndarray | None:
        """
        Solves for the hands at world poses, left first, staying near the seed.
        """
        goal_tasks = []
        for frame, world_T_tcp in zip(GRASP_FRAMES, world_T_tcps):
            goal = CartesianConfiguration("", frame, world_T_tcp)  # In the world frame
            goal_tasks.append(FrameTask(self.oink, self.scene, goal))
        constraints = [self.position_limit]
        if left_T_right is not None:
            constraints.append(
                RelativePoseConstraint(
                    self.oink,
                    self.scene,
                    LEFT_GRASP_FRAME,
                    RIGHT_GRASP_FRAME,
                    left_T_right,
                )
            )
        try:
            q = self.oink.solveIterativeIk(
                self.scene.toFullJointPositions(self.group_name, q_seed),
                goal_tasks,
                extra_tasks=[
                    ConfigurationTask(
                        self.oink,
                        q_seed,
                        np.full(self.oink.num_variables, POSTURE_WEIGHT),
                        ConfigurationTaskOptions(priority=2),
                    )
                ],
                constraints=constraints,
                options=self.ik_options,
            )
        except RuntimeError:
            return None
        return q[self.q_indices]

    def solve_grasp(
        self,
        world_T_object: np.ndarray,
        hand_flips: tuple[bool, bool],
        left_T_right: np.ndarray | None,
        q_seed: np.ndarray,
    ) -> Grasp | None:
        """
        Solves for the grasp of the object at a pose and the pre-grasp above it, with the right
        hand held at left_T_right from the left, or given its own goal if that is None.
        """
        object_T_tcps = get_object_T_tcps(hand_flips)
        if left_T_right is not None:
            object_T_tcps = object_T_tcps[:1]
        q_grasp = self.solve_ik(
            [world_T_object @ object_T_tcp for object_T_tcp in object_T_tcps],
            left_T_right,
            q_seed,
        )
        if q_grasp is None:
            return None
        q_above = self.solve_ik(
            [above(world_T_object) @ object_T_tcp for object_T_tcp in object_T_tcps],
            left_T_right,
            q_grasp,
        )
        if q_above is None:
            return None
        return Grasp(hand_flips, q_grasp, q_above)

    def find_grasp(
        self,
        world_T_src: np.ndarray,
        world_T_dst: np.ndarray,
        hand_flips: tuple[bool, bool],
    ) -> tuple[Grasp, Grasp] | None:
        """
        Finds a way of picking the object up from one pose and putting it down at another, with
        the hands holding it the same way at both.

        Each pick is solved from a random seed with the base in front of the object, and its
        place from the pick, with the base moved along with the object.

        Returns a (pick, place) grasp pair.
        """
        q_reference = self.scene.getCurrentJointPositions()
        base_xy = world_T_src[:2, 3] - [BASE_STANDOFF, 0.0]
        q_reference[self.base_indices] = [*base_xy, 1.0, 0.0]
        for _ in range(IK_ATTEMPTS):
            q_seed = self.scene.randomCollisionFreePositions(
                self.seed_joint_names, q_reference
            )
            if q_seed is None:
                continue
            pick = self.solve_grasp(
                world_T_src, hand_flips, None, q_seed[self.q_indices]
            )
            if pick is None:
                continue
            q_pick = self.scene.toFullJointPositions(self.group_name, pick.q_grasp)
            left_T_right = self.scene.forwardKinematics(
                q_pick, RIGHT_GRASP_FRAME, LEFT_GRASP_FRAME
            )
            q_seed = pick.q_grasp.copy()
            q_seed[self.base_q[:2]] += world_T_dst[:2, 3] - world_T_src[:2, 3]
            place = self.solve_grasp(world_T_dst, hand_flips, left_T_right, q_seed)
            if place is not None:
                return pick, place
        return None

    def find_grasps(
        self, world_T_src: np.ndarray, world_T_dst: np.ndarray
    ) -> list[tuple[Grasp, Grasp]]:
        """Finds the ways of carrying the object between two poses, as (pick, place) pairs."""
        t_start = time.time()
        grasp_pairs = []
        for hand_flips in HAND_FLIPS:
            grasp_pair = self.find_grasp(world_T_src, world_T_dst, hand_flips)
            if grasp_pair is not None:
                grasp_pairs.append(grasp_pair)
        print(
            f"  Found {len(grasp_pairs)} ways of grasping the object in "
            f"{time.time() - t_start:.3f} s"
        )
        return grasp_pairs


class MotionPlanner:
    """Plans, animates, and executes the robot's motions, with and without the object."""

    def __init__(
        self,
        scene: Scene,
        viz: ViserVisualizer,
        group_name: str,
        max_planning_time: float,
        carry_planning_time: float,
        rng_seed: int,
    ):
        self.scene = scene
        self.viz = viz
        self.group_name = group_name
        group_info = scene.getJointGroupInfo(group_name)
        self.q_indices = group_info.q_indices
        self.joint_names = group_info.joint_names

        self.rrt = RRT(
            scene,
            RRTOptions(
                group_name=group_name,
                collision_check_step_size=COLLISION_CHECK_STEP_SIZE,
                max_planning_time=max_planning_time,
                rrt_connect=True,
            ),
        )
        self.rrt.setRngSeed(rng_seed)

        # Constrained paths can't be shortcut, since the shortcutter ignores the constraints. The
        # trajectory's corner blends smooth them out instead.
        self.carry_rrt = RRT(
            scene,
            RRTOptions(
                group_name=group_name,
                max_nodes=40000,
                max_connection_distance=0.5,
                collision_check_step_size=COLLISION_CHECK_STEP_SIZE,
                max_planning_time=carry_planning_time,
                rrt_connect=True,
                constraint_projection=ConstraintProjectorOptions(path_step_size=0.1),
            ),
        )
        self.carry_rrt.setRngSeed(rng_seed)

        self.shortcutter = PathShortcutter(
            scene,
            PathShortcuttingOptions(
                group_name=group_name, max_step_size=COLLISION_CHECK_STEP_SIZE
            ),
        )
        self.toppra = PathParameterizerTOPPRA(scene, group_name)

    def to_full(self, q: np.ndarray) -> np.ndarray:
        return self.scene.toFullJointPositions(self.group_name, q)

    def animate(self, path: JointPath, traj: JointTrajectory):
        visualizePath(
            self.viz, self.scene, path, GRASP_FRAMES, COLLISION_CHECK_STEP_SIZE
        )
        for q in traj.positions:
            self.viz.display(self.to_full(q))
            time.sleep(TRAJ_DT)

    def report_carry(
        self,
        name: str,
        traj: JointTrajectory,
        constraint: PoseConstraint,
        t_start: float,
    ):
        """Reports the planning time and how far the hands drifted from the grasp."""
        errors = np.array(
            [constraint.computeError(self.to_full(q)) for q in traj.positions]
        )
        print(
            f"  {name}: planned in {time.time() - t_start:.3f} s, "
            f"max hand-to-hand error {1000.0 * np.abs(errors[:, :3]).max():.2f} mm, "
            f"{np.degrees(np.abs(errors[:, 3:]).max()):.3f} deg"
        )

    def move_to(
        self, q_goals: np.ndarray | list[np.ndarray], straight: bool = False
    ) -> int:
        """
        Plans from the current configuration to a goal, straight or with RRT, and animates it.

        Returns the index of the goal reached.
        """
        t_start = time.time()
        q_start = self.scene.getCurrentJointPositions()[self.q_indices]
        if not isinstance(q_goals, list):
            q_goals = [q_goals]
        goal_index = 0
        q_goal = q_goals[0]
        if straight:
            if len(q_goals) != 1:
                raise ValueError("Straight-line motion takes a single goal.")
            if hasCollisionsAlongPath(
                self.scene,
                self.to_full(q_start),
                self.to_full(q_goal),
                STRAIGHT_CHECK_STEP_SIZE,
            ):
                raise RuntimeError("Straight-line motion is in collision.")
            path = JointPath()
            path.joint_names = self.joint_names
            path.positions = [q_start, q_goal]
        else:
            result = self.rrt.planToAny(
                JointConfiguration(self.joint_names, q_start),
                [JointConfiguration(self.joint_names, q) for q in q_goals],
            )
            goal_index = result.goal_index
            q_goal = q_goals[goal_index]
            path = self.shortcutter.shortcut(result.path)

        # Paths with the planar base joint are resampled densely along each edge, and the default
        # Hermite spline would stop at every resampled point. Linear blends don't.
        traj = self.toppra.generate(
            path, TOPPRAOptions(dt=TRAJ_DT, mode=SplineFittingMode.LinearBlend)
        )
        name = "Straight move" if straight else "RRT move"
        print(f"  {name}: planned in {time.time() - t_start:.3f} s")
        self.animate(path, traj)
        self.scene.setJointPositions(self.to_full(q_goal))
        return goal_index

    def carry_cartesian(
        self,
        name: str,
        object_waypoints: list[np.ndarray],
        hand_flips: tuple[bool, bool],
        constraint: PoseConstraint,
    ):
        """Carries the object through world poses in straight lines, and animates it."""
        t_start = time.time()
        q_start = self.scene.getCurrentJointPositions()
        world_T_left = self.scene.forwardKinematics(q_start, LEFT_GRASP_FRAME)
        object_T_left, _ = get_object_T_tcps(hand_flips)
        left_targets = [world_T_left] + [
            world_T_object @ object_T_left for world_T_object in object_waypoints
        ]
        path = CartesianPath(["universe"], [LEFT_GRASP_FRAME], [left_targets])
        planner = make_cartesian_planner(self.scene, constraint.tform, q_start)
        start = JointConfiguration()
        start.positions = q_start
        traj = planner.plan(path, start)

        # Express the trajectory in the main joint group, with the torso where it is.
        traj.positions = [
            self.scene.toFullJointPositions(LIFT_GROUP, q)[self.q_indices]
            for q in traj.positions
        ]
        traj.joint_names = self.joint_names
        for q_prev, q_next in zip(traj.positions[:-1], traj.positions[1:]):
            if hasCollisionsAlongPath(
                self.scene,
                self.to_full(q_prev),
                self.to_full(q_next),
                STRAIGHT_CHECK_STEP_SIZE,
            ):
                raise RuntimeError(f"{name} is in collision.")

        self.report_carry(name, traj, constraint, t_start)
        joint_path = JointPath()
        joint_path.joint_names = traj.joint_names
        joint_path.positions = traj.positions[:: max(1, len(traj.positions) // 50)]
        self.animate(joint_path, traj)
        self.scene.setJointPositions(self.to_full(traj.positions[-1]))

    def carry_to(self, q_goals: list[np.ndarray], constraint: PoseConstraint) -> int:
        """
        Plans a motion that keeps both hands on the object with constrained RRT to whichever of
        the goals it reaches first, and animates it. Returns the index of the goal reached.
        """
        t_start = time.time()
        start = JointConfiguration(
            self.joint_names, self.scene.getCurrentJointPositions()[self.q_indices]
        )
        goals = [JointConfiguration(self.joint_names, q) for q in q_goals]
        result = self.carry_rrt.planToAny(start, goals, [constraint])
        path = result.path

        # The constraint only holds at the planned waypoints and on the straight lines between
        # them. Straight segments with small corner blends stay close to it, and unlike splines,
        # they don't slow down through the dense, slightly noisy waypoints the planner returns.
        traj = self.toppra.generate(
            path,
            TOPPRAOptions(
                dt=TRAJ_DT,
                mode=SplineFittingMode.LinearBlend,
                max_blend_deviation=0.005,
            ),
        )
        self.report_carry("Constrained RRT carry", traj, constraint, t_start)
        self.animate(path, traj)
        self.scene.setJointPositions(self.to_full(q_goals[result.goal_index]))
        return result.goal_index


def main(
    max_planning_time: float = 5.0,
    carry_planning_time: float = 2.0,
    host: str = "localhost",
    port: str = "8000",
    rng_seed: int = 1337,
):
    """
    Carry an object with two arms back and forth between two tables.
    """
    model_data = get_model_data()[MODEL_NAME]
    group_name = model_data.default_joint_group
    package_paths = [get_package_share_dir()]

    urdf_xml = xacro.process_file(model_data.urdf_path).toxml()
    srdf_xml = xacro.process_file(model_data.srdf_path).toxml()

    scene = Scene(
        "bimanual_pick_and_place_scene",
        loadUrdfSceneDescriptionFromXml(urdf_xml, package_paths),
    )
    scene.importJointLimitsFromConfig(
        loadJointLimitsConfig(model_data.yaml_config_path)
    )
    scene.importSrdf(srdf_xml)
    q_indices = scene.getJointGroupInfo(group_name).q_indices

    # Build a separate Pinocchio model with mimic joints for viz.
    model = pin.buildModelFromXML(urdf_xml, mimic=True)
    collision_model = pin.buildGeomFromUrdfString(
        model, urdf_xml, pin.GeometryType.COLLISION, package_dirs=package_paths
    )
    visual_model = pin.buildGeomFromUrdfString(
        model, urdf_xml, pin.GeometryType.VISUAL, package_dirs=package_paths
    )

    # Obstacles are added to the scene and the visualization models separately
    grey, brown = [0.5, 0.5, 0.5, 0.5], [0.6, 0.4, 0.2, 0.8]
    obstacles = [
        ObstacleConfig.box(
            "ground_plane",
            (4.0, 4.0, 0.2),
            (0, 0, -0.1),
            grey,
            BASE_LINKS,
        ),
        *[
            ObstacleConfig.box(
                f"table_{i}",
                (*size_xy, height),
                (*xy, height / 2.0),
                brown,
                ["ground_plane"],
            )
            for i, (xy, size_xy, height) in enumerate(
                zip(TABLE_XYS, TABLE_SIZES_XY, TABLE_HEIGHTS)
            )
        ],
        ObstacleConfig(
            name=OBJECT_NAME,
            geom=coal.Box(*OBJECT_SIZE),
            parent_frame="universe",
            tform=get_object_pose_on_table(0),
            color=np.array([1.0, 0.5, 0.0, 1.0]),
        ),
    ]
    for obstacle in obstacles:
        obstacle.addToScene(scene)
        obstacle.addToPinocchioModels(model, collision_model, visual_model)

    viz = ViserVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=True, loadModel=True, host=host, port=port)

    q_home = np.array(model_data.starting_joint_config)
    scene.setJointPositions(q_home)
    viz.display(q_home)

    scene.setRngSeed(rng_seed)
    grasp_solver = GraspSolver(scene, group_name)
    planner = MotionPlanner(
        scene, viz, group_name, max_planning_time, carry_planning_time, rng_seed
    )

    run_requested = threading.Event()
    run_button = viz.viewer.gui.add_button("Pick and place")
    run_button.on_click(lambda _: run_requested.set())

    # Each run carries the object from one table to the other, then the next run carries it back.
    src, dst = 0, 1
    while True:
        run_requested.wait()
        run_button.disabled = True
        print(f"Carrying the object from table {src} to table {dst}:")

        world_T_src = get_object_pose_on_table(src)
        world_T_dst = get_object_pose_on_table(dst)
        grasp_pairs = grasp_solver.find_grasps(world_T_src, world_T_dst)
        if not grasp_pairs:
            print("  Could not find a way of carrying the object.")
            run_requested.clear()
            run_button.disabled = False
            continue

        # Approach whichever pre-grasp the planner reaches first.
        i = planner.move_to([pick.q_above for pick, _ in grasp_pairs])
        grasp, place = grasp_pairs[i]
        planner.move_to(grasp.q_grasp, straight=True)

        # The object is attached to the left hand, but may touch both hands.
        set_fingers(scene, viz, FINGER_CLOSED)
        attach_object(scene, viz, OBJECT_NAME, LEFT_GRASP_FRAME, HAND_LINKS)
        constraint = make_grasp_constraint(scene, group_name)

        # Lift the object straight up, move it over to the other table around the obstacles,
        # and lower it straight down.
        planner.carry_cartesian(
            "Lift", [above(world_T_src)], grasp.hand_flips, constraint
        )
        planner.carry_to([place.q_above], constraint)
        planner.carry_cartesian("Lower", [world_T_dst], grasp.hand_flips, constraint)

        # Open the grippers before detaching, since closed fingers overlap the object slightly.
        set_fingers(scene, viz, FINGER_OPEN)
        detach_object(scene, viz, OBJECT_NAME)
        # The IK solver checks collisions against a snapshot of the scene, so refresh it.
        grasp_solver.oink.refreshContext()

        planner.move_to(place.q_above, straight=True)
        planner.move_to(q_home[q_indices])

        viz.viewer.scene.remove_by_name("/path")

        src, dst = dst, src
        run_requested.clear()
        run_button.disabled = False


if __name__ == "__main__":
    tyro.cli(main)
