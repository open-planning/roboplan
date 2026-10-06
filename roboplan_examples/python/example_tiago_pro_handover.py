#!/usr/bin/env python3

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
from common import (
    ObstacleConfig,
    attach_object,
    detach_object,
    get_model_data,
    reparent_object,
)
from pinocchio.visualize import ViserVisualizer

from roboplan.core import (
    CartesianConfiguration,
    JointConfiguration,
    JointPath,
    PathShortcutter,
    PathShortcuttingOptions,
    Scene,
    hasCollisionsAlongPath,
    loadJointLimitsConfig,
    loadUrdfSceneDescriptionFromXml,
)
from roboplan.example_models import get_package_share_dir
from roboplan.rrt import RRT, RRTOptions
from roboplan.simple_ik import SimpleIk, SimpleIkOptions
from roboplan.toppra import PathParameterizerTOPPRA, TOPPRAOptions
from roboplan.visualization import visualizePath

MODEL_NAME = "tiago_pro"

OBJECT_NAME = "object"
HAND_LINK_NAMES = (
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
BASE_LINKS = [
    "base_link",
    "base_dock_link",
    *[
        f"{link}_{side}_link"
        for link in ("wheel", "suspension")
        for side in ("front_left", "front_right", "rear_left", "rear_right")
    ],
]

# The object is a bar lying along the world y axis. Each hand grasps the end on its own side,
# top-down, with the fingers closing across the bar's width, so the two hands can hold it at
# once to hand it over. The grasping frame's x axis points out of the gripper, and its fingers
# close along its y axis.
OBJECT_SIZE = (0.04, 0.30, 0.08)
GRASP_ROTATION = pin.utils.rotate("y", np.pi / 2) @ pin.utils.rotate("x", -np.pi / 2)
GRASP_OFFSET_Y = 0.11
GRASP_OFFSET_Z = 0.025
FINGER_OPEN = 0.07
FINGER_CLOSED = 0.04

# A table on each side of the robot, each within reach of the arm on that side, and the pose
# in front of the robot where the object is handed over.
TABLE_XYS = {"right": (0.6, -0.3), "left": (0.6, 0.3)}
TABLE_SIZE = (0.3, 0.4, 0.6)
HANDOVER_XYZ = (0.55, 0.0, 0.85)

APPROACH_DISTANCE = 0.1
COLLISION_CHECK_STEP_SIZE = 0.02
TRAJ_DT = 0.01


def get_object_pose_on_table(side: str) -> np.ndarray:
    """Returns the world pose of the object resting on a table."""
    z = TABLE_SIZE[2] + OBJECT_SIZE[2] / 2.0 + 0.002
    return pin.SE3(np.eye(3), np.array([*TABLE_XYS[side], z])).homogeneous


def get_object_T_tcp(side: str, flipped: bool) -> np.ndarray:
    """
    Returns a hand's grasp pose on the object, optionally turned a half turn about its approach
    axis, since the gripper is symmetric.
    """
    offset_y = GRASP_OFFSET_Y if side == "left" else -GRASP_OFFSET_Y
    rotation = GRASP_ROTATION
    if flipped:
        rotation = rotation @ pin.utils.rotate("x", np.pi)
    return pin.SE3(rotation, np.array([0.0, offset_y, GRASP_OFFSET_Z])).homogeneous


def above(world_T_object: np.ndarray) -> np.ndarray:
    """Returns a pose APPROACH_DISTANCE above another."""
    world_T_above = world_T_object.copy()
    world_T_above[2, 3] += APPROACH_DISTANCE
    return world_T_above


@dataclass
class Arm:
    """One of the robot's arms, along with its planners and grasps."""

    side: str
    group_name: str
    grasp_frame: str
    hand_links: list[str]
    q_indices: np.ndarray
    finger_indices: np.ndarray
    ik_solver: SimpleIk
    rrt: RRT
    shortcutter: PathShortcutter
    toppra: PathParameterizerTOPPRA

    # Joint group positions for grasping the object on this arm's table, and at the handover
    # pose, and for the pre-grasps above each of them.
    q_table: np.ndarray | None = None
    q_table_above: np.ndarray | None = None
    q_handover: np.ndarray | None = None
    q_handover_above: np.ndarray | None = None


def main(
    max_planning_time: float = 5.0,
    host: str = "localhost",
    port: str = "8000",
    rng_seed: int = 1337,
):
    """
    Hand an object from one of the TIAGo Pro's arms to the other.

    One arm picks the object up off the table on its side and holds it out in front of the
    robot. The other arm grasps the other end of it, and once the first arm lets go, places it
    on the table on its own side. Each run hands the object back the other way.

    Parameters:
        max_planning_time: The maximum time (in seconds) to search for a path.
        host: The host for the ViserVisualizer.
        port: The port for the ViserVisualizer.
        rng_seed: The seed for the IK solvers' and RRT planners' random number generators.
    """
    model_data = get_model_data()[MODEL_NAME]
    package_paths = [get_package_share_dir()]

    urdf_xml = xacro.process_file(model_data.urdf_path).toxml()
    srdf_xml = xacro.process_file(model_data.srdf_path).toxml()

    scene = Scene(
        "tiago_pro_handover_scene",
        loadUrdfSceneDescriptionFromXml(urdf_xml, package_paths),
    )
    scene.importJointLimitsFromConfig(
        loadJointLimitsConfig(model_data.yaml_config_path)
    )
    scene.importSrdf(srdf_xml)

    # Build a separate Pinocchio model with mimic joints for viz.
    model = pin.buildModelFromXML(urdf_xml, mimic=True)
    collision_model = pin.buildGeomFromUrdfString(
        model, urdf_xml, pin.GeometryType.COLLISION, package_dirs=package_paths
    )
    visual_model = pin.buildGeomFromUrdfString(
        model, urdf_xml, pin.GeometryType.VISUAL, package_dirs=package_paths
    )

    # Obstacles are added to the scene and the visualization models separately (see above).
    obstacles = [
        ObstacleConfig.box(
            "ground_plane",
            (3.0, 3.0, 0.2),
            (0, 0, -0.1),
            [0.5, 0.5, 0.5, 0.5],
            BASE_LINKS,
        ),
        *[
            ObstacleConfig.box(
                f"table_{side}",
                TABLE_SIZE,
                (*xy, TABLE_SIZE[2] / 2.0),
                [0.6, 0.4, 0.2, 0.8],
                ["ground_plane"],
            )
            for side, xy in TABLE_XYS.items()
        ],
        ObstacleConfig(
            name=OBJECT_NAME,
            geom=coal.Box(*OBJECT_SIZE),
            parent_frame="universe",
            tform=get_object_pose_on_table("right"),
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

    def make_arm(side: str) -> Arm:
        group_name = f"arm_{side}"
        ik_solver = SimpleIk(
            scene,
            SimpleIkOptions(
                group_name=group_name,
                max_iters=200,
                max_restarts=10,
                check_collisions=True,
            ),
        )
        ik_solver.setRngSeed(rng_seed)
        rrt = RRT(
            scene,
            RRTOptions(
                group_name=group_name,
                collision_check_step_size=COLLISION_CHECK_STEP_SIZE,
                max_planning_time=max_planning_time,
                rrt_connect=True,
            ),
        )
        rrt.setRngSeed(rng_seed)
        return Arm(
            side=side,
            group_name=group_name,
            grasp_frame=f"gripper_{side}_grasping_link",
            hand_links=[f"gripper_{side}_{link}_link" for link in HAND_LINK_NAMES],
            q_indices=scene.getJointGroupInfo(group_name).q_indices,
            finger_indices=scene.getJointPositionIndices(
                [f"gripper_{side}_finger_joint"]
            ),
            ik_solver=ik_solver,
            rrt=rrt,
            shortcutter=PathShortcutter(
                scene,
                PathShortcuttingOptions(
                    group_name=group_name, max_step_size=COLLISION_CHECK_STEP_SIZE
                ),
            ),
            toppra=PathParameterizerTOPPRA(scene, group_name),
        )

    arms = {side: make_arm(side) for side in ("right", "left")}

    def solve_ik(
        arm: Arm, world_T_tcp: np.ndarray, q_seed: np.ndarray
    ) -> np.ndarray | None:
        goal = CartesianConfiguration()
        goal.base_frame = ""  # The world frame
        goal.tip_frame = arm.grasp_frame
        goal.tform = world_T_tcp
        start = JointConfiguration()
        start.positions = q_seed
        solution = JointConfiguration()
        if not arm.ik_solver.solveIk(goal, start, solution):
            return None
        return solution.positions

    def solve_grasps(arm: Arm):
        """
        Solves for the arm's grasps on its table and at the handover pose, and the pre-grasps
        above them. The pre-grasps are seeded from the grasps so that the approaches are
        straight.

        The arm holds the object the same way in both places, since it can't regrasp it, so
        both ways round of the symmetric gripper are tried until one works in both places.
        """
        world_T_handover = pin.SE3(np.eye(3), np.array(HANDOVER_XYZ)).homogeneous
        world_T_table = get_object_pose_on_table(arm.side)
        for flipped in (False, True):
            object_T_tcp = get_object_T_tcp(arm.side, flipped)
            q_seed = q_home[arm.q_indices]
            q_table = solve_ik(arm, world_T_table @ object_T_tcp, q_seed)
            q_handover = solve_ik(arm, world_T_handover @ object_T_tcp, q_seed)
            if q_table is None or q_handover is None:
                continue
            q_table_above = solve_ik(arm, above(world_T_table) @ object_T_tcp, q_table)
            q_handover_above = solve_ik(
                arm, above(world_T_handover) @ object_T_tcp, q_handover
            )
            if q_table_above is None or q_handover_above is None:
                continue
            arm.q_table, arm.q_table_above = q_table, q_table_above
            arm.q_handover, arm.q_handover_above = q_handover, q_handover_above
            return
        raise RuntimeError(f"Could not solve IK for the {arm.side} arm's grasps.")

    for arm in arms.values():
        solve_grasps(arm)

    def move_to(arm: Arm, q_goal: np.ndarray, straight: bool = False):
        """
        Plans one arm's motion from its current configuration to a goal, straight or with RRT,
        and animates it.
        """
        q_start = scene.getCurrentJointPositions()[arm.q_indices]
        if straight:
            if hasCollisionsAlongPath(
                scene,
                scene.toFullJointPositions(arm.group_name, q_start),
                scene.toFullJointPositions(arm.group_name, q_goal),
                COLLISION_CHECK_STEP_SIZE / 4.0,
            ):
                raise RuntimeError("Straight-line motion is in collision.")
            path = JointPath()
            path.joint_names = scene.getJointGroupInfo(arm.group_name).joint_names
            path.positions = [q_start, q_goal]
        else:
            start = JointConfiguration()
            start.positions = q_start
            goal = JointConfiguration()
            goal.positions = q_goal
            path = arm.shortcutter.shortcut(arm.rrt.plan(start, goal))

        traj = arm.toppra.generate(path, TOPPRAOptions(dt=TRAJ_DT))
        visualizePath(viz, scene, path, [arm.grasp_frame], COLLISION_CHECK_STEP_SIZE)
        for q in traj.positions:
            viz.display(scene.toFullJointPositions(arm.group_name, q))
            time.sleep(TRAJ_DT)
        scene.setJointPositions(scene.toFullJointPositions(arm.group_name, q_goal))

    def set_fingers(arm: Arm, position: float, duration: float = 0.5):
        """Opens or closes one arm's gripper, animating the motion."""
        q_start = scene.getCurrentJointPositions()
        q = q_start.copy()
        for alpha in np.linspace(0.0, 1.0, int(duration / TRAJ_DT)):
            q[arm.finger_indices] = (1.0 - alpha) * q_start[
                arm.finger_indices
            ] + alpha * position
            viz.display(q)
            time.sleep(TRAJ_DT)
        scene.setJointPositions(q)

    run_requested = threading.Event()
    run_button = viz.viewer.gui.add_button("Hand over")
    run_button.on_click(lambda _: run_requested.set())

    # Each run hands the object from one arm to the other, then the next run hands it back.
    giver, taker = arms["right"], arms["left"]
    while True:
        run_requested.wait()
        run_button.disabled = True

        # The giver picks the object up off its table and holds it out at the handover pose.
        move_to(giver, giver.q_table_above)
        move_to(giver, giver.q_table, straight=True)
        set_fingers(giver, FINGER_CLOSED)
        attach_object(scene, viz, OBJECT_NAME, giver.grasp_frame, giver.hand_links)
        move_to(giver, giver.q_table_above, straight=True)
        move_to(giver, giver.q_handover)

        # The taker grasps the other end of the object, then the giver lets go and moves away.
        # The giver opens its gripper before the object changes hands, since closed fingers
        # overlap the object slightly.
        move_to(taker, taker.q_handover_above)
        move_to(taker, taker.q_handover, straight=True)
        set_fingers(taker, FINGER_CLOSED)
        set_fingers(giver, FINGER_OPEN)
        reparent_object(scene, viz, OBJECT_NAME, taker.grasp_frame, taker.hand_links)
        move_to(giver, giver.q_handover_above, straight=True)
        move_to(giver, q_home[giver.q_indices])

        # The taker places the object on its table.
        move_to(taker, taker.q_handover_above, straight=True)
        move_to(taker, taker.q_table_above)
        move_to(taker, taker.q_table, straight=True)
        set_fingers(taker, FINGER_OPEN)
        detach_object(scene, viz, OBJECT_NAME)
        move_to(taker, taker.q_table_above, straight=True)
        move_to(taker, q_home[taker.q_indices])

        viz.viewer.scene.remove_by_name("/path")

        giver, taker = taker, giver
        run_requested.clear()
        run_button.disabled = False


if __name__ == "__main__":
    tyro.cli(main)
