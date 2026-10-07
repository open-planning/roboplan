#!/usr/bin/env python3

import threading
import time

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

MODEL_NAME = "ur5"

OBJECT_NAME = "object"
GRASP_FRAME = "tool0"

# The object is a box held between the UR5 gripper fingers
OBJECT_SIZE = (0.04, 0.04, 0.2)
TOOL0_T_OBJECT = pin.SE3(np.eye(3), np.array([0.0, 0.0, 0.12])).homogeneous

# The object's cross-section is square and the gripper is symmetric, so it can be grasped (and
# placed) at any quarter turn about its vertical axis.
GRASP_YAWS = [0.0, np.pi / 2, np.pi, 3 * np.pi / 2]

# Two tables with a divider between them
TABLE_SIZE = (0.2, 0.2, 0.2)
TABLE_XYS = [(0.45, -0.35), (0.45, 0.35)]
DIVIDER_SIZE = (0.3, 0.02, 0.4)

APPROACH_DISTANCE = 0.1
COLLISION_CHECK_STEP_SIZE = 0.02
TRAJ_DT = 0.01


def get_object_pose_on_table(table_xy: tuple[float, float]) -> np.ndarray:
    """Returns the world pose of the object resting on a table, gripper-down."""
    z = TABLE_SIZE[2] + OBJECT_SIZE[2] / 2.0 + 0.002
    return pin.SE3(pin.utils.rotate("x", np.pi), np.array([*table_xy, z])).homogeneous


def main(
    max_planning_time: float = 5.0,
    host: str = "localhost",
    port: str = "8000",
    rng_seed: int = 1337,
):
    """
    Carry an object back and forth between two tables.

    Parameters:
        max_planning_time: The maximum time (in seconds) to search for a path.
        host: The host for the ViserVisualizer.
        port: The port for the ViserVisualizer.
        rng_seed: The seed for the IK solver's and RRT planner's random number generators.
    """
    model_data = get_model_data()[MODEL_NAME]
    group_name = model_data.default_joint_group
    package_paths = [get_package_share_dir()]

    urdf_xml = xacro.process_file(model_data.urdf_path).toxml()
    srdf_xml = xacro.process_file(model_data.srdf_path).toxml()

    scene = Scene(
        "pick_and_place_scene",
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

    # Obstacles are added to the scene and the visualization models separately (see above).
    grey, brown = [0.5, 0.5, 0.5, 0.5], [0.6, 0.4, 0.2, 0.8]
    obstacles = [
        ObstacleConfig.box(
            "ground_plane", (1.5, 1.5, 0.2), (0, 0, -0.1), grey, ["base_link"]
        ),
        ObstacleConfig.box(
            "divider",
            DIVIDER_SIZE,
            (0.45, 0.0, DIVIDER_SIZE[2] / 2.0),
            [0.0, 0.0, 1.0, 0.5],
            ["ground_plane"],
        ),
        *[
            ObstacleConfig.box(
                f"table_{i}",
                TABLE_SIZE,
                (*xy, TABLE_SIZE[2] / 2.0),
                brown,
                ["ground_plane"],
            )
            for i, xy in enumerate(TABLE_XYS)
        ],
        ObstacleConfig(
            name=OBJECT_NAME,
            geom=coal.Box(*OBJECT_SIZE),
            parent_frame="universe",
            tform=get_object_pose_on_table(TABLE_XYS[0]),
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

    ik_solver = SimpleIk(
        scene,
        SimpleIkOptions(group_name=group_name, max_iters=200, check_collisions=True),
    )
    ik_solver.setRngSeed(rng_seed)
    world_T_base = scene.forwardKinematics(q_home, model_data.base_link)

    def solve_ik(
        world_T_object: np.ndarray, q_seed: np.ndarray, yaw: float
    ) -> np.ndarray:
        """Solves for grasping the object, turned by a yaw about its vertical axis."""
        goal = CartesianConfiguration()
        goal.base_frame = model_data.base_link
        goal.tip_frame = GRASP_FRAME
        object_T_grasp = pin.SE3(pin.utils.rotate("z", yaw), np.zeros(3)).homogeneous
        goal.tform = (
            np.linalg.inv(world_T_base)
            @ world_T_object
            @ object_T_grasp
            @ np.linalg.inv(TOOL0_T_OBJECT)
        )
        start = JointConfiguration()
        start.positions = q_seed
        solution = JointConfiguration()
        if not ik_solver.solveIk(goal, start, solution):
            raise RuntimeError(f"Could not solve IK for grasp pose:\n{world_T_object}")
        return solution.positions

    # For each table and grasp yaw, solve for the grasp and the pre-grasp above it. The
    # pre-grasp is seeded from the grasp so that the approach is straight. grasps[table] holds
    # a (q_grasp, q_above) pair for every yaw IK could reach.
    grasps = []
    for xy in TABLE_XYS:
        table_grasps = []
        for yaw in GRASP_YAWS:
            world_T_object = get_object_pose_on_table(xy)
            try:
                q_grasp = solve_ik(world_T_object, q_home[q_indices], yaw)
                world_T_object[2, 3] += APPROACH_DISTANCE
                table_grasps.append((q_grasp, solve_ik(world_T_object, q_grasp, yaw)))
            except RuntimeError:
                continue
        if not table_grasps:
            raise RuntimeError(f"Could not solve IK for any grasp at table {xy}.")
        grasps.append(table_grasps)

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
    shortcutter = PathShortcutter(
        scene,
        PathShortcuttingOptions(
            group_name=group_name, max_step_size=COLLISION_CHECK_STEP_SIZE
        ),
    )
    toppra = PathParameterizerTOPPRA(scene, group_name)

    def move_to(q_goals: np.ndarray | list[np.ndarray], straight: bool = False) -> int:
        """
        Plans from the current configuration to a goal, straight or with RRT, and animates it.

        With RRT, several goals may be given, and the planner moves to whichever it reaches
        first. Returns the index of the goal reached.
        """
        q_start = scene.getCurrentJointPositions()[q_indices]
        if not isinstance(q_goals, list):
            q_goals = [q_goals]
        goal_index = 0
        q_goal = q_goals[0]
        if straight:
            if len(q_goals) != 1:
                raise ValueError("Straight-line motion takes a single goal.")
            if hasCollisionsAlongPath(
                scene,
                scene.toFullJointPositions(group_name, q_start),
                scene.toFullJointPositions(group_name, q_goal),
                COLLISION_CHECK_STEP_SIZE / 4.0,
            ):
                raise RuntimeError("Straight-line motion is in collision.")
            path = JointPath()
            path.joint_names = scene.getJointGroupInfo(group_name).joint_names
            path.positions = [q_start, q_goal]
        else:
            start = JointConfiguration()
            start.positions = q_start
            goals = []
            for q in q_goals:
                goal = JointConfiguration()
                goal.positions = q
                goals.append(goal)
            result = rrt.planToAny(start, goals)
            goal_index = result.goal_index
            q_goal = q_goals[goal_index]
            path = shortcutter.shortcut(result.path)

        traj = toppra.generate(path, TOPPRAOptions(dt=TRAJ_DT))
        visualizePath(
            viz,
            scene,
            path,
            [GRASP_FRAME],
            COLLISION_CHECK_STEP_SIZE,
        )
        for q in traj.positions:
            viz.display(scene.toFullJointPositions(group_name, q))
            time.sleep(TRAJ_DT)
        scene.setJointPositions(scene.toFullJointPositions(group_name, q_goal))
        return goal_index

    run_requested = threading.Event()
    run_button = viz.viewer.gui.add_button("Pick and place")
    run_button.on_click(lambda _: run_requested.set())

    # Each run carries the object from one table to the other, then the next run carries it back.
    src, dst = 0, 1
    while True:
        run_requested.wait()
        run_button.disabled = True
        # Approach whichever grasp's pre-grasp the planner reaches first.
        i = move_to([q_above for _, q_above in grasps[src]])
        q_src, q_src_above = grasps[src][i]
        move_to(q_src, straight=True)

        attach_object(scene, viz, OBJECT_NAME, GRASP_FRAME, ["wrist_3_link"])

        # Likewise, place the object at whichever quarter turn the planner reaches first.
        move_to(q_src_above, straight=True)
        i = move_to([q_above for _, q_above in grasps[dst]])
        q_dst, q_dst_above = grasps[dst][i]
        move_to(q_dst, straight=True)

        detach_object(scene, viz, OBJECT_NAME)

        move_to(q_dst_above, straight=True)
        move_to(q_home[q_indices])

        viz.viewer.scene.remove_by_name("/path")

        src, dst = dst, src
        run_requested.clear()
        run_button.disabled = False


if __name__ == "__main__":
    tyro.cli(main)
