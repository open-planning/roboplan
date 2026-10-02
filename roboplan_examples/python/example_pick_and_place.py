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
from common import ObstacleConfig, get_model_data
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

# Two tables with a divider between them
TABLE_SIZE = (0.2, 0.2, 0.2)
TABLE_XYS = [(0.45, -0.35), (0.45, 0.35)]
DIVIDER_SIZE = (0.3, 0.02, 0.4)

APPROACH_DISTANCE = 0.1
COLLISION_CHECK_STEP_SIZE = 0.02
TRAJ_DT = 0.01


def box_obstacle(name: str, size, xyz, color, disabled_collisions) -> ObstacleConfig:
    return ObstacleConfig(
        name=name,
        geom=coal.Box(*size),
        parent_frame="universe",
        tform=pin.SE3(np.eye(3), np.array(xyz)).homogeneous,
        color=np.array(color),
        disabled_collisions=disabled_collisions,
    )


def get_object_pose_on_table(table_xy: tuple[float, float]) -> np.ndarray:
    """Returns the world pose of the object resting on a table, gripper-down."""
    z = TABLE_SIZE[2] + OBJECT_SIZE[2] / 2.0 + 0.002
    return pin.SE3(pin.utils.rotate("x", np.pi), np.array([*table_xy, z])).homogeneous


def set_viz_parent(viz: ViserVisualizer, name: str, frame_name: str, q: np.ndarray):
    """
    Reparents a geometry in the visualizer's Pinocchio models, keeping its current world pose.

    The scene and the visualizer keep separate Pinocchio models because Pinocchio/coal don't
    have nanobindings yet, so attaching in the scene doesn't move the object in the visualizer.
    """
    pin.forwardKinematics(viz.model, viz.data, q)
    frame_id = viz.model.getFrameId(frame_name)
    joint_id = viz.model.frames[frame_id].parentJoint
    for geom_model in (viz.collision_model, viz.visual_model):
        geom_obj = geom_model.geometryObjects[geom_model.getGeometryId(name)]
        world_T_geom = viz.data.oMi[geom_obj.parentJoint] * geom_obj.placement
        geom_obj.parentFrame = frame_id
        geom_obj.parentJoint = joint_id
        geom_obj.placement = viz.data.oMi[joint_id].actInv(world_T_geom)


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
        box_obstacle(
            "ground_plane", (1.5, 1.5, 0.2), (0, 0, -0.1), grey, ["base_link"]
        ),
        box_obstacle(
            "divider",
            DIVIDER_SIZE,
            (0.45, 0.0, DIVIDER_SIZE[2] / 2.0),
            [0.0, 0.0, 1.0, 0.5],
            ["ground_plane"],
        ),
        *[
            box_obstacle(
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

    def solve_ik(world_T_object: np.ndarray, q_seed: np.ndarray) -> np.ndarray:
        goal = CartesianConfiguration()
        goal.base_frame = model_data.base_link
        goal.tip_frame = GRASP_FRAME
        goal.tform = (
            np.linalg.inv(world_T_base) @ world_T_object @ np.linalg.inv(TOOL0_T_OBJECT)
        )
        start = JointConfiguration()
        start.positions = q_seed
        solution = JointConfiguration()
        if not ik_solver.solveIk(goal, start, solution):
            raise RuntimeError(f"Could not solve IK for grasp pose:\n{world_T_object}")
        return solution.positions

    # For each table, solve for the grasp and the pre-grasp above it. The pre-grasp is seeded
    # from the grasp so that the approach is straight.
    grasps = []
    for xy in TABLE_XYS:
        world_T_object = get_object_pose_on_table(xy)
        q_grasp = solve_ik(world_T_object, q_home[q_indices])
        world_T_object[2, 3] += APPROACH_DISTANCE
        grasps.append((q_grasp, solve_ik(world_T_object, q_grasp)))

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

    def move_to(q_goal: np.ndarray, straight: bool = False):
        """
        Plans from the current configuration to a goal, straight or with RRT, and animates it.
        """
        q_start = scene.getCurrentJointPositions()[q_indices]
        if straight:
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
            goal = JointConfiguration()
            goal.positions = q_goal
            path = shortcutter.shortcut(rrt.plan(start, goal))

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

    run_requested = threading.Event()
    run_button = viz.viewer.gui.add_button("Pick and place")
    run_button.on_click(lambda _: run_requested.set())

    # Each run carries the object from one table to the other, then the next run carries it back.
    src, dst = 0, 1
    while True:
        run_requested.wait()
        run_button.disabled = True
        q_src, q_src_above = grasps[src]
        q_dst, q_dst_above = grasps[dst]

        move_to(q_src_above)
        move_to(q_src, straight=True)

        scene.attachObject(OBJECT_NAME, GRASP_FRAME, ["wrist_3_link"])
        set_viz_parent(viz, OBJECT_NAME, GRASP_FRAME, scene.getCurrentJointPositions())

        move_to(q_src_above, straight=True)
        move_to(q_dst_above)
        move_to(q_dst, straight=True)

        scene.detachObject(OBJECT_NAME)
        set_viz_parent(viz, OBJECT_NAME, "universe", scene.getCurrentJointPositions())

        move_to(q_dst_above, straight=True)
        move_to(q_home[q_indices])

        viz.viewer.scene.remove_by_name("/path")

        src, dst = dst, src
        run_requested.clear()
        run_button.disabled = False


if __name__ == "__main__":
    tyro.cli(main)
