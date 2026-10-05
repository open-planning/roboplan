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

MODEL_NAME = "tiago_pro"
GROUP_NAME = "arm_right"

OBJECT_NAME = "object"
GRASP_FRAME = "gripper_right_grasping_link"
FINGER_JOINT = "gripper_right_finger_joint"
HAND_LINKS = [
    f"gripper_right_{link}_link"
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
BASE_LINKS = [
    "base_link",
    "base_dock_link",
    *[
        f"{link}_{side}_link"
        for link in ("wheel", "suspension")
        for side in ("front_left", "front_right", "rear_left", "rear_right")
    ],
]

# The object is an upright box, grasped top-down near its top. The grasping frame's x axis
# points out of the gripper, so it is turned to point down.
OBJECT_SIZE = (0.04, 0.04, 0.12)
OBJECT_T_TCP = pin.SE3(
    pin.utils.rotate("y", np.pi / 2), np.array([0.0, 0.0, 0.03])
).homogeneous
FINGER_OPEN = 0.07
FINGER_CLOSED = 0.04

# A table in front of the robot, within reach of the right arm.
TABLE_XY = (0.6, -0.2)
TABLE_SIZE = (0.4, 0.4, 0.6)

APPROACH_DISTANCE = 0.1
COLLISION_CHECK_STEP_SIZE = 0.02
TRAJ_DT = 0.01


def get_object_pose_on_table() -> np.ndarray:
    """Returns the world pose of the object resting on the table."""
    z = TABLE_SIZE[2] + OBJECT_SIZE[2] / 2.0 + 0.002
    return pin.SE3(np.eye(3), np.array([*TABLE_XY, z])).homogeneous


def main(
    max_planning_time: float = 5.0,
    host: str = "localhost",
    port: str = "8000",
    rng_seed: int = 1337,
):
    """
    Pick an object up off a table with the TIAGo Pro's right arm, then put it back down.

    Parameters:
        max_planning_time: The maximum time (in seconds) to search for a path.
        host: The host for the ViserVisualizer.
        port: The port for the ViserVisualizer.
        rng_seed: The seed for the IK solver's and RRT planner's random number generators.
    """
    model_data = get_model_data()[MODEL_NAME]
    package_paths = [get_package_share_dir()]

    urdf_xml = xacro.process_file(model_data.urdf_path).toxml()
    srdf_xml = xacro.process_file(model_data.srdf_path).toxml()

    scene = Scene(
        "tiago_pro_pick_scene",
        loadUrdfSceneDescriptionFromXml(urdf_xml, package_paths),
    )
    scene.importJointLimitsFromConfig(
        loadJointLimitsConfig(model_data.yaml_config_path)
    )
    scene.importSrdf(srdf_xml)
    q_indices = scene.getJointGroupInfo(GROUP_NAME).q_indices
    finger_indices = scene.getJointPositionIndices([FINGER_JOINT])

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
        ObstacleConfig.box(
            "table",
            TABLE_SIZE,
            (*TABLE_XY, TABLE_SIZE[2] / 2.0),
            [0.6, 0.4, 0.2, 0.8],
            ["ground_plane"],
        ),
        ObstacleConfig(
            name=OBJECT_NAME,
            geom=coal.Box(*OBJECT_SIZE),
            parent_frame="universe",
            tform=get_object_pose_on_table(),
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
        SimpleIkOptions(
            group_name=GROUP_NAME,
            max_iters=200,
            max_restarts=10,
            check_collisions=True,
        ),
    )
    ik_solver.setRngSeed(rng_seed)

    def solve_ik(world_T_object: np.ndarray, q_seed: np.ndarray) -> np.ndarray:
        goal = CartesianConfiguration()
        goal.base_frame = ""  # The world frame
        goal.tip_frame = GRASP_FRAME
        goal.tform = world_T_object @ OBJECT_T_TCP
        start = JointConfiguration()
        start.positions = q_seed
        solution = JointConfiguration()
        if not ik_solver.solveIk(goal, start, solution):
            raise RuntimeError(f"Could not solve IK for grasp pose:\n{world_T_object}")
        return solution.positions

    # Solve for the grasp and the pre-grasp above it. The pre-grasp is seeded from the grasp so
    # that the approach is straight.
    world_T_object = get_object_pose_on_table()
    q_grasp = solve_ik(world_T_object, q_home[q_indices])
    world_T_above = world_T_object.copy()
    world_T_above[2, 3] += APPROACH_DISTANCE
    q_above = solve_ik(world_T_above, q_grasp)

    rrt = RRT(
        scene,
        RRTOptions(
            group_name=GROUP_NAME,
            collision_check_step_size=COLLISION_CHECK_STEP_SIZE,
            max_planning_time=max_planning_time,
            rrt_connect=True,
        ),
    )
    rrt.setRngSeed(rng_seed)
    shortcutter = PathShortcutter(
        scene,
        PathShortcuttingOptions(
            group_name=GROUP_NAME, max_step_size=COLLISION_CHECK_STEP_SIZE
        ),
    )
    toppra = PathParameterizerTOPPRA(scene, GROUP_NAME)

    def move_to(q_goal: np.ndarray, straight: bool = False):
        """
        Plans from the current configuration to a goal, straight or with RRT, and animates it.
        """
        q_start = scene.getCurrentJointPositions()[q_indices]
        if straight:
            if hasCollisionsAlongPath(
                scene,
                scene.toFullJointPositions(GROUP_NAME, q_start),
                scene.toFullJointPositions(GROUP_NAME, q_goal),
                COLLISION_CHECK_STEP_SIZE / 4.0,
            ):
                raise RuntimeError("Straight-line motion is in collision.")
            path = JointPath()
            path.joint_names = scene.getJointGroupInfo(GROUP_NAME).joint_names
            path.positions = [q_start, q_goal]
        else:
            start = JointConfiguration()
            start.positions = q_start
            goal = JointConfiguration()
            goal.positions = q_goal
            path = shortcutter.shortcut(rrt.plan(start, goal))

        traj = toppra.generate(path, TOPPRAOptions(dt=TRAJ_DT))
        visualizePath(viz, scene, path, [GRASP_FRAME], COLLISION_CHECK_STEP_SIZE)
        for q in traj.positions:
            viz.display(scene.toFullJointPositions(GROUP_NAME, q))
            time.sleep(TRAJ_DT)
        scene.setJointPositions(scene.toFullJointPositions(GROUP_NAME, q_goal))

    def set_fingers(position: float, duration: float = 0.5):
        """Opens or closes the gripper, animating the motion."""
        q_start = scene.getCurrentJointPositions()
        q = q_start.copy()
        for alpha in np.linspace(0.0, 1.0, int(duration / TRAJ_DT)):
            q[finger_indices] = (1.0 - alpha) * q_start[
                finger_indices
            ] + alpha * position
            viz.display(q)
            time.sleep(TRAJ_DT)
        scene.setJointPositions(q)

    run_requested = threading.Event()
    run_button = viz.viewer.gui.add_button("Pick")
    run_button.on_click(lambda _: run_requested.set())

    while True:
        run_requested.wait()
        run_button.disabled = True

        # Approach the object from above, grasp it, lift it straight up, and bring it back to
        # the home position.
        move_to(q_above)
        move_to(q_grasp, straight=True)

        set_fingers(FINGER_CLOSED)
        attach_object(scene, viz, OBJECT_NAME, GRASP_FRAME, HAND_LINKS)

        move_to(q_above, straight=True)
        move_to(q_home[q_indices])

        # Put the object back down where it was, so that the pick can be run again.
        move_to(q_above)
        move_to(q_grasp, straight=True)

        # Open the gripper before detaching, since closed fingers overlap the object slightly.
        set_fingers(FINGER_OPEN)
        detach_object(scene, viz, OBJECT_NAME)

        move_to(q_above, straight=True)
        move_to(q_home[q_indices])

        viz.viewer.scene.remove_by_name("/path")

        run_requested.clear()
        run_button.disabled = False


if __name__ == "__main__":
    tyro.cli(main)
