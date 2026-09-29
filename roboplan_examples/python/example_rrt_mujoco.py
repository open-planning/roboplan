#!/usr/bin/env python3

"""
RRT pick and place on a scene built from a MuJoCo (MJCF) model, executed in MuJoCo.

Unlike the other planning examples, which build a :class:`Scene` from a URDF/SRDF pair, this one
constructs the scene directly from an MJCF file using :func:`loadMjcfModel`. The Panda MJCF is
fetched via ``robot_descriptions`` (the mujoco_menagerie collection), so no local model files are
needed.

The robot picks up a tall block on one side of a wall and places it on the other. Once grasped,
the block is attached to the hand in the planning scene, so the RRT plans the transport around the
wall accounting for the carried block. After placing, the block is detached and the robot plans
back home around it. Each segment is time-parameterized with TOPP-RA and executed in the MuJoCo
viewer by driving the robot's position actuators.

Note: MuJoCo's passive viewer must be launched from the main thread on macOS, so run this example
with ``mjpython example_rrt_mujoco.py`` there. On Linux, plain ``python`` works.
"""

import tempfile
import time

import mujoco
import mujoco.viewer
import numpy as np
import tyro
import yaml
from robot_descriptions import panda_mj_description

from roboplan.core import (
    Box,
    CartesianConfiguration,
    JointConfiguration,
    JointPath,
    PathShortcutter,
    PathShortcuttingOptions,
    Scene,
    loadJointLimitsConfig,
    loadMjcfModel,
)
from roboplan.rrt import RRT, RRTOptions
from roboplan.simple_ik import SimpleIk, SimpleIkOptions
from roboplan.toppra import PathParameterizerTOPPRA, SplineFittingMode, TOPPRAOptions

EE_FRAME = "hand"
FINGER_FRAMES = ["left_finger", "right_finger"]

# MJCF models define no velocity or acceleration limits, so these are applied to every arm joint.
MAX_JOINT_VELOCITY = 1.0  # rad/s
MAX_JOINT_ACCELERATION = 2.0  # rad/s^2

# Size of the (square) ground plane, in meters.
FLOOR_SIZE = 4.0
FLOOR_THICKNESS = 0.1

# A vertical wall standing on the floor beside the arm's home pose, parallel to the x axis (meters).
WALL_CENTER = [0.675, 0.2, 0.4]
WALL_SIZE = [0.65, 0.04, 0.8]
WALL_RGBA = [0.8, 0.45, 0.3, 0.6]
# MuJoCo collides with the convex hulls of the robot's meshes, which are larger than the meshes
# the planner checks, so the wall is padded in the planning scene to keep plans clear of it.
# It also can be helpful to compensate for slight tracking lag when following planned paths.
WALL_PADDING = 0.01

# A tall block picked up on one side of the wall and placed on the other (meters).
BLOCK_SIZE = [0.04, 0.04, 0.2]
BLOCK_MASS = 0.05  # kg
BLOCK_RGBA = [0.2, 0.4, 0.8, 1.0]
PICK_XY = [0.5, -0.1]
PLACE_XY = [0.4, 0.5]

# The block is grasped from above this far below its top, with the fingertip pads this far along
# the hand's z axis. Approach and retreat moves go straight up and down by APPROACH_HEIGHT.
GRASP_DEPTH = 0.02
HAND_TO_FINGERTIPS = 0.103
APPROACH_HEIGHT = 0.1

# Gripper actuator commands, and how long to wait for the gripper to open or close (seconds).
GRIPPER_OPEN = 255.0
GRIPPER_CLOSED = 0.0
GRIPPER_WAIT = 1.0


def _add_floor_to_scene(scene: Scene, base_link: str) -> None:
    """Adds a ground plane at z=0 to the planning scene so the robot plans above a floor."""
    tform = np.eye(4)
    tform[2, 3] = -FLOOR_THICKNESS / 2.0  # Sink the box so its top face sits at z=0.
    scene.addBoxGeometry(
        "floor",
        "universe",
        Box(FLOOR_SIZE, FLOOR_SIZE, FLOOR_THICKNESS),
        tform,
        np.array([0.55, 0.55, 0.6, 1.0]),
    )
    # The base rests on the floor, so that contact should not count as a collision.
    scene.setCollisions("floor", base_link, False)


def _add_wall_to_scene(scene: Scene) -> None:
    """Adds a vertical wall to the planning scene for the robot to plan around."""
    tform = np.eye(4)
    tform[:3, 3] = WALL_CENTER
    scene.addBoxGeometry(
        "wall",
        "universe",
        Box(*[dim + 2.0 * WALL_PADDING for dim in WALL_SIZE]),
        tform,
        np.array(WALL_RGBA),
    )
    # The wall stands on the floor, so that contact should not count as a collision.
    scene.setCollisions("floor", "wall", False)


def _add_block_to_scene(scene: Scene) -> None:
    """Adds the block to the planning scene, standing on the floor at the pick location."""
    tform = np.eye(4)
    tform[:3, 3] = [*PICK_XY, BLOCK_SIZE[2] / 2.0]
    scene.addBoxGeometry(
        "block", "universe", Box(*BLOCK_SIZE), tform, np.array(BLOCK_RGBA)
    )
    # The block rests on the floor, so that contact should not count as a collision.
    scene.setCollisions("floor", "block", False)


def _build_mujoco_model(mjcf_path: str) -> mujoco.MjModel:
    """Loads the MJCF into MuJoCo and adds a matching ground plane at z=0, wall, and block."""
    spec = mujoco.MjSpec.from_file(str(mjcf_path))
    if not any(geom.name == "floor" for geom in spec.worldbody.geoms):
        floor = spec.worldbody.add_geom()
        floor.name = "floor"
        floor.type = mujoco.mjtGeom.mjGEOM_PLANE
        floor.size = [0.0, 0.0, 0.05]  # An infinite plane with 0.05 m grid lines.
        floor.pos = [0.0, 0.0, 0.0]
        floor.rgba = [0.55, 0.55, 0.6, 1.0]
    wall = spec.worldbody.add_geom()
    wall.name = "wall"
    wall.type = mujoco.mjtGeom.mjGEOM_BOX
    wall.size = [dim / 2.0 for dim in WALL_SIZE]
    wall.pos = WALL_CENTER
    wall.rgba = WALL_RGBA
    block = spec.worldbody.add_body()
    block.name = "block"
    block.add_freejoint()
    block_geom = block.add_geom()
    block_geom.type = mujoco.mjtGeom.mjGEOM_BOX
    block_geom.size = [dim / 2.0 for dim in BLOCK_SIZE]
    block_geom.mass = BLOCK_MASS
    block_geom.rgba = BLOCK_RGBA
    # The gripper squeezes gently, so the block needs high friction to stay in the grasp,
    # including torsional friction (condim 4) to keep it from pivoting about the fingertips.
    block_geom.friction = [1.5, 0.1, 0.0001]
    block_geom.condim = 4
    # Stiff, elliptic friction cones keep the block from creeping out of the fingers.
    spec.option.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
    spec.option.impratio = 10.0
    # Start the block standing on the floor at the pick location.
    home = spec.key("home")
    home.qpos = [*home.qpos, *PICK_XY, BLOCK_SIZE[2] / 2.0, 1.0, 0.0, 0.0, 0.0]
    return spec.compile()


def _home_configuration(scene: Scene, mj_model: mujoco.MjModel) -> np.ndarray:
    """Returns the MJCF's ``home`` keyframe as a full joint configuration for the scene."""
    key = mj_model.key("home")
    return np.array(
        [
            key.qpos[int(mj_model.joint(name).qposadr[0])]
            for name in scene.getJointNames()
        ]
    )


def _set_joint_limits(scene: Scene, joint_names: list[str]) -> None:
    """Applies finite velocity and acceleration limits, which TOPP-RA needs."""
    limits = {
        "joint_limits": {
            name: {
                "max_velocity": [MAX_JOINT_VELOCITY],
                "max_acceleration": [MAX_JOINT_ACCELERATION],
            }
            for name in joint_names
        }
    }
    with tempfile.NamedTemporaryFile("w", suffix=".yaml") as config_file:
        yaml.safe_dump(limits, config_file)
        config_file.flush()
        scene.importJointLimitsFromConfig(loadJointLimitsConfig(config_file.name))


def _draw_trace(viewer: mujoco.viewer.Handle, points: np.ndarray) -> None:
    """Draws a polyline through the points in the viewer as a chain of thin green capsules."""
    scn = viewer.user_scn
    capsule = mujoco.mjtGeom.mjGEOM_CAPSULE
    rgba = np.array([0.1, 0.8, 0.2, 1.0], dtype=np.float32)
    with viewer.lock():
        for geom, start, end in zip(scn.geoms, points[:-1], points[1:]):
            mujoco.mjv_initGeom(
                geom, capsule, np.zeros(3), np.zeros(3), np.zeros(9), rgba
            )
            mujoco.mjv_connector(geom, capsule, 0.004, start, end)
        scn.ngeom = min(len(points) - 1, scn.maxgeom)


def main(
    seed: int = 0,
    max_connection_distance: float = 2.0,
    collision_check_step_size: float = 0.05,
    goal_biasing_probability: float = 0.15,
    max_nodes: int = 5000,
    max_planning_time: float = 5.0,
    rrt_connect: bool = True,
    include_shortcutting: bool = True,
    max_shortcutting_iters: int = 100,
    playback_speed: float = 1.0,
    loop: bool = True,
):
    """
    Plan an RRT pick and place of a tall block on an MJCF-derived scene and execute it in MuJoCo.

    Parameters:
        seed: Seed for the IK solver and the RRT.
        max_connection_distance: Maximum connection distance between two search nodes.
        collision_check_step_size: Configuration-space step size for collision checking along edges.
        goal_biasing_probability: Weighting of the goal node during random sampling.
        max_nodes: The maximum number of nodes to add to the search tree.
        max_planning_time: The maximum time (in seconds) to search for a path.
        rrt_connect: Whether or not to use RRT-Connect.
        include_shortcutting: Whether or not to include path shortcutting for found paths.
        max_shortcutting_iters: The maximum number of path shortcutting iterations.
        playback_speed: Real-time multiplier for executing the trajectory in the viewer.
        loop: Whether to keep resetting and replaying the trajectory until the viewer is closed.
    """
    # Fetch the MJCF from the mujoco_menagerie via robot_descriptions (downloaded and cached
    # on first use), then build both a RoboPlan scene and a MuJoCo model from the same file.
    mjcf_path = panda_mj_description.MJCF_PATH
    print(f"Loading MJCF: {mjcf_path}")

    scene = Scene("panda", loadMjcfModel(mjcf_path))
    scene.allowAdjacentLinkCollisions()
    mj_model = _build_mujoco_model(mjcf_path)
    mj_data = mujoco.MjData(mj_model)

    base_link = scene.getJointGroupInfo("").link_names[0]
    _add_floor_to_scene(scene, base_link)
    _add_wall_to_scene(scene)
    _add_block_to_scene(scene)

    # Map each arm joint to its MuJoCo position actuator. Only joint-transmission actuators are
    # driven by the planner; the tendon-driven gripper is commanded open and closed directly.
    arm_actuator = {}
    gripper_ctrl = None
    for i in range(mj_model.nu):
        actuator = mj_model.actuator(i)
        if int(actuator.trntype[0]) == mujoco.mjtTrn.mjTRN_JOINT:
            arm_actuator[mj_model.joint(int(actuator.trnid[0])).name] = i
        else:
            gripper_ctrl = i

    # MJCF models have no SRDF, so define a group of just the driven arm joints to plan for.
    # The finger joints stay open in the planning scene.
    group_name = "arm"
    joint_names = scene.getJointNames()
    arm_joints = [name for name in joint_names if name in arm_actuator]
    scene.addGroup(group_name, arm_joints)
    _set_joint_limits(scene, arm_joints)
    q_indices = np.asarray(scene.getJointGroupInfo(group_name).q_indices)

    scene.setRngSeed(seed)
    q_home = _home_configuration(scene, mj_model)
    scene.setJointPositions(q_home)

    # Solve IK for top-down grasps above and at the pick and place locations, keeping the hand's
    # home orientation. Each lower pose is seeded from the one above it, so the straight up and
    # down moves between them stay short.
    ik = SimpleIk(
        scene, SimpleIkOptions(group_name=group_name, max_time=0.1, max_restarts=5)
    )
    ik.setRngSeed(seed)
    hand_rotation = scene.forwardKinematics(q_home, EE_FRAME)[:3, :3]
    grasp_height = BLOCK_SIZE[2] - GRASP_DEPTH + HAND_TO_FINGERTIPS

    def solve_ik(xy: list[float], height: float, seed_q: np.ndarray) -> np.ndarray:
        goal = CartesianConfiguration()
        goal.tip_frame = EE_FRAME
        goal.tform = np.eye(4)
        goal.tform[:3, :3] = hand_rotation
        goal.tform[:3, 3] = [*xy, height]
        start = JointConfiguration()
        start.positions = seed_q
        solution = JointConfiguration()
        if not ik.solveIk(goal, start, solution):
            raise SystemExit(f"Could not solve IK for a grasp at {xy}.")
        return solution.positions

    q_pregrasp = solve_ik(PICK_XY, grasp_height + APPROACH_HEIGHT, q_home[q_indices])
    q_grasp = solve_ik(PICK_XY, grasp_height, q_pregrasp)
    q_preplace = solve_ik(PLACE_XY, grasp_height + APPROACH_HEIGHT, q_home[q_indices])
    q_place = solve_ik(PLACE_XY, grasp_height, q_preplace)

    # The planners snapshot the scene on every call, so they see the block once it is attached.
    rrt = RRT(
        scene,
        RRTOptions(
            group_name=group_name,
            max_nodes=max_nodes,
            max_connection_distance=max_connection_distance,
            collision_check_step_size=collision_check_step_size,
            goal_biasing_probability=goal_biasing_probability,
            max_planning_time=max_planning_time,
            rrt_connect=rrt_connect,
        ),
    )
    rrt.setRngSeed(seed)
    shortcutter = PathShortcutter(
        scene,
        PathShortcuttingOptions(
            group_name=group_name,
            max_step_size=collision_check_step_size,
            max_iters=max_shortcutting_iters,
        ),
    )

    def rrt_path(q_start: np.ndarray, q_goal: np.ndarray) -> JointPath:
        start = JointConfiguration()
        start.positions = q_start
        goal = JointConfiguration()
        goal.positions = q_goal
        t_start = time.time()
        path = rrt.plan(start, goal)
        print(
            f"  Found a path with {len(path.positions)} waypoints in {time.time() - t_start:.3f} s"
        )
        if include_shortcutting:
            path = shortcutter.shortcut(path)
            print(f"  Shortcut the path to {len(path.positions)} waypoints")
        return path

    def straight_path(q_start: np.ndarray, q_goal: np.ndarray) -> JointPath:
        path = JointPath()
        path.joint_names = arm_joints
        path.positions = [q_start, q_goal]
        return path

    # Time-parameterize each path into a trajectory sampled at the physics timestep, so playback
    # takes exactly one MuJoCo step per sample, and interleave the gripper commands. TOPP-RA
    # collision checks the path, so each segment is parameterized with the scene as planned.
    toppra = PathParameterizerTOPPRA(scene, group_name)
    toppra_options = TOPPRAOptions(
        dt=mj_model.opt.timestep, mode=SplineFittingMode.Adaptive
    )
    wait_steps = round(GRIPPER_WAIT / mj_model.opt.timestep)
    samples = []  # (arm command, gripper command) at each physics step

    def add_motion(path: JointPath, gripper: float) -> None:
        traj = toppra.generate(path, toppra_options)
        samples.extend((q, gripper) for q in traj.positions)

    def add_wait(q: np.ndarray, gripper: float) -> None:
        samples.extend([(q, gripper)] * wait_steps)

    print("Planning to the block...")
    add_motion(rrt_path(q_home[q_indices], q_pregrasp), GRIPPER_OPEN)
    add_motion(straight_path(q_pregrasp, q_grasp), GRIPPER_OPEN)
    add_wait(q_grasp, GRIPPER_CLOSED)

    # Attach the block to the hand at the grasp, ignoring its contact with the closed fingers.
    # The block now moves with the hand, so the transport is planned around it.
    scene.setJointPositions(scene.toFullJointPositions(group_name, q_grasp))
    scene.attachObject("block", EE_FRAME, FINGER_FRAMES)
    add_motion(straight_path(q_grasp, q_pregrasp), GRIPPER_CLOSED)
    # Once lifted, the carried block must also clear the floor.
    scene.setCollisions("floor", "block", True)
    print("Planning the transport with the attached block...")
    add_motion(rrt_path(q_pregrasp, q_preplace), GRIPPER_CLOSED)
    scene.setCollisions("floor", "block", False)
    add_motion(straight_path(q_preplace, q_place), GRIPPER_CLOSED)
    add_wait(q_place, GRIPPER_OPEN)

    # Release the block and retract. Once detached, the block stays where it was placed, and the
    # unburdened arm can plan home closer around the wall.
    scene.setJointPositions(scene.toFullJointPositions(group_name, q_place))
    scene.detachObject("block")
    add_motion(straight_path(q_place, q_preplace), GRIPPER_OPEN)
    print("Planning back home around the placed block...")
    add_motion(rrt_path(q_preplace, q_home[q_indices]), GRIPPER_OPEN)
    add_wait(q_home[q_indices], GRIPPER_OPEN)
    print(
        f"Generated a {len(samples) * mj_model.opt.timestep:.2f} s trajectory with {len(samples)} samples"
    )

    # The end effector's path along every 10th sample, drawn in the viewer below.
    trace = np.array(
        [
            scene.forwardKinematics(
                scene.toFullJointPositions(group_name, q), EE_FRAME
            )[:3, 3]
            for q, _ in samples[::10]
        ]
    )

    arm_ctrl = [arm_actuator[name] for name in arm_joints]
    # Execute the trajectory in the viewer by driving the position actuators, pacing playback to
    # wall-clock time. The robot physically tracks the planned trajectory under MuJoCo dynamics.
    print("Executing in MuJoCo (close the viewer window to exit)...")
    with mujoco.viewer.launch_passive(
        mj_model, mj_data, show_left_ui=False, show_right_ui=False
    ) as viewer:
        _draw_trace(viewer, trace)
        while viewer.is_running():
            mujoco.mj_resetDataKeyframe(mj_model, mj_data, mj_model.key("home").id)
            mujoco.mj_forward(mj_model, mj_data)
            for q, gripper in samples:
                if not viewer.is_running():
                    break
                step_start = time.perf_counter()
                mj_data.ctrl[arm_ctrl] = q
                mj_data.ctrl[gripper_ctrl] = gripper
                mujoco.mj_step(mj_model, mj_data)
                viewer.sync()
                if playback_speed > 0:
                    step_time = mj_model.opt.timestep / playback_speed
                    time.sleep(max(0.0, step_time - (time.perf_counter() - step_start)))

            if not loop:
                # Keep the window responsive until the user closes it.
                while viewer.is_running():
                    viewer.sync()
                    time.sleep(0.01)


if __name__ == "__main__":
    tyro.cli(main)
