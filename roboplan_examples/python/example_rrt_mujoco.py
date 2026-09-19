#!/usr/bin/env python3

"""
Point-to-point RRT planning on a scene built from a MuJoCo (MJCF) model, executed in MuJoCo.

Unlike the other planning examples, which build a :class:`Scene` from a URDF/SRDF pair, this one
constructs the scene directly from an MJCF file using :func:`loadMjcfModel`. The MJCF is fetched
via ``robot_descriptions`` (the mujoco_menagerie collection), so no local model files are needed.

The same MJCF is loaded into MuJoCo, an RRT plan is computed between two joint configurations on
either side of a wall, and the resulting path is (optionally shortcut and) time-parameterized with
TOPP-RA into a trajectory. The trajectory is visualized and executed in the MuJoCo viewer by
driving the robot's position actuators.

Note: MuJoCo's passive viewer must be launched from the main thread on macOS, so run this example
with ``mjpython example_rrt_mujoco.py`` there. On Linux, plain ``python`` works.
"""

import importlib
import tempfile
import time

import mujoco
import mujoco.viewer
import numpy as np
import tyro
import yaml

from roboplan.core import (
    Box,
    JointConfiguration,
    PathShortcutter,
    PathShortcuttingOptions,
    Scene,
    loadJointLimitsConfig,
    loadMjcfModel,
)
from roboplan.rrt import RRT, RRTOptions
from roboplan.toppra import PathParameterizerTOPPRA, SplineFittingMode, TOPPRAOptions

# Friendly names mapped to their ``robot_descriptions`` MJCF module and end-effector frame. These
# are redundant (7-DOF) arms, which have the extra freedom needed to plan around the floor to most
# reachable poses.
ROBOT_DESCRIPTIONS = {
    "panda": ("panda_mj_description", "hand"),
    "iiwa14": ("iiwa14_mj_description", "link7"),
}

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

# Goals are sampled beyond this x and y (meters), on the far side of the wall from the start.
GOAL_MIN_XY = (0.45, 0.32)


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
        "wall", "universe", Box(*WALL_SIZE), tform, np.array(WALL_RGBA)
    )
    # The wall stands on the floor, so that contact should not count as a collision.
    scene.setCollisions("floor", "wall", False)


def _build_mujoco_model(mjcf_path: str) -> mujoco.MjModel:
    """Loads the MJCF into MuJoCo and adds a matching ground plane at z=0 and wall."""
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
    return spec.compile()


def _home_configuration(
    scene: Scene, mj_model: mujoco.MjModel
) -> tuple[np.ndarray, np.ndarray]:
    """Returns the full joint configuration to start from (in the scene's joint order) and the
    matching actuator commands.

    Uses the MJCF's ``home`` keyframe when present, otherwise falls back to zeros.
    """
    joint_names = scene.getJointNames()
    q = np.zeros(len(joint_names))
    ctrl = np.zeros(mj_model.nu)
    key_id = mujoco.mj_name2id(mj_model, mujoco.mjtObj.mjOBJ_KEY, "home")
    if key_id >= 0:
        key = mj_model.key(key_id)
        ctrl[:] = key.ctrl
        for idx, name in enumerate(joint_names):
            q[idx] = key.qpos[int(mj_model.joint(name).qposadr[0])]
    return q, ctrl


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
        scn.ngeom = len(points) - 1


def _sample_reachable_goal(
    scene: Scene,
    ee_frame: str,
    min_height: float,
    min_reach: float,
    max_reach: float,
    min_xy: tuple[float, float],
    max_samples: int = 5000,
) -> np.ndarray | None:
    """Samples a collision-free goal whose end effector reaches out to a natural, tidy pose.

    Filtering the goal by end-effector placement keeps the demo looking sensible: the arm reaches
    out and up rather than, say, folding its tool underneath itself.
    """
    for _ in range(max_samples):
        # Check the end-effector placement first, as it is much cheaper than collision checking.
        q = scene.randomPositions()
        position = scene.forwardKinematics(q, ee_frame)[:3, 3]
        reach = np.hypot(position[0], position[1])
        if (
            position[2] >= min_height
            and min_reach <= reach <= max_reach
            and np.all(position[:2] >= min_xy)
            and not scene.hasCollisions(q)
        ):
            return q
    return None


def main(
    robot: str = "panda",
    seed: int = 0,
    max_connection_distance: float = 2.0,
    collision_check_step_size: float = 0.05,
    goal_biasing_probability: float = 0.15,
    max_nodes: int = 5000,
    max_planning_time: float = 5.0,
    include_shortcutting: bool = False,
    max_shortcutting_iters: int = 100,
    goal_min_height: float = 0.2,
    goal_min_reach: float = 0.35,
    goal_max_reach: float = 0.7,
    playback_speed: float = 1.0,
    loop: bool = True,
):
    """
    Plan an RRT trajectory on an MJCF-derived scene, visualize it, and execute it in MuJoCo.

    Parameters:
        robot: Which robot to load. One of: panda, iiwa14.
        seed: Seed for sampling the goal configuration and for the RRT.
        max_connection_distance: Maximum connection distance between two search nodes.
        collision_check_step_size: Configuration-space step size for collision checking along edges.
        goal_biasing_probability: Weighting of the goal node during random sampling.
        max_nodes: The maximum number of nodes to add to the search tree.
        max_planning_time: The maximum time (in seconds) to search for a path.
        include_shortcutting: Whether or not to include path shortcutting for found paths.
        max_shortcutting_iters: The maximum number of path shortcutting iterations.
        goal_min_height: Minimum end-effector height (meters) for the sampled goal.
        goal_min_reach: Minimum end-effector horizontal reach (meters) for the sampled goal.
        goal_max_reach: Maximum end-effector horizontal reach (meters) for the sampled goal.
        playback_speed: Real-time multiplier for executing the trajectory in the viewer.
        loop: Whether to keep replaying the trajectory until the viewer is closed.
    """
    if robot not in ROBOT_DESCRIPTIONS:
        raise SystemExit(
            f"Unknown robot '{robot}'. Choose one of: {', '.join(ROBOT_DESCRIPTIONS)}"
        )

    # Fetch the MJCF from the mujoco_menagerie via robot_descriptions (downloaded and cached
    # on first use), then build both a RoboPlan scene and a MuJoCo model from the same file.
    module_name, ee_frame = ROBOT_DESCRIPTIONS[robot]
    description = importlib.import_module(f"robot_descriptions.{module_name}")
    mjcf_path = description.MJCF_PATH
    print(f"Loading MJCF: {mjcf_path}")

    scene = Scene(robot, loadMjcfModel(mjcf_path))
    scene.allowAdjacentLinkCollisions()
    mj_model = _build_mujoco_model(mjcf_path)
    mj_data = mujoco.MjData(mj_model)

    joint_names = scene.getJointNames()
    for name in joint_names:
        if scene.getJointInfo(name).num_position_dofs != 1:
            raise SystemExit(
                f"This example only supports single-DOF joints, but '{name}' is multi-DOF."
            )

    base_link = scene.getJointGroupInfo("").link_names[0]
    _add_floor_to_scene(scene, base_link)
    _add_wall_to_scene(scene)

    # Map each arm joint to its MuJoCo position actuator. Only joint-transmission actuators are
    # driven directly; anything else (e.g. the Panda's tendon-driven gripper) is left at its home
    # command, so the gripper simply holds its pose while the arm moves.
    arm_actuator = {}
    for i in range(mj_model.nu):
        actuator = mj_model.actuator(i)
        if int(actuator.trntype[0]) == mujoco.mjtTrn.mjTRN_JOINT:
            arm_actuator[mj_model.joint(int(actuator.trnid[0])).name] = i

    # MJCF models have no SRDF, so define a group of just the driven arm joints to plan for.
    group_name = "arm"
    arm_joints = [name for name in joint_names if name in arm_actuator]
    scene.addGroup(group_name, arm_joints)
    _set_joint_limits(scene, arm_joints)
    q_indices = np.asarray(scene.getJointGroupInfo(group_name).q_indices)

    # Plan from the home configuration to a random, collision-free, nicely-placed goal.
    scene.setRngSeed(seed)
    q_home, home_ctrl = _home_configuration(scene, mj_model)
    scene.setJointPositions(q_home)
    q_goal = _sample_reachable_goal(
        scene, ee_frame, goal_min_height, goal_min_reach, goal_max_reach, GOAL_MIN_XY
    )
    if q_goal is None:
        raise SystemExit(
            "Could not sample a collision-free goal within the requested workspace; "
            "try a different seed or widen the goal reach/height bounds."
        )

    start = JointConfiguration()
    start.positions = q_home[q_indices]
    goal = JointConfiguration()
    goal.positions = q_goal[q_indices]

    options = RRTOptions(
        group_name=group_name,
        max_nodes=max_nodes,
        max_connection_distance=max_connection_distance,
        collision_check_step_size=collision_check_step_size,
        goal_biasing_probability=goal_biasing_probability,
        max_planning_time=max_planning_time,
    )
    rrt = RRT(scene, options)
    rrt.setRngSeed(seed)

    print("Planning...")
    t_start = time.time()
    path = rrt.plan(start, goal)
    print(
        f"Found a path with {len(path.positions)} waypoints in {time.time() - t_start:.3f} s"
    )

    if include_shortcutting:
        shortcutter = PathShortcutter(
            scene,
            PathShortcuttingOptions(
                group_name=group_name,
                max_step_size=collision_check_step_size,
                max_iters=max_shortcutting_iters,
            ),
        )
        t_start = time.time()
        path = shortcutter.shortcut(path)
        print(
            f"Shortcut the path to {len(path.positions)} waypoints in {time.time() - t_start:.3f} s"
        )

    # Time-parameterize the path into a trajectory sampled at the physics timestep, so playback
    # takes exactly one MuJoCo step per sample.
    toppra = PathParameterizerTOPPRA(scene, group_name)
    traj = toppra.generate(
        path, TOPPRAOptions(dt=mj_model.opt.timestep, mode=SplineFittingMode.Adaptive)
    )
    print(
        f"Generated a {traj.times[-1]:.2f} s trajectory with {len(traj.times)} samples"
    )

    # The end effector's path along every 10th trajectory sample, drawn in the viewer below.
    trace = np.array(
        [
            scene.forwardKinematics(
                scene.toFullJointPositions(group_name, q), ee_frame
            )[:3, 3]
            for q in traj.positions[::10]
        ]
    )

    arm_ctrl = [arm_actuator[name] for name in traj.joint_names]

    def reset_to_home() -> None:
        for idx, name in enumerate(joint_names):
            mj_data.qpos[int(mj_model.joint(name).qposadr[0])] = q_home[idx]
        mj_data.qvel[:] = 0.0
        mj_data.ctrl[:] = home_ctrl
        mujoco.mj_forward(mj_model, mj_data)

    # Execute the trajectory in the viewer by driving the position actuators, pacing playback to
    # wall-clock time. The robot physically tracks the planned trajectory under MuJoCo dynamics,
    # then holds the final sample so the servo settles onto the goal.
    settle_time = 1.0
    settle_steps = round(settle_time / mj_model.opt.timestep)
    samples = [*traj.positions, *[traj.positions[-1]] * settle_steps]

    print("Executing in MuJoCo (close the viewer window to exit)...")
    with mujoco.viewer.launch_passive(
        mj_model, mj_data, show_left_ui=False, show_right_ui=False
    ) as viewer:
        _draw_trace(viewer, trace)
        while viewer.is_running():
            reset_to_home()
            for q in samples:
                if not viewer.is_running():
                    break
                step_start = time.perf_counter()
                mj_data.ctrl[arm_ctrl] = q
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
