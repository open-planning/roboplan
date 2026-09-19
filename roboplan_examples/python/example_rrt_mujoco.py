#!/usr/bin/env python3

"""
Point-to-point RRT planning on a scene built from a MuJoCo (MJCF) model, executed in MuJoCo.

Unlike the other planning examples, which build a :class:`Scene` from a URDF/SRDF pair, this one
constructs the scene directly from an MJCF file using :func:`loadMjcfModel`. The MJCF is fetched
via ``robot_descriptions`` (the mujoco_menagerie collection), so no local model files are needed.

The same MJCF is loaded into MuJoCo, an RRT plan is computed between two joint configurations, and
the resulting path is executed in the MuJoCo viewer by driving the robot's position actuators.

Note: MuJoCo's passive viewer must be launched from the main thread on macOS, so run this example
with ``mjpython example_rrt_mujoco.py`` there. On Linux, plain ``python`` works.
"""

import importlib
import time
from itertools import pairwise

import mujoco
import mujoco.viewer
import numpy as np
import tyro

from roboplan.core import Box, JointConfiguration, Scene, loadMjcfModel
from roboplan.rrt import RRT, RRTOptions

# Friendly names mapped to their ``robot_descriptions`` MJCF module. These are redundant (7-DOF)
# arms, which have the extra freedom needed to plan around the floor to most reachable poses.
ROBOT_DESCRIPTIONS = {
    "panda": "panda_mj_description",
    "iiwa14": "iiwa14_mj_description",
}

# Size of the (square) ground plane, in meters.
FLOOR_SIZE = 4.0
FLOOR_THICKNESS = 0.1


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


def _build_mujoco_model(mjcf_path: str) -> mujoco.MjModel:
    """Loads the MJCF into MuJoCo and adds a matching ground plane at z=0."""
    spec = mujoco.MjSpec.from_file(str(mjcf_path))
    if not any(geom.name == "floor" for geom in spec.worldbody.geoms):
        floor = spec.worldbody.add_geom()
        floor.name = "floor"
        floor.type = mujoco.mjtGeom.mjGEOM_PLANE
        floor.size = [0.0, 0.0, 0.05]  # An infinite plane with 0.05 m grid lines.
        floor.pos = [0.0, 0.0, 0.0]
        floor.rgba = [0.55, 0.55, 0.6, 1.0]
    return spec.compile()


def _home_keyframe(mj_model: mujoco.MjModel) -> int | None:
    """Returns the index of the MJCF's ``home`` keyframe, or None if it has none."""
    return next(
        (k for k in range(mj_model.nkey) if mj_model.key(k).name == "home"), None
    )


def _home_configuration(
    scene: Scene, mj_model: mujoco.MjModel, home_key: int | None
) -> np.ndarray:
    """Returns the full joint configuration to start from, in the scene's joint order.

    Uses the MJCF's ``home`` keyframe when present, otherwise falls back to zeros.
    """
    joint_names = scene.getJointNames()
    q = np.zeros(len(joint_names))
    if home_key is not None:
        key_qpos = mj_model.key(home_key).qpos
        for idx, name in enumerate(joint_names):
            q[idx] = key_qpos[int(mj_model.joint(name).qposadr[0])]
    return q


def _end_effector_frame(link_names: list[str]) -> str:
    """Picks a reasonable end-effector frame: the last link that is not a gripper finger."""
    for name in reversed(link_names):
        if "finger" not in name and "gripper" not in name:
            return name
    return link_names[-1]


def _sample_reachable_goal(
    scene: Scene,
    ee_frame: str,
    min_height: float,
    min_reach: float,
    max_reach: float,
    max_samples: int = 500,
) -> np.ndarray | None:
    """Samples a collision-free goal whose end effector reaches out to a natural, tidy pose.

    Filtering the goal by end-effector placement keeps the demo looking sensible: the arm reaches
    out and up rather than, say, folding its tool underneath itself.
    """
    for _ in range(max_samples):
        q = scene.randomCollisionFreePositions()
        if q is None:
            continue
        position = scene.forwardKinematics(q, ee_frame)[:3, 3]
        reach = np.hypot(position[0], position[1])
        if position[2] >= min_height and min_reach <= reach <= max_reach:
            return q
    return None


def main(
    robot: str = "panda",
    seed: int = 0,
    max_connection_distance: float = 2.0,
    collision_check_step_size: float = 0.05,
    goal_biasing_probability: float = 0.15,
    max_nodes: int = 5000,
    max_planning_time: float = 15.0,
    goal_min_height: float = 0.2,
    goal_min_reach: float = 0.35,
    goal_max_reach: float = 0.7,
    playback_speed: float = 1.0,
    loop: bool = True,
):
    """
    Plan a point-to-point RRT path on an MJCF-derived scene and execute it in MuJoCo.

    Parameters:
        robot: Which robot to load. One of: panda, iiwa14.
        seed: Seed for sampling the goal configuration and for the RRT.
        max_connection_distance: Maximum connection distance between two search nodes.
        collision_check_step_size: Configuration-space step size for collision checking along edges.
        goal_biasing_probability: Weighting of the goal node during random sampling.
        max_nodes: The maximum number of nodes to add to the search tree.
        max_planning_time: The maximum time (in seconds) to search for a path.
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
    description = importlib.import_module(
        f"robot_descriptions.{ROBOT_DESCRIPTIONS[robot]}"
    )
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

    link_names = scene.getJointGroupInfo("").link_names
    _add_floor_to_scene(scene, link_names[0])
    ee_frame = _end_effector_frame(link_names)

    # MJCF models have no SRDF, so the scene exposes only the default group containing every joint.
    group_name = ""
    q_indices = np.asarray(scene.getJointGroupInfo(group_name).q_indices)

    # Plan from the home configuration to a random, collision-free, nicely-placed goal.
    scene.setRngSeed(seed)
    home_key = _home_keyframe(mj_model)
    q_home = _home_configuration(scene, mj_model, home_key)
    q_goal = _sample_reachable_goal(
        scene, ee_frame, goal_min_height, goal_min_reach, goal_max_reach
    )
    if q_goal is None:
        raise SystemExit(
            "Could not sample a collision-free goal within the requested workspace; "
            "try a different seed or widen the goal reach/height bounds."
        )

    start = JointConfiguration()
    start.positions = q_home
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

    # Densify the sparse RRT waypoints into a smooth joint-space reference for the controller.
    waypoints = [np.asarray(p) for p in path.positions]
    reference = []
    for q_a, q_b in pairwise(waypoints):
        num_steps = max(2, int(np.max(np.abs(q_b - q_a)) / 0.01))
        for step in range(num_steps):
            reference.append(q_a + (q_b - q_a) * step / num_steps)
    reference.append(waypoints[-1])

    # Map each planned arm joint to its MuJoCo position actuator. Only joint-transmission actuators
    # are driven directly; anything else (e.g. the Panda's tendon-driven gripper) is left at its
    # home command so the gripper simply holds its pose while the arm moves.
    arm_actuator = {}
    for i in range(mj_model.nu):
        actuator = mj_model.actuator(i)
        if int(actuator.trntype[0]) == mujoco.mjtTrn.mjTRN_JOINT:
            arm_actuator[mj_model.joint(int(actuator.trnid[0])).name] = i
    tracked = [
        (arm_actuator[name], idx)
        for idx, name in enumerate(path.joint_names)
        if name in arm_actuator
    ]

    home_ctrl = np.zeros(mj_model.nu)
    if home_key is not None:
        home_ctrl[:] = mj_model.key(home_key).ctrl

    def reset_to_home() -> None:
        for idx, name in enumerate(joint_names):
            mj_data.qpos[int(mj_model.joint(name).qposadr[0])] = q_home[idx]
        mj_data.qvel[:] = 0.0
        mj_data.ctrl[:] = home_ctrl
        mujoco.mj_forward(mj_model, mj_data)

    # Execute the trajectory in the viewer by driving the position actuators, pacing playback to
    # wall-clock time. The robot physically tracks the planned reference under MuJoCo dynamics.
    steps_per_waypoint = 8
    settle_steps = 500
    step_dt = mj_model.opt.timestep * steps_per_waypoint

    print("Executing in MuJoCo (close the viewer window to exit)...")
    with mujoco.viewer.launch_passive(
        mj_model, mj_data, show_left_ui=False, show_right_ui=False
    ) as viewer:
        while viewer.is_running():
            reset_to_home()
            for q in reference:
                if not viewer.is_running():
                    break
                for actuator_index, waypoint_index in tracked:
                    mj_data.ctrl[actuator_index] = q[waypoint_index]
                for _ in range(steps_per_waypoint):
                    mujoco.mj_step(mj_model, mj_data)
                viewer.sync()
                if playback_speed > 0:
                    time.sleep(step_dt / playback_speed)

            # Hold at the goal so the servo settles onto the final waypoint.
            for _ in range(settle_steps):
                if not viewer.is_running():
                    break
                for actuator_index, waypoint_index in tracked:
                    mj_data.ctrl[actuator_index] = reference[-1][waypoint_index]
                mujoco.mj_step(mj_model, mj_data)
            viewer.sync()

            if not loop:
                # Keep the window responsive until the user closes it.
                while viewer.is_running():
                    viewer.sync()
                    time.sleep(0.01)


if __name__ == "__main__":
    tyro.cli(main)
