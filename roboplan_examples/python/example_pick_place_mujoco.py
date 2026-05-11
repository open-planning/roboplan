#!/usr/bin/env python3

import time

import mujoco
import mujoco.viewer
import numpy as np
import pinocchio as pin
import tyro
import xacro

from common import MODELS
from roboplan.core import CartesianConfiguration, JointConfiguration, JointPath, Scene
from roboplan.example_models import get_package_share_dir
from roboplan.rrt import RRT, RRTOptions
from roboplan.simple_ik import SimpleIk, SimpleIkOptions
from roboplan.toppra import PathParameterizerTOPPRA, SplineFittingMode


def make_tform(rot: np.ndarray, pos: np.ndarray) -> np.ndarray:
    """Build a Fortran-order 4x4 homogeneous transform."""
    return np.asfortranarray(pin.SE3(rot, pos).homogeneous)


def execute_trajectory(traj, data, viewer, dt: float, carry_cube: bool = False,
                       cube_jnt_addr: int = 0, hand_body_id: int = 0,
                       grasp_offset_world: np.ndarray = None) -> None:
    """Step through a joint trajectory in the MuJoCo viewer."""
    for i in range(len(traj.times)):
        if not viewer.is_running():
            break
        data.qpos[:7] = traj.positions[i]
        mujoco.mj_forward(data.model if hasattr(data, 'model') else mujoco.MjModel, data)
        if carry_cube:
            data.qpos[cube_jnt_addr:cube_jnt_addr + 3] = data.xpos[hand_body_id] + grasp_offset_world
            data.qpos[cube_jnt_addr + 3:cube_jnt_addr + 7] = data.xquat[hand_body_id]
            mujoco.mj_forward(data.model if hasattr(data, 'model') else mujoco.MjModel, data)
        viewer.sync()
        time.sleep(dt)


def main(
    # Task geometry
    cube_x: float = 0.5,
    cube_y: float = 0.0,
    place_x: float = 0.3,
    place_y: float = 0.4,
    # Grasp offsets
    pregrasp_z_offset: float = 0.2,
    grasp_z_offset: float = 0.12,
    lift_z_offset: float = 0.2,
    preplace_z_offset: float = 0.32,
    place_z_offset: float = 0.12,
    # IK
    ik_max_iters: int = 200,
    ik_max_time: float = 2.0,
    ik_max_restarts: int = 10,
    ik_step_size: float = 0.25,
    # RRT
    rrt_max_nodes: int = 4000,
    rrt_max_connection_distance: float = 1.5,
    rrt_collision_check_step_size: float = 0.05,
    rrt_goal_biasing_probability: float = 0.15,
    rrt_connect: bool = True,
    # TOPP-RA
    toppra_dt: float = 0.02,
    toppra_mode: SplineFittingMode = SplineFittingMode.Hermite,
    velocity_scale: float = 0.5,
) -> None:
    """Run a pick-and-place example with the Franka arm in MuJoCo simulation.

    Plans collision-free trajectories using roboplan and executes them in a
    MuJoCo physics simulation. The cube pose is read back from MuJoCo state
    to compute grasp targets.

    Parameters:
        cube_x: X position of the cube spawn location.
        cube_y: Y position of the cube spawn location.
        place_x: X position of the place target.
        place_y: Y position of the place target.
        pregrasp_z_offset: Height above cube center for the pre-grasp pose.
        grasp_z_offset: Height above cube center for the grasp pose.
        lift_z_offset: Height above grasp pose for the lift pose.
        preplace_z_offset: Height above place target for the pre-place pose.
        place_z_offset: Height above place target for the final descent.
        ik_max_iters: Maximum IK iterations per attempt.
        ik_max_time: Maximum IK solve time in seconds.
        ik_max_restarts: Maximum IK random restarts.
        ik_step_size: IK integration step size.
        rrt_max_nodes: Maximum RRT tree nodes.
        rrt_max_connection_distance: Maximum RRT connection distance.
        rrt_collision_check_step_size: RRT collision check step size.
        rrt_goal_biasing_probability: RRT goal biasing probability.
        rrt_connect: Whether to use RRT-Connect (bidirectional).
        toppra_dt: Time step for TOPP-RA trajectory generation.
        toppra_mode: Spline fitting mode for TOPP-RA.
        velocity_scale: Scale factor for trajectory velocity (lower = slower/smoother).
    """
    CUBE_HALF_SIZE = 0.025

    # --- MuJoCo scene setup ---
    spec = mujoco.MjSpec.from_file("external/mujoco_menagerie/franka_emika_panda/panda.xml")

    spec.worldbody.add_geom(
        type=mujoco.mjtGeom.mjGEOM_PLANE,
        size=[0, 0, 0.05],
        rgba=[0.8, 0.8, 0.8, 1],
    )

    cube_body = spec.worldbody.add_body(name="cube", pos=[cube_x, cube_y, CUBE_HALF_SIZE])
    cube_body.add_freejoint()
    cube_body.add_geom(
        type=mujoco.mjtGeom.mjGEOM_BOX,
        size=[CUBE_HALF_SIZE, CUBE_HALF_SIZE, CUBE_HALF_SIZE],
        rgba=[1, 0, 0, 1],
    )

    eq = spec.add_equality()
    eq.name = "grasp_weld"
    eq.type = mujoco.mjtEq.mjEQ_WELD
    eq.objtype = mujoco.mjtObj.mjOBJ_BODY
    eq.name1 = "hand"
    eq.name2 = "cube"
    eq.active = False

    model = spec.compile()
    data = mujoco.MjData(model)

    # --- Roboplan scene setup ---
    robot = MODELS["franka"]
    package_paths = [get_package_share_dir()]
    urdf_xml = xacro.process_file(robot.urdf_path).toxml()
    srdf_xml = xacro.process_file(robot.srdf_path).toxml()

    scene = Scene(
        "panda_scene",
        urdf=urdf_xml,
        srdf=srdf_xml,
        package_paths=package_paths,
        yaml_config_path=robot.yaml_config_path,
    )

    start_config = np.array(robot.starting_joint_config[:7])
    full_config = np.array(robot.starting_joint_config)
    scene.setJointPositions(full_config)

    data.qpos[:7] = start_config
    mujoco.mj_forward(model, data)

    # --- Read cube pose from MuJoCo state ---
    cube_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "cube")
    cube_pos = data.xpos[cube_body_id].copy()

    # --- Pose computation ---
    grasp_rot = np.array([
        [1,  0,  0],
        [0, -1,  0],
        [0,  0, -1],
    ])

    pregrasp_pos = cube_pos + np.array([0, 0, pregrasp_z_offset])
    grasp_pos = cube_pos + np.array([0, 0, grasp_z_offset])
    lift_pos = grasp_pos + np.array([0, 0, lift_z_offset])
    place_pos = np.array([place_x, place_y, CUBE_HALF_SIZE])

    pregrasp_tform = make_tform(grasp_rot, pregrasp_pos)
    grasp_tform = make_tform(grasp_rot, grasp_pos)
    lift_tform = make_tform(grasp_rot, lift_pos)
    preplace_tform = make_tform(grasp_rot, place_pos + np.array([0, 0, preplace_z_offset]))
    place_tform = make_tform(grasp_rot, place_pos + np.array([0, 0, place_z_offset]))

    # --- IK setup ---
    ik_options = SimpleIkOptions(
        group_name="fr3_arm",
        max_iters=ik_max_iters,
        max_time=ik_max_time,
        max_restarts=ik_max_restarts,
        step_size=ik_step_size,
        check_collisions=False,
    )
    ik = SimpleIk(scene, ik_options)
    q_indices = scene.getJointGroupInfo("fr3_arm").q_indices
    arm_joint_names = list(scene.getJointGroupInfo("fr3_arm").joint_names)

    goal = CartesianConfiguration()
    goal.base_frame = "fr3_link0"
    goal.tip_frame = "fr3_hand"

    start_jc = JointConfiguration()
    start_jc.positions = np.array(robot.starting_joint_config)[q_indices]

    def solve_ik(tform, seed, label):
        goal.tform = tform
        sol = JointConfiguration()
        ok = ik.solveIk([goal], seed, sol)
        print(f"{label} IK: {'OK' if ok else 'FAILED'}")
        return sol

    # --- IK for each waypoint ---
    pregrasp_solution = solve_ik(pregrasp_tform, start_jc, "Pre-grasp")
    grasp_solution = solve_ik(grasp_tform, pregrasp_solution, "Grasp")
    lift_solution = solve_ik(lift_tform, grasp_solution, "Lift")
    preplace_solution = solve_ik(preplace_tform, lift_solution, "Pre-place")
    place_solution = solve_ik(place_tform, preplace_solution, "Place")

    # --- Motion planning ---
    rrt_options = RRTOptions(
        group_name="fr3_arm",
        max_nodes=rrt_max_nodes,
        max_connection_distance=rrt_max_connection_distance,
        collision_check_step_size=rrt_collision_check_step_size,
        goal_biasing_probability=rrt_goal_biasing_probability,
        rrt_connect=rrt_connect,
    )
    rrt = RRT(scene, rrt_options)
    toppra = PathParameterizerTOPPRA(scene, "fr3_arm")

    def plan_rrt(start, goal_sol, label):
        path = rrt.plan(start, goal_sol)
        traj = toppra.generate(path, dt=toppra_dt, mode=toppra_mode, velocity_scale=velocity_scale)
        print(f"{label}: {len(traj.times)} points, {traj.times[-1]:.2f}s")
        return traj

    def plan_linear(start_sol, end_sol, label):
        path = JointPath()
        path.joint_names = arm_joint_names
        path.positions = [start_sol.positions, end_sol.positions]
        traj = toppra.generate(path, dt=toppra_dt, mode=toppra_mode, velocity_scale=velocity_scale)
        print(f"{label}: {len(traj.times)} points, {traj.times[-1]:.2f}s")
        return traj

    home_traj = plan_rrt(start_jc, pregrasp_solution, "Home -> pre-grasp")
    approach_traj = plan_linear(pregrasp_solution, grasp_solution, "Approach")
    lift_traj = plan_linear(grasp_solution, lift_solution, "Lift")
    transport_traj = plan_rrt(lift_solution, preplace_solution, "Transport")
    place_traj = plan_linear(preplace_solution, place_solution, "Place descent")
    retreat_traj = plan_rrt(place_solution, start_jc, "Retreat")

    # --- MuJoCo body IDs for cube tracking ---
    hand_body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "hand")
    cube_jnt_addr = model.jnt_qposadr[model.body_jntadr[cube_body_id]]

    def run_traj(traj, carry_cube=False, grasp_offset=None):
        for i in range(len(traj.times)):
            if not viewer.is_running():
                break
            data.qpos[:7] = traj.positions[i]
            mujoco.mj_forward(model, data)
            if carry_cube:
                data.qpos[cube_jnt_addr:cube_jnt_addr + 3] = data.xpos[hand_body_id] + grasp_offset
                data.qpos[cube_jnt_addr + 3:cube_jnt_addr + 7] = data.xquat[hand_body_id]
                mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(toppra_dt)

    # --- Simulation ---
    with mujoco.viewer.launch_passive(model, data) as viewer:
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        viewer.cam.distance = 2.5
        viewer.cam.lookat = [0.3, 0.0, 0.3]

        time.sleep(1.0)

        # home -> pre-grasp
        run_traj(home_traj)

        # open gripper
        data.qpos[7] = 0.04
        data.qpos[8] = 0.04
        mujoco.mj_forward(model, data)
        viewer.sync()
        time.sleep(0.3)

        # approach: pre-grasp -> grasp
        run_traj(approach_traj)

        # close gripper
        data.qpos[7] = CUBE_HALF_SIZE
        data.qpos[8] = CUBE_HALF_SIZE
        mujoco.mj_forward(model, data)
        viewer.sync()
        time.sleep(0.3)

        # record cube offset relative to hand
        grasp_offset_world = data.xpos[cube_body_id].copy() - data.xpos[hand_body_id].copy()

        # lift, transport, place descent — carry cube throughout
        run_traj(lift_traj, carry_cube=True, grasp_offset=grasp_offset_world)
        run_traj(transport_traj, carry_cube=True, grasp_offset=grasp_offset_world)
        run_traj(place_traj, carry_cube=True, grasp_offset=grasp_offset_world)

        # open gripper + release cube
        data.qpos[7] = 0.04
        data.qpos[8] = 0.04
        mujoco.mj_forward(model, data)
        viewer.sync()
        time.sleep(0.5)

        # retreat to home
        run_traj(retreat_traj)

        while viewer.is_running():
            mujoco.mj_step(model, data)
            viewer.sync()


if __name__ == "__main__":
    tyro.cli(main)
