#!/usr/bin/env python3

import sys
import threading
import time
import tyro
import xacro

import numpy as np
import pinocchio as pin

from common import MODELS
from roboplan.core import Scene, CartesianConfiguration
from roboplan.example_models import get_package_share_dir
from roboplan.optimal_ik import (
    ConfigurationTask,
    ConfigurationTaskParams,
    FrameTask,
    FrameTaskParams,
    Oink,
    PositionLimit,
    VelocityLimit,
)
from roboplan.viser_visualizer import ViserVisualizer


def main(
    model: str = "ur5",
    task_gain: float = 1.0,
    lm_damping: float = 2.0,
    control_freq: float = 100.0,
    max_joint_velocity: float = 1.0,
    host: str = "localhost",
    port: str = "8000",
):
    """
    Run the optimal IK example with the provided parameters.

    Parameters:
        model: The name of the model to use (ur5, franka, or dual).
        task_gain: Task gain (alpha) for the IK solver (0-1).
        lm_damping: Levenberg-Marquardt damping for regularization.
        control_freq: Control loop frequency in Hz.
        max_joint_velocity: Maximum joint velocity in rad/s for all joints.
        host: The host for the ViserVisualizer.
        port: The port for the ViserVisualizer.
    """

    if model not in MODELS:
        print(f"Invalid model requested: {model}")
        sys.exit(1)

    model_data = MODELS[model]
    package_paths = [get_package_share_dir()]

    # Pre-process with xacro. This is not necessary for raw URDFs.
    urdf_xml = xacro.process_file(model_data.urdf_path).toxml()
    srdf_xml = xacro.process_file(model_data.srdf_path).toxml()

    # Specify argument names to distinguish overloaded Scene constructors from python.
    scene = Scene(
        "oink_scene",
        urdf=urdf_xml,
        srdf=srdf_xml,
        package_paths=package_paths,
        yaml_config_path=model_data.yaml_config_path,
    )

    # Print joint information
    print(f"\n=== Model: {model} ===")
    joint_names = scene.getJointNames()
    actuated_joint_names = scene.getActuatedJointNames()
    print(f"Total joints: {len(joint_names)}")
    print(f"Actuated joints: {len(actuated_joint_names)}")
    print(f"\nAll joint names:")
    for i, name in enumerate(joint_names):
        print(f"  {i}: {name}")
    print(f"\nActuated joint names:")
    for i, name in enumerate(actuated_joint_names):
        print(f"  {i}: {name}")
    print()

    q_full = scene.getCurrentJointPositions()

    # Create a redundant Pinocchio model just for visualization.
    # When Pinocchio 4.x releases nanobind bindings, we should be able to directly grab the model from the scene instead.
    model_pin = pin.buildModelFromXML(urdf_xml)
    collision_model = pin.buildGeomFromUrdfString(
        model_pin, urdf_xml, pin.GeometryType.COLLISION, package_dirs=package_paths
    )
    visual_model = pin.buildGeomFromUrdfString(
        model_pin, urdf_xml, pin.GeometryType.VISUAL, package_dirs=package_paths
    )

    viz = ViserVisualizer(model_pin, collision_model, visual_model)
    viz.initViewer(open=True, loadModel=True, host=host, port=port)

    # Determine the velocity space dimension (nv) by computing a Jacobian
    # For models with quaternion joints, nv < nq
    jac = scene.computeFrameJacobian(q_full, model_data.ee_names[0])
    num_variables = jac.shape[1]  # nv (velocity space dimension)
    print(f"\nConfiguration space dimension (nq): {len(q_full)}")
    print(f"Velocity space dimension (nv): {num_variables}")

    # Set up the Oink solver with position and velocity limit constraints
    oink = Oink(num_variables)

    # Thread-safe access to tasks and scene
    tasks_lock = threading.Lock()
    scene_lock = threading.Lock()

    # Control loop time step
    dt = 1.0 / control_freq

    # Create position limit constraint
    position_limit = PositionLimit(num_variables, gain=1.0)

    # Create velocity limit constraint
    # v_max is in rad/s, the constraint limits delta_q to [-dt*v_max, dt*v_max]
    v_max = np.full(num_variables, max_joint_velocity)
    velocity_limit = VelocityLimit(num_variables, dt, v_max)

    constraints = [position_limit, velocity_limit]

    # Validate starting joint configuration size (should match nq)
    q_canonical_raw = np.array(model_data.starting_joint_config)
    if len(q_canonical_raw) != len(q_full):
        print(
            f"\nWarning: starting_joint_config size ({len(q_canonical_raw)}) doesn't match "
            f"configuration space dimension ({len(q_full)}), using current scene positions instead"
        )
        with scene_lock:
            q_canonical = scene.getCurrentJointPositions()
    else:
        q_canonical = q_canonical_raw
    print(
        f"\nUsing starting pose for '{model}' (configuration space size: {len(q_canonical)})"
    )
    print(f"  {q_canonical}")

    # Create a ConfigurationTask to regularize toward the starting pose
    # Joint weights: 0.2 for base joints, 0.1 for all others
    joint_weights = np.full(num_variables, 0.1)
    joint_weights[0] = 0.2
    # For dual-arm, also weight the right arm's base joint symmetrically
    if model == "dual":
        joints_per_arm = num_variables // 2  # 9 joints per arm (7 arm + 2 gripper)
        joint_weights[joints_per_arm] = 0.2
    config_params = ConfigurationTaskParams(task_gain=0.1, lm_damping=0.0)
    config_task = ConfigurationTask(q_canonical, joint_weights, config_params)

    # Task list for the control loop
    tasks = []

    # Task parameters (define before using in callbacks)
    task_params = FrameTaskParams(
        position_cost=2.0,
        orientation_cost=1.0,
        task_gain=task_gain,
        lm_damping=lm_damping,
    )

    # Goal configuration
    goals = []
    transform_controls = []

    # First, create all goals and controls
    for name in model_data.ee_names:
        goal = CartesianConfiguration()
        goal.base_frame = model_data.base_link
        goal.tip_frame = name
        goals.append(goal)

        # Create an interactive marker
        controls = viz.viewer.scene.add_transform_controls(
            "/ik_marker/" + name,
            depth_test=False,
            scale=0.2,
            disable_sliders=True,
            visible=True,
        )
        transform_controls.append(controls)

    # Now set up the callback after all controls are created
    def update_goals(_):
        # Thread-safe update of tasks
        with tasks_lock:
            tasks.clear()
            tasks.append(config_task)

            # Set the goal from the marker position
            for goal, controls in zip(goals, transform_controls):
                goal.tform = pin.SE3(
                    pin.Quaternion(controls.wxyz[[1, 2, 3, 0]]), controls.position
                ).homogeneous

                # Create a FrameTask for this goal
                frame_task = FrameTask(goal.tip_frame, goal, num_variables, task_params)
                tasks.append(frame_task)

    # Attach the callback to all controls
    for controls in transform_controls:
        controls.on_update(update_goals)

    # Control loop
    running = True

    def control_loop():
        while running:
            loop_start = time.time()

            # Thread-safe task access
            with tasks_lock:
                current_tasks = tasks.copy()

            # Only solve if we have tasks
            if current_tasks:
                # Thread-safe scene access for IK solving
                with scene_lock:
                    # Get current joint configuration
                    q_current = scene.getCurrentJointPositions()

                    # Solve IK for one step with constraints
                    try:
                        delta_q = oink.solveIk(current_tasks, constraints, scene)
                    except RuntimeError as e:
                        print(f"Warning: IK solver failed: {e}, using zero delta_q")
                        delta_q = np.zeros(num_variables)

                    # Integrate: delta_q is a displacement (already limited by VelocityLimit)
                    q_current = scene.integrate(q_current, delta_q)

                    # Update scene state
                    scene.setJointPositions(q_current)

                    # Update forward kinematics after applying velocities
                    # This ensures FK is current for the next iteration's solveIk
                    for goal in goals:
                        scene.forwardKinematics(q_current, goal.tip_frame)

                viz.display(q_current)

            # Maintain control loop rate
            elapsed = time.time() - loop_start
            time.sleep(max(0, dt - elapsed))

    # Start control loop in separate thread
    control_thread = threading.Thread(target=control_loop, daemon=True)
    control_thread.start()

    # Create a marker reset button.
    reset_button = viz.viewer.gui.add_button("Reset Marker")

    @reset_button.on_click
    def reset_position(_):
        with tasks_lock:
            tasks.clear()
        with scene_lock:
            q_current = scene.getCurrentJointPositions()
            for goal, controls in zip(goals, transform_controls):
                fk_tform = scene.forwardKinematics(q_current, goal.tip_frame)
                controls.position = fk_tform[:3, 3]
                controls.wxyz = pin.Quaternion(fk_tform[:3, :3]).coeffs()[[3, 0, 1, 2]]

    random_button = viz.viewer.gui.add_button("Randomize Pose")

    @random_button.on_click
    def randomize_position(_):
        with tasks_lock:
            tasks.clear()
        with scene_lock:
            q_rand = scene.randomCollisionFreePositions()
            scene.setJointPositions(q_rand)
        reset_position(_)
        viz.display(q_rand)

    # Display the arm and marker at the starting position
    q_full = q_canonical.copy()
    with scene_lock:
        scene.setJointPositions(q_full)
    viz.display(q_full)
    reset_position(None)

    # Sleep forever, control loop runs in background thread
    try:
        while True:
            time.sleep(10.0)
    except KeyboardInterrupt:
        running = False
        control_thread.join(timeout=1.0)


if __name__ == "__main__":
    tyro.cli(main)
