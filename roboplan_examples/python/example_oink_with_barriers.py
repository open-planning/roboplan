#!/usr/bin/env python3

import sys
import threading
import time
import tyro
import xacro

import numpy as np
import pinocchio as pin
from pinocchio.visualize import ViserVisualizer

from common import MODELS
from filters import SE3LowPassFilter
from roboplan.core import Scene, CartesianConfiguration
from roboplan.example_models import get_package_share_dir
from roboplan.optimal_ik import (
    ConfigurationTask,
    ConfigurationTaskOptions,
    FrameTask,
    FrameTaskOptions,
    Oink,
    PositionBarrier,
    PositionLimit,
    VelocityLimit,
)


def main(
    model: str = "ur5",
    task_gain: float = 1.0,
    lm_damping: float = 0.01,
    regularization: float = 1e-6,
    control_freq: float = 400.0,
    barrier_gain: float = 10.0,
    barrier_size: float = 0.5,
    safety_margin: float = 0.05,
    max_position_error: float = 0.15,
    max_rotation_error: float = 0.5,
    reference_filter_tau: float = 0.1,
    host: str = "localhost",
    port: str = "8000",
):
    """
    Tutorial on optimal IK with Control Barrier Functions (CBF) safety constraints.

    This example demonstrates how to use PositionBarrier to enforce safety constraints
    on end-effector motion. Barriers prevent the robot from leaving a safe region while
    tracking target poses.

    Parameters:
        model: The name of the model to use.
        task_gain: Task gain (alpha) for the IK solver (0-1).
        lm_damping: Levenberg-Marquardt damping for regularization.
        regularization: Tikhonov regularization weight for the QP Hessian. Higher values
            improve numerical stability but may reduce task tracking accuracy.
        control_freq: Control loop frequency in Hz.
        barrier_gain: Barrier gain for CBF constraint. Since the linear class-K function
            provides proportional force, use lower values (5-20) compared to saturating
            functions. Higher values = stronger barrier response.
        barrier_size: Size of the cubic barrier box around the EE start position (meters).
        safety_margin: Distance from boundary where barrier activates (meters). With linear
            barriers, smaller values (0.02-0.1m) are typically sufficient.
        max_position_error: Maximum position error magnitude in meters. Prevents large
            jumps that can invalidate CBF linearization.
        max_rotation_error: Maximum rotation error magnitude in radians. Prevents large
            jumps that can invalidate CBF linearization.
        reference_filter_tau: Time constant (tau) for exponential low-pass filter in seconds.
            The filter reaches ~63% of target per tau seconds. For tau=0.1, the filter
            smooths target pose changes to prevent sudden jumps. Set to 0 to disable.
        host: The host for the ViserVisualizer.
        port: The port for the ViserVisualizer.
    """

    if model not in MODELS:
        print(f"Invalid model requested: {model}")
        sys.exit(1)

    if reference_filter_tau < 0:
        print(f"Invalid reference_filter_tau: {reference_filter_tau} (must be >= 0)")
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
    joint_names = scene.getJointGroupInfo(model_data.default_joint_group).joint_names
    print(
        f"Number of joints in group '{model_data.default_joint_group}': {len(joint_names)}"
    )
    print(f"Joint names:")
    for i, name in enumerate(joint_names):
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

    # Set up the Oink solver
    oink = Oink(scene, model_data.default_joint_group)
    num_variables = len(oink.v_indices)
    print(f"\nConfiguration space dimension (nq): {len(q_full)}")
    print(f"Velocity space dimension (nv): {num_variables}")

    # Thread-safe access to scene
    scene_lock = threading.Lock()

    # Control loop time step
    dt = 1.0 / control_freq

    # Create position limit constraint
    position_limit = PositionLimit(oink, gain=1.0)

    # Create velocity limit constraint
    v_max = np.hstack(
        [scene.getJointInfo(name).limits.max_velocity for name in joint_names]
    )
    velocity_limit = VelocityLimit(oink, dt, v_max)

    constraints = [position_limit, velocity_limit]

    print(f"\nReference Filtering:")
    if reference_filter_tau > 0:
        print(
            f"  tau: {reference_filter_tau}s (time constant for exponential filter; "
            f"reaches ~63% of step per tau seconds for tau=0.1.)"
        )
    else:
        print(f"  tau: {reference_filter_tau}s (disabled, raw targets used)")

    # Validate starting joint configuration size (should match nq)
    q_canonical = np.array(model_data.starting_joint_config)
    if len(q_canonical) != len(q_full):
        print(
            f"\nWarning: starting_joint_config size ({len(q_canonical)}) doesn't match "
            f"configuration space dimension ({len(q_full)}), using current scene positions instead"
        )
        with scene_lock:
            q_canonical = scene.getCurrentJointPositions()
    print(
        f"\nUsing starting pose for '{model}' (configuration space size: {len(q_canonical)})"
    )
    print(f"  {q_canonical}")

    # Create a ConfigurationTask to regularize toward the starting pose
    joint_weights = np.full(num_variables, 0.05)
    config_options = ConfigurationTaskOptions(task_gain=0.1, lm_damping=0.0)
    config_task = ConfigurationTask(
        oink, q_canonical[oink.q_indices], joint_weights, config_options
    )

    # Task parameters (define before using in callbacks)
    # max_position_error and max_rotation_error prevent large error jumps that can
    # invalidate the linearized CBF constraint, improving barrier stability.
    task_options = FrameTaskOptions(
        position_cost=1.0,
        orientation_cost=0.1,
        task_gain=task_gain,
        lm_damping=lm_damping,
        max_position_error=max_position_error,
        max_rotation_error=max_rotation_error,
    )

    # First, create all frame tasks and controls
    frame_tasks = []
    transform_controls = []
    for name in model_data.ee_names:
        goal = CartesianConfiguration()
        goal.base_frame = model_data.base_link
        goal.tip_frame = name

        frame_task = FrameTask(oink, scene, goal, task_options)
        frame_tasks.append(frame_task)

        # Create an interactive marker
        controls = viz.viewer.scene.add_transform_controls(
            "/ik_marker/" + name,
            depth_test=False,
            scale=0.2,
            disable_sliders=True,
            visible=True,
        )
        transform_controls.append(controls)

    # Create reference filters for smooth target pose changes
    # With tau=0, filters act as pass-through; with tau>0, they smooth sudden changes
    reference_filters = []
    raw_targets = []  # Store unfiltered targets from user input
    for name in model_data.ee_names:
        initial_pose = scene.forwardKinematics(q_full, name)
        ref_filter = SE3LowPassFilter(tau=reference_filter_tau)
        ref_filter.reset(initial_pose)
        reference_filters.append(ref_filter)
        raw_targets.append(initial_pose.copy())

    # Now set up the callback after all controls are created
    def update_goals(_):
        global paused
        with scene_lock:
            for idx, controls in enumerate(transform_controls):
                tform = pin.SE3(
                    pin.Quaternion(controls.wxyz[[1, 2, 3, 0]]), controls.position
                ).homogeneous
                # Store the raw target from the marker
                raw_targets[idx] = tform.copy()
        paused = False

    # Attach the callback to all controls
    for controls in transform_controls:
        controls.on_update(update_goals)

    tasks = frame_tasks + [config_task]

    # Control loop
    running = True
    global paused
    paused = True  # Start paused until user moves marker

    def control_loop():
        delta_q = np.zeros(num_variables)
        delta_q_full = np.zeros(model_pin.nv)
        while running:
            loop_start = time.time()

            # Thread-safe scene access for IK solving
            if not paused:
                with scene_lock:
                    # Get current joint configuration
                    q_current = scene.getCurrentJointPositions()

                    # Update reference filters (tau=0 acts as pass-through)
                    for idx in range(len(frame_tasks)):
                        filtered_target = reference_filters[idx].update(
                            raw_targets[idx], dt
                        )
                        frame_tasks[idx].setTargetFrameTransform(filtered_target)

                    # Solve IK for one step with constraints and barriers
                    try:
                        oink.solveIk(
                            scene, tasks, constraints, barriers, delta_q, regularization
                        )
                    except RuntimeError as e:
                        delta_q = np.zeros(num_variables)
                        print(f"Warning: IK solver failed: {e}, using zero delta_q")

                    delta_q_full[oink.v_indices] = delta_q

                    # CRITICAL: Validate solution with enforceBarriers() using FK
                    # This catches cases where linearization error causes barrier violation
                    oink.enforceBarriers(scene, barriers, delta_q_full, tolerance=0.0)

                    # Integrate: delta_q is a displacement (already limited by VelocityLimit)
                    q_current = scene.integrate(q_current, delta_q_full)

                    # Update scene state and forward kinematics after applying velocities
                    # This ensures FK is current for the next iteration's solveIk
                    scene.setJointPositions(q_current)
                    for task in tasks:
                        if isinstance(task, FrameTask):
                            scene.forwardKinematics(q_current, task.frame_name)

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
        global paused
        paused = True
        with scene_lock:
            q_current = scene.getCurrentJointPositions()
            for idx, controls in enumerate(transform_controls):
                fk_tform = scene.forwardKinematics(
                    q_current, frame_tasks[idx].frame_name
                )
                controls.position = fk_tform[:3, 3]
                controls.wxyz = pin.Quaternion(fk_tform[:3, :3]).coeffs()[[3, 0, 1, 2]]
                # Reset raw target and filter state to current pose
                raw_targets[idx] = fk_tform.copy()
                reference_filters[idx].reset(fk_tform)
        viz.display(q_current)

    random_button = viz.viewer.gui.add_button("Randomize Pose")

    @random_button.on_click
    def randomize_position(_):
        global paused
        paused = True
        with scene_lock:
            q_rand = scene.randomCollisionFreePositions()
            scene.setJointPositions(q_rand)
        reset_position(_)

    # Display the arm and marker at the starting position
    q_full = q_canonical.copy()
    with scene_lock:
        scene.setJointPositions(q_full)

        # Initialize raw targets and filters to current EE poses
        for idx, name in enumerate(model_data.ee_names):
            initial_pose = scene.forwardKinematics(q_full, name)
            raw_targets[idx] = initial_pose.copy()
            reference_filters[idx].reset(initial_pose)

    # Create position barriers for each end-effector with conservative parameters
    # - High gain ensures strong resistance to boundary approach
    # - Safety margin shifts the effective boundary inward to account for linearization errors
    barriers = []
    barrier_colors = [
        ((255, 100, 100), (255, 50, 50)),  # Red for first EE
        ((100, 255, 100), (50, 255, 50)),  # Green for second EE
        ((100, 100, 255), (50, 50, 255)),  # Blue for third EE (if needed)
    ]

    for idx, ee_name in enumerate(model_data.ee_names):
        # Get the initial pose for this end-effector
        ee_pose = scene.forwardKinematics(q_full, ee_name)
        ee_pos = ee_pose[:3, 3]

        # Create barrier bounds centered at this EE's initial position
        half_size = barrier_size / 2.0
        p_min = ee_pos - half_size
        p_max = ee_pos + half_size

        # Create the barrier for this end-effector
        position_barrier = PositionBarrier(
            oink,
            scene,
            ee_name,
            p_min,
            p_max,
            dt,
            gain=barrier_gain,
            safe_displacement_gain=1.0,
            safety_margin=safety_margin,
        )
        barriers.append(position_barrier)

        # Visualize the barrier box in Viser with unique name and color per EE
        box_color, wireframe_color = barrier_colors[idx % len(barrier_colors)]
        viz.viewer.scene.add_box(
            f"/barrier_box_{ee_name}",
            dimensions=(barrier_size, barrier_size, barrier_size),
            position=ee_pos,
            color=box_color,
            opacity=0.15,
        )
        # Add wireframe edges for better visibility
        viz.viewer.scene.add_box(
            f"/barrier_box_wireframe_{ee_name}",
            dimensions=(barrier_size, barrier_size, barrier_size),
            position=ee_pos,
            color=wireframe_color,
            opacity=0.5,
            side="back",  # Only render back faces for wireframe effect
        )

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
