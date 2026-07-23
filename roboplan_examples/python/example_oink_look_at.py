#!/usr/bin/env python3

import sys
import threading
import time
import tyro
import xacro

import numpy as np
import pinocchio as pin
from pinocchio.visualize import ViserVisualizer

from common import get_model_data
from roboplan.filters import SE3LowPassFilter
from roboplan.core import Scene
from roboplan.example_models import get_package_share_dir
from roboplan.optimal_ik import (
    AccelerationLimit,
    ConfigurationTask,
    ConfigurationTaskOptions,
    LookAtTask,
    LookAtTaskOptions,
    Oink,
    PositionLimit,
    SelfCollisionBarrier,
    SelfCollisionBarrierOptions,
    VelocityLimit,
)


def main(
    model: str = "ur5",
    look_distance: float = 0.3,
    orientation_cost: float = 1.0,
    distance_cost: float = 1.0,
    task_gain: float = 1.0,
    lm_damping: float = 0.01,
    regularization: float = 1e-3,
    control_freq: float = 100.0,
    reference_filter_tau: float = 0.1,
    self_collision_num_pairs: int = 0,
    self_collision_d_min: float = 0.02,
    self_collision_d_max: float = 0.25,
    self_collision_gain: float = 1.0,
    limit_acceleration: bool = False,
    host: str = "localhost",
    port: str = "8000",
):
    """
    Run the optimal IK look-at example.

    Each end effector points its tool Z axis at an interactive marker (the "object")
    while holding a fixed standoff distance from it. Drag the marker around and the
    arm will keep facing it.

    Parameters:
        model: The name of the model to use (ur5, franka, or dual).
        look_distance: Standoff distance (meters) to hold between the end effector
            and the target point.
        orientation_cost: Cost weight for the look-at alignment error.
        distance_cost: Cost weight for the standoff distance error.
        task_gain: Task gain (alpha) for the IK solver (0-1).
        lm_damping: Levenberg-Marquardt damping for regularization.
        regularization: Tikhonov regularization weight for the QP Hessian. Higher values
            improve numerical stability but may reduce task tracking accuracy.
        control_freq: Control loop frequency in Hz.
        reference_filter_tau: Time constant for reference filtering in seconds. Smooths
            target point changes to prevent sudden jumps. Set to 0 to disable filtering.
        self_collision_num_pairs: Number of collision pairs to use for the solver's
            self-collision barrier. If zero, no collision barrier will be used.
            Note that this can significantly increase solve time, especially for models
            that use high-resolution meshes for collision geometries.
        self_collision_d_min: Minimum distance (meters) the IK solver will try to keep
            between every pair of self-collision bodies declared by the SRDF.
        self_collision_d_max: Maximum distance (meters) the IK solver will use for a
            broadphase culling step. This can significantly speed up collision checking
            by pruning out far-away meshes, especially if they have complex geometries.
        self_collision_gain: Barrier gain (gamma) for the self-collision barrier. Higher
            values produce stronger pushback as bodies approach `self_collision_d_min`.
        limit_acceleration: If true, adds an acceleration limit.
            Note that this can cause overshoot with sudden marker motions, though.
        host: The host for the ViserVisualizer.
        port: The port for the ViserVisualizer.
    """
    model_data = get_model_data().get(model)
    if model_data is None:
        print(f"Invalid model requested: {model}")
        sys.exit(1)

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

    # Create a redundant Pinocchio model just for visualization with mimic joints.
    # When Pinocchio 4.x releases nanobind bindings, we should be able to directly grab the model from the scene instead.
    model_pin = pin.buildModelFromXML(urdf_xml, mimic=True)
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

    if limit_acceleration:
        a_max = np.hstack(
            [scene.getJointInfo(name).limits.max_acceleration for name in joint_names]
        )
        accel_limit = AccelerationLimit(oink, dt, a_max)
        constraints.append(accel_limit)

    # Self-collision barrier: keep every collision pair in the model at least
    # `self_collision_d_min` meters apart. Skip when the model has no collision pairs.
    if self_collision_num_pairs > 0:
        print(
            f"Self-collision barrier enabled with {self_collision_num_pairs} collision pair(s)."
        )
        self_collision_barrier = SelfCollisionBarrier(
            oink,
            scene,
            dt=dt,
            options=SelfCollisionBarrierOptions(
                n_collision_pairs=self_collision_num_pairs,
                gain=self_collision_gain,
                safe_displacement_gain=0.01,
                d_min=self_collision_d_min,
                d_max=self_collision_d_max,
            ),
        )
        barriers = [self_collision_barrier]
    else:
        barriers = []

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

    # Create a ConfigurationTask to regularize toward the starting pose.
    # The task runs at priority 2 so it is projected into the nullspace of the (priority 1)
    # LookAtTask. The look-at task only constrains 3 of the end effector's degrees of freedom
    # (facing direction + distance), so this regularization resolves the remaining redundancy
    # without sacrificing the look-at behavior.
    joint_weights = np.full(num_variables, 0.05)
    config_options = ConfigurationTaskOptions(task_gain=1.0, lm_damping=0.0, priority=2)
    config_task = ConfigurationTask(
        oink, q_canonical[oink.q_indices], joint_weights, config_options
    )

    # Task parameters. The look axis is the frame's local +Z axis by default.
    look_axis = np.array([0.0, 0.0, 1.0])
    task_options = LookAtTaskOptions(
        orientation_cost=orientation_cost,
        distance_cost=distance_cost,
        task_gain=task_gain,
        lm_damping=lm_damping,
        look_axis=look_axis,
    )

    def target_point_from_fk(fk_tform):
        """The point at `look_distance` along the frame's look axis, in world coordinates."""
        return fk_tform[:3, 3] + fk_tform[:3, :3] @ look_axis * look_distance

    # First, create all look-at tasks and controls
    look_tasks = []
    transform_controls = []
    for name in model_data.ee_names:
        initial_point = target_point_from_fk(scene.forwardKinematics(q_full, name))

        look_task = LookAtTask(oink, scene, name, initial_point, look_distance, task_options)
        look_tasks.append(look_task)

        # Create an interactive marker for the point to look at. Rotations are
        # disabled since only the marker's position matters.
        controls = viz.viewer.scene.add_transform_controls(
            "/look_target/" + name,
            depth_test=False,
            scale=0.2,
            disable_sliders=True,
            disable_rotations=True,
            visible=True,
        )
        transform_controls.append(controls)

        # Add a small sphere as the "object" being looked at. It is parented to the
        # marker, so it follows the marker as it is dragged around.
        viz.viewer.scene.add_icosphere(
            "/look_target/" + name + "/object",
            radius=0.03,
            color=(255, 180, 0),
        )

    # Create reference filters for smooth target point changes
    # These filters smooth sudden changes in the target point to prevent large jumps.
    # The target is stored as a homogeneous transform with identity rotation so the
    # SE3 filter can be reused; only the translation part is meaningful.
    reference_filters = []
    raw_targets = []  # Store unfiltered targets from user input
    for look_task in look_tasks:
        initial_target = np.eye(4)
        initial_target[:3, 3] = look_task.target_point
        ref_filter = SE3LowPassFilter(tau=reference_filter_tau)
        ref_filter.reset(initial_target)
        reference_filters.append(ref_filter)
        raw_targets.append(initial_target.copy())

    # Now set up the callback after all controls are created
    def update_goals(_):
        global paused
        with scene_lock:
            for idx, controls in enumerate(transform_controls):
                raw_targets[idx][:3, 3] = controls.position
        paused = False

    # Attach the callback to all controls
    for controls in transform_controls:
        controls.on_update(update_goals)

    tasks = look_tasks + [config_task]

    # Control loop
    running = True
    global paused
    paused = True  # Start paused until user moves marker

    def control_loop():
        delta_q = np.zeros(num_variables)
        delta_q_full = np.zeros(model_pin.nv)
        # Visualization is throttled and runs outside the scene lock so the Viser push to
        # the browser cannot stretch the control period. The solver assumes a fixed dt, so
        # keeping the control loop at a steady rate prevents jitter.
        display_period = max(dt, 1.0 / 30.0)
        last_display = 0.0
        while running:
            loop_start = time.time()
            q_to_display = None

            # Thread-safe scene access for IK solving
            if not paused:
                with scene_lock:
                    # Get current joint configuration
                    q_current = scene.getCurrentJointPositions()

                    # Update reference filters if enabled (smooths target point changes)
                    # The filter gradually approaches the raw target to prevent sudden jumps
                    if reference_filter_tau > 0:
                        for idx, ref_filter in enumerate(reference_filters):
                            filtered_target = ref_filter.update(raw_targets[idx], dt)
                            look_tasks[idx].setTargetPoint(filtered_target[:3, 3])
                    else:
                        # No filtering - use raw targets directly
                        for idx in range(len(look_tasks)):
                            look_tasks[idx].setTargetPoint(raw_targets[idx][:3, 3])

                    # Center the acceleration bound on the previous step's velocity
                    # (delta_q / dt) so the limit couples consecutive control steps.
                    if limit_acceleration:
                        accel_limit.setLastVelocity(delta_q / dt)

                    # Solve IK for one step with constraints (and the self-collision
                    # barrier when the model has collision pairs).
                    try:
                        oink.solveIk(
                            scene, tasks, constraints, barriers, delta_q, regularization
                        )
                    except RuntimeError as e:
                        delta_q = np.zeros(num_variables)
                        print(f"Warning: IK solver failed: {e}, using zero delta_q")

                    # Integrate: delta_q is a displacement (already limited by VelocityLimit)
                    delta_q_full[oink.v_indices] = delta_q

                    # Validate barrier feasibility post-solve and zero delta_q on violation.
                    if barriers:
                        oink.enforceBarriers(
                            scene, barriers, delta_q_full, tolerance=0.0
                        )

                    q_current = scene.integrate(q_current, delta_q_full)

                    # Update scene state and forward kinematics after applying velocities
                    # This ensures FK is current for the next iteration's solveIk
                    scene.setJointPositions(q_current)
                    for task in tasks:
                        if isinstance(task, LookAtTask):
                            scene.forwardKinematics(q_current, task.frame_name)

                    q_to_display = q_current
            else:
                # While paused (including just after a reset), hold the filter and
                # last-velocity state at rest so resuming starts from zero velocity
                # rather than replaying a stale displacement.
                delta_q[:] = 0.0
                delta_q_full[:] = 0.0

            # Throttled visualization, outside the scene lock, so a slow browser push does
            # not perturb the control-loop timing.
            if (
                q_to_display is not None
                and (loop_start - last_display) >= display_period
            ):
                viz.display(q_to_display)
                last_display = loop_start

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
                    q_current, look_tasks[idx].frame_name
                )
                target_point = target_point_from_fk(fk_tform)
                controls.position = target_point
                # Reset raw target and filter state to the current look point
                raw_targets[idx][:3, 3] = target_point
                look_tasks[idx].setTargetPoint(target_point)
                if reference_filter_tau > 0:
                    reference_filters[idx].reset(raw_targets[idx])
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

        # Initialize raw targets and filters to the current look points
        for idx, name in enumerate(model_data.ee_names):
            target_point = target_point_from_fk(scene.forwardKinematics(q_full, name))
            raw_targets[idx][:3, 3] = target_point
            look_tasks[idx].setTargetPoint(target_point)
            if reference_filter_tau > 0:
                reference_filters[idx].reset(raw_targets[idx])

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
