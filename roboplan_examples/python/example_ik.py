#!/usr/bin/env python3

import sys
import time

import numpy as np
import pinocchio as pin
import tyro
import xacro
from common import get_model_data
from pinocchio.visualize import ViserVisualizer

from roboplan.core import (
    CartesianConfiguration,
    JointConfiguration,
    Scene,
    loadJointLimitsConfig,
    loadUrdfSceneDescriptionFromXml,
)
from roboplan.example_models import get_package_share_dir
from roboplan.simple_ik import SimpleIk, SimpleIkOptions


def main(
    model: str = "ur5",
    max_iters: int = 100,
    step_size: float = 1.0,
    max_linear_error_norm: float = 0.001,
    max_angular_error_norm: float = 0.001,
    check_collisions: bool = True,
    host: str = "localhost",
    port: str = "8000",
):
    """
    Drag an end-effector marker in Viser and solve IK to follow it.

    Parameters:
        model: The name of the model to use.
        max_iters: Maximum number of iterations for the IK solver.
        step_size: Integration step size for the IK solver.
        max_linear_error_norm: The maximum linear error norm (m) for the IK solver.
        max_angular_error_norm: The maximum angular error norm (rad) for the IK solver.
        check_collisions: Whether to check for collisions when solving IK.
        host: The host for the ViserVisualizer.
        port: The port for the ViserVisualizer.
    """
    model_data = get_model_data().get(model)
    if model_data is None:
        print(f"Invalid model requested: {model}")
        sys.exit(1)

    # Pre-process with xacro. This is not necessary for raw URDFs.
    urdf_xml = xacro.process_file(model_data.urdf_path).toxml()
    srdf_xml = xacro.process_file(model_data.srdf_path).toxml()
    package_paths = [get_package_share_dir()]

    scene = Scene(
        "test_scene",
        loadUrdfSceneDescriptionFromXml(urdf_xml, package_paths),
    )
    scene.importJointLimitsFromConfig(
        loadJointLimitsConfig(model_data.yaml_config_path)
    )
    scene.importSrdf(srdf_xml)
    q_indices = scene.getJointGroupInfo(model_data.default_joint_group).q_indices

    # Build a separate Pinocchio model (with mimic joints) for visualization. Until Pinocchio
    # and Coal have nanobind bindings, it cannot be taken from the scene.
    model = pin.buildModelFromXML(urdf_xml, mimic=True)
    collision_model = pin.buildGeomFromUrdfString(
        model, urdf_xml, pin.GeometryType.COLLISION, package_dirs=package_paths
    )
    visual_model = pin.buildGeomFromUrdfString(
        model, urdf_xml, pin.GeometryType.VISUAL, package_dirs=package_paths
    )

    viz = ViserVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=True, loadModel=True, host=host, port=port)

    # Set up an IK solver
    options = SimpleIkOptions(
        group_name=model_data.default_joint_group,
        max_iters=max_iters,
        step_size=step_size,
        max_linear_error_norm=max_linear_error_norm,
        max_angular_error_norm=max_angular_error_norm,
        check_collisions=check_collisions,
    )
    ik_solver = SimpleIk(scene, options)

    start = JointConfiguration()
    start.positions = np.array(model_data.starting_joint_config)[q_indices]

    goals = []
    transform_controls = []
    solution = JointConfiguration()

    for ee_name in model_data.ee_names:
        goal = CartesianConfiguration()
        goal.base_frame = model_data.base_link
        goal.tip_frame = ee_name
        goals.append(goal)

    def solveIk(_):
        q_full = scene.getCurrentJointPositions()
        world_T_base = scene.forwardKinematics(q_full, model_data.base_link)
        for goal, controls in zip(goals, transform_controls):
            world_T_target = pin.SE3(
                pin.Quaternion(controls.wxyz[[1, 2, 3, 0]]), controls.position
            ).homogeneous
            goal.tform = np.linalg.inv(world_T_base) @ world_T_target

        result = ik_solver.solveIk(goals, start, solution)
        if result:
            q_full = scene.toFullJointPositions(
                model_data.default_joint_group, solution.positions
            )
            viz.display(q_full)
            scene.setJointPositions(q_full)
            start.positions = solution.positions

    for ee_name in model_data.ee_names:
        controls = viz.viewer.scene.add_transform_controls(
            f"/ik_markers/{ee_name}",
            depth_test=False,
            scale=0.2,
            disable_sliders=True,
            visible=True,
        )
        controls.on_update(solveIk)
        transform_controls.append(controls)

    reset_button = viz.viewer.gui.add_button("Reset Marker")

    @reset_button.on_click
    def reset_position(_):
        for goal, controls in zip(goals, transform_controls):
            world_T_target = scene.forwardKinematics(
                scene.getCurrentJointPositions(), goal.tip_frame
            )
            controls.position = world_T_target[:3, 3]
            controls.wxyz = pin.Quaternion(world_T_target[:3, :3]).coeffs()[
                [3, 0, 1, 2]
            ]

    random_button = viz.viewer.gui.add_button("Randomize Pose")

    @random_button.on_click
    def randomize_position(pos):
        q_rand = scene.randomCollisionFreePositions()[q_indices]
        q_full = scene.toFullJointPositions(model_data.default_joint_group, q_rand)
        scene.setJointPositions(q_full)
        viz.display(q_full)
        start.positions = q_rand
        reset_position(pos)

    # Display the arm and marker at the starting position, then sleep forever.
    randomize_position(None)
    try:
        while True:
            time.sleep(10.0)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    tyro.cli(main)
