#!/usr/bin/env python3

import sys
import queue
import time
import tyro
import xacro

import matplotlib.pyplot as plt
import pinocchio as pin

from common import MODELS
from roboplan.core import JointConfiguration, PathShortcutter, Scene
from roboplan.example_models import get_package_share_dir
from roboplan.rrt import RRTOptions, RRT
from roboplan.toppra import PathParameterizerTOPPRA
from roboplan.viser_visualizer import ViserVisualizer
from roboplan.visualization import (
    visualizeJointTrajectory,
    visualizePath,
    plotJointTrajectory,
    visualizeTree,
)


def main(
    model: str = "ur5",
    max_connection_distance: float = 3.0,
    collision_check_step_size: float = 0.05,
    collision_check_use_bisection: bool = False,
    goal_biasing_probability: float = 0.15,
    max_nodes: int = 1000,
    max_planning_time: float = 5.0,
    rrt_connect: bool = False,
    include_shortcutting: bool = False,
    max_shortcutting_iters: int = 100,
    host: str = "localhost",
    port: str = "8000",
    rng_seed: int | None = None,
    include_obstacles: bool = False,
):
    """
    Run the RRT example with the provided parameters.

    Parameters:
        model: The name of the model to use.
        max_connection_distance: Maximum connection distance between two search nodes.
        collision_check_step_size: Configuration-space step size for collision checking along edges.
        collision_check_use_bisection: If true, uses bisection instead of linear search for collision checking along edges.
            This can be helpful in collision-dense environments, but has a lower worst-case performance.
        goal_biasing_probability: Weighting of the goal node during random sampling.
        max_nodes: The maximum number of nodes to add to the search tree.
        max_planning_time: The maximum time (in seconds) to search for a path.
        rrt_connect: Whether or not to use RRT-Connect.
        include_shortcutting: Whether or not to include path shortcutting for found paths.
        max_shortcutting_iters: The maximum number of path shortcutting iterations.
        host: The host for the ViserVisualizer.
        port: The port for the ViserVisualizer.
        rng_seed: The seed for selecting random start and end poses and solving RRT.
        include_obstacles: Whether or not to include additional obstacles in the scene.
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
        "test_scene",
        urdf=urdf_xml,
        srdf=srdf_xml,
        package_paths=package_paths,
        yaml_config_path=model_data.yaml_config_path,
    )
    group_info = scene.getJointGroupInfo(model_data.default_joint_group)
    q_indices = group_info.q_indices

    # Create a redundant Pinocchio model just for visualization.
    # When Pinocchio 4.x releases nanobind bindings, we should be able to directly grab the model from the scene instead.
    model = pin.buildModelFromXML(urdf_xml)
    collision_model = pin.buildGeomFromUrdfString(
        model, urdf_xml, pin.GeometryType.COLLISION, package_dirs=package_paths
    )
    visual_model = pin.buildGeomFromUrdfString(
        model, urdf_xml, pin.GeometryType.VISUAL, package_dirs=package_paths
    )

    # Optionally add obstacles.
    # Again, until Pinocchio 4.x releases nanobind bindings, we need to add the obstacles separately
    # to the scene and to the Pinocchio models used for visualization.
    if include_obstacles:
        for obstacle in model_data.obstacles:
            obstacle.addToScene(scene)
            obstacle.addToPinocchioModels(model, collision_model, visual_model)

    viz = ViserVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=True, loadModel=True, host=host, port=port)

    # Set up an RRT and perform path planning.
    options = RRTOptions(
        group_name=model_data.default_joint_group,
        max_nodes=max_nodes,
        max_connection_distance=max_connection_distance,
        collision_check_step_size=collision_check_step_size,
        collision_check_use_bisection=collision_check_use_bisection,
        goal_biasing_probability=goal_biasing_probability,
        max_planning_time=max_planning_time,
        rrt_connect=rrt_connect,
    )
    rrt = RRT(scene, options)

    toppra = PathParameterizerTOPPRA(scene, model_data.default_joint_group)
    traj_dt = 0.01

    if include_shortcutting:
        shortcutter = PathShortcutter(scene, model_data.default_joint_group)

    traj_queue = queue.Queue()
    cur_traj = None
    animate = False

    if rng_seed:
        rrt.setRngSeed(rng_seed)

    q_full = scene.randomCollisionFreePositions()
    scene.setJointPositions(q_full)
    viz.display(q_full)
    time.sleep(0.1)

    # Create a path planning button.
    plan_button = viz.viewer.gui.add_button("Plan path")

    @plan_button.on_click
    def plan_path(_):
        nonlocal animate
        animate = False
        plan_button.disabled = True
        animate_button.disabled = True

        start = JointConfiguration()
        start.positions = q_full[q_indices]
        assert start.positions is not None

        goal = JointConfiguration()
        goal.positions = scene.randomCollisionFreePositions()[q_indices]
        assert goal.positions is not None

        print("\nPlanning...")
        t_start = time.time()
        try:
            path = rrt.plan(start, goal)
        finally:
            plan_button.disabled = False
            animate_button.disabled = False
        print(f"Found a path in {time.time() - t_start:.3f} s")

        # Optionally include path shortening
        if include_shortcutting:
            print("Shortcutting path...")
            t_start = time.time()
            shortened_path = shortcutter.shortcut(
                path,
                max_step_size=options.collision_check_step_size,
                max_iters=max_shortcutting_iters,
            )
            print(f"Shortcutted path in {time.time() - t_start:.3f} s")

        # Set up TOPP-RA to time-parameterize the path
        print("Generating trajectory...")
        t_start = time.time()
        traj = toppra.generate(
            shortened_path if include_shortcutting else path, traj_dt
        )
        print(f"Generated trajectory in {time.time() - t_start:.3f} s")

        # Visualize the tree and path
        viz.display(q_full)
        visualizeTree(viz, scene, rrt, model_data.ee_names, 0.05)
        if include_shortcutting:
            visualizePath(
                viz, scene, path, model_data.ee_names, 0.05, (100, 0, 0), "/rrt/path"
            )
            visualizeJointTrajectory(
                viz,
                scene,
                traj,
                model_data.ee_names,
                (0, 100, 0),
                "/rrt/shortcut_path",
            )
        else:
            visualizeJointTrajectory(
                viz, scene, traj, model_data.ee_names, (100, 0, 0), "/rrt/path"
            )

        traj_queue.put(traj)
        plan_button.disabled = False
        animate_button.disabled = False

    # Create a trajectory animation button.
    animate_button = viz.viewer.gui.add_button("Animate trajectory")
    animate_button.disabled = True

    @animate_button.on_click
    def animate_trajectory(_):
        plan_button.disabled = True
        animate_button.disabled = True
        nonlocal animate
        animate = True

    # Main display and animation loop.
    plt.figure()
    plt.ion()
    while True:
        if not traj_queue.empty():
            plt.clf()
            cur_traj = traj_queue.get()
            fig = plotJointTrajectory(cur_traj, scene)
            plt.draw()
            fig.canvas.draw()
            fig.canvas.flush_events()
            plt.pause(0.1)
        elif animate and cur_traj is not None:
            print("Animating trajectory...")
            for q in cur_traj.positions:
                q_full[q_indices] = q
                viz.display(q_full)
                time.sleep(traj_dt)
            animate = False
            plan_button.disabled = False
            animate_button.disabled = False
            print("...done!")
        else:
            time.sleep(0.1)


if __name__ == "__main__":
    tyro.cli(main)
