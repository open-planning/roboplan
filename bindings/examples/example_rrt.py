import sys
import time
import tyro

import pinocchio as pin
from common import MODELS, ROBOPLAN_EXAMPLES_DIR
from roboplan import (
    shortcutPath,
    JointConfiguration,
    Scene,
    RRTOptions,
    RRT,
)
from roboplan.viser_visualizer import ViserVisualizer, visualizePath, visualizeTree


def main(
    model: str = "ur5",
    max_connection_distance: float = 1.0,
    collision_check_step_size: float = 0.05,
    goal_biasing_probability: float = 0.15,
    max_nodes: int = 1000,
    max_planning_time: float = 3.0,
    rrt_connect: bool = False,
    include_shortcutting: bool = False,
    host: str = "localhost",
    port: str = "8000",
):
    """
    Run the RRT example with the provided parameters.


    Parameters:
        model: The name of the model to user (ur5 or franka).
        max_connection_distance: Maximum connection distance between two search nodes.
        collision_check_step_size: Configuration-space step size for collision checking along edges.
        goal_biasing_probability: Weighting of the goal node during random sampling.
        max_nodes: The maximum number of nodes to add to the search tree.
        max_planning_time: The maximum time (in seconds) to search for a path.
        rrt_connect: Whether or not to use RRT-Connect.
        include_shortcutting: Whether or not to include path shortcutting for found paths.
        host: The host for the ViserVisualizer.
        port: The port for the ViserVisualizer.
    """

    if model not in MODELS:
        print(f"Invalid model requested: {model}")
        sys.exit(1)

    urdf_path, srdf_path, ee_name, _, _ = MODELS[model]
    package_paths = [ROBOPLAN_EXAMPLES_DIR]

    scene = Scene("test_scene", urdf_path, srdf_path, package_paths)

    # Create a redundant Pinocchio model just for visualization.
    # When Pinocchio 4.x releases nanobind bindings, we should be able to directly grab the model from the scene instead.
    model, collision_model, visual_model = pin.buildModelsFromUrdf(
        urdf_path, package_dirs=package_paths
    )
    viz = ViserVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=True, loadModel=True, host=host, port=port)

    # Optionally include path shortening
    include_shortcutting = True

    # Set up an RRT and perform path planning.
    options = RRTOptions()
    options.max_connection_distance = max_connection_distance
    options.collision_check_step_size = collision_check_step_size
    options.goal_biasing_probability = goal_biasing_probability
    options.max_nodes = max_nodes
    options.max_planning_time = max_planning_time
    options.rrt_connect = rrt_connect
    rrt = RRT(scene, options)

    start = JointConfiguration()
    start.positions = scene.randomCollisionFreePositions()
    assert start.positions is not None

    goal = JointConfiguration()
    goal.positions = scene.randomCollisionFreePositions()
    assert goal.positions is not None

    path = rrt.plan(start, goal)
    assert path is not None

    if include_shortcutting:
        shortcut_path = shortcutPath(
            scene, path, options.collision_check_step_size, 1000
        )

    # Visualize the tree and path
    print(path)
    viz.display(start.positions)
    visualizePath(viz, scene, path, ee_name, 0.05)
    visualizeTree(viz, scene, rrt, ee_name, 0.05)

    if include_shortcutting:
        print("Shortcutted path:")
        print(shortcut_path)
        visualizePath(
            viz, scene, shortcut_path, ee_name, 0.05, (0, 100, 0), "/rrt/shortcut_path"
        )

    try:
        while True:
            time.sleep(10.0)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    tyro.cli(main)
