import sys
import time
import tyro
import xacro

import matplotlib.pyplot as plt
import pinocchio as pin
from common import MODELS, ROBOPLAN_EXAMPLES_DIR
from roboplan import (
    JointConfiguration,
    PathParameterizerTOPPRA,
    PathShortcutter,
    Scene,
    RRTOptions,
    RRT,
)
from roboplan.viser_visualizer import ViserVisualizer
from roboplan.visualization import visualizePath, visualizeTree


def main(
    model: str = "ur5",
    max_connection_distance: float = 1.0,
    collision_check_step_size: float = 0.05,
    goal_biasing_probability: float = 0.15,
    max_nodes: int = 1000,
    max_planning_time: float = 5.0,
    rrt_connect: bool = False,
    include_shortcutting: bool = False,
    host: str = "localhost",
    port: str = "8000",
    rng_seed: int | None = None,
    include_obstacles: bool = False,
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
        rng_seed: The seed for selecting random start and end poses and solving RRT.
        include_obstacles: Whether or not to include additional obstacles in the scene.
    """

    if model not in MODELS:
        print(f"Invalid model requested: {model}")
        sys.exit(1)

    model_data = MODELS[model]
    package_paths = [ROBOPLAN_EXAMPLES_DIR]

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
    q_indices = scene.getJointGroupInfo(model_data.default_joint_group).q_indices

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
    options = RRTOptions()
    options.group_name = model_data.default_joint_group
    options.max_connection_distance = max_connection_distance
    options.collision_check_step_size = collision_check_step_size
    options.goal_biasing_probability = goal_biasing_probability
    options.max_nodes = max_nodes
    options.max_planning_time = max_planning_time
    options.rrt_connect = rrt_connect
    rrt = RRT(scene, options)

    if rng_seed:
        scene.setRngSeed(rng_seed)
        rrt.setRngSeed(rng_seed)

    q_full = scene.randomCollisionFreePositions()
    scene.setJointPositions(q_full)
    viz.display(q_full)
    time.sleep(0.1)

    start = JointConfiguration()
    start.positions = q_full[q_indices]
    assert start.positions is not None

    goal = JointConfiguration()
    goal.positions = scene.randomCollisionFreePositions()[q_indices]
    assert goal.positions is not None

    print("Planning...")
    path = rrt.plan(start, goal)
    assert path is not None
    print(f"Found a path:\n{path}")

    # Optionally include path shortening
    if include_shortcutting:
        print("Shortcutting path...")
        shortcutter = PathShortcutter(scene, model_data.default_joint_group)
        shortened_path = shortcutter.shortcut(
            path,
            max_step_size=options.collision_check_step_size,
            max_iters=1000,
        )
        print(f"Shortcutted path:\n{shortened_path}")

    # Set up TOPP-RA path parameterization
    print("Generating trajectory...")
    dt = 0.01
    toppra = PathParameterizerTOPPRA(scene, model_data.default_joint_group)
    traj = toppra.generate(shortened_path if include_shortcutting else path, dt)

    # Visualize the tree and path
    viz.display(q_full)
    visualizeTree(viz, scene, rrt, model_data.ee_names, 0.05)
    if include_shortcutting:
        visualizePath(viz, scene, path, model_data.ee_names, 0.05)
        visualizePath(
            viz,
            scene,
            traj,
            model_data.ee_names,
            0.05,
            (0, 100, 0),
            "/rrt/shortcut_path",
        )
    else:
        visualizePath(viz, scene, traj, model_data.ee_names, 0.05)

    # TODO: Make this a reusable function
    plt.ion()
    plt.plot(traj.times, traj.positions)
    plt.xlabel("Time")
    plt.ylabel("Joint positions")
    plt.title("Time-parameterized trajectory")
    dof_names = []
    for name in scene.getJointNames():
        for idx in range(scene.getJointInfo(name).num_position_dofs):
            dof_names.append(f"{name}:{idx}")
    plt.legend(dof_names)
    plt.show()

    # Animate the trajectory
    input("Press 'Enter' to animate the trajectory.")
    for q in traj.positions:
        q_full[q_indices] = q
        viz.display(q_full)
        time.sleep(dt)

    try:
        while True:
            time.sleep(10.0)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    tyro.cli(main)
