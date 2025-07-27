from pathlib import Path
import time

import pinocchio as pin
from roboplan import (
    get_package_share_dir,
    shortcutPath,
    JointConfiguration,
    Scene,
    RRTOptions,
    RRT,
)
from roboplan.viser_visualizer import ViserVisualizer, visualizePath, visualizeTree


if __name__ == "__main__":

    roboplan_examples_dir = Path(get_package_share_dir())
    urdf_path = roboplan_examples_dir / "ur_robot_model" / "ur5_gripper.urdf"
    srdf_path = roboplan_examples_dir / "ur_robot_model" / "ur5_gripper.srdf"
    package_paths = [roboplan_examples_dir]

    scene = Scene("test_scene", urdf_path, srdf_path, package_paths)

    # Create a redundant Pinocchio model just for visualization.
    # When Pinocchio 4.x releases nanobind bindings, we should be able to directly grab the model from the scene instead.
    model, collision_model, visual_model = pin.buildModelsFromUrdf(
        urdf_path, package_dirs=package_paths
    )
    viz = ViserVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=True, loadModel=True)

    # Optionally include path shortening
    include_shortcutting = True

    # Set up an RRT and perform path planning.
    options = RRTOptions()
    options.max_connection_distance = 1.0
    options.collision_check_step_size = 0.05
    options.goal_biasing_probability = 0.15
    options.max_nodes = 1000
    options.max_planning_time = 3.0
    options.rrt_connect = False
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
    visualizePath(viz, scene, path, "tool0", 0.05)
    visualizeTree(viz, scene, rrt, "tool0", 0.05)

    if include_shortcutting:
        print("Shortcutted path:")
        print(shortcut_path)
        visualizePath(
            viz, scene, shortcut_path, "tool0", 0.05, (0, 100, 0), "/rrt/shortcut_path"
        )

    try:
        while True:
            time.sleep(10.0)
    except KeyboardInterrupt:
        pass
