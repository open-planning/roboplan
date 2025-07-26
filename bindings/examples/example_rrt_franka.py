from pathlib import Path
import time

import numpy as np
import pinocchio as pin
from roboplan import (
    get_package_share_dir,
    JointConfiguration,
    Scene,
    RRTOptions,
    RRT,
)
from roboplan.viser_visualizer import ViserVisualizer, visualizePath


if __name__ == "__main__":

    roboplan_examples_dir = Path(get_package_share_dir())
    urdf_path = roboplan_examples_dir / "franka_robot_model" / "fr3.urdf"
    srdf_path = roboplan_examples_dir / "franka_robot_model" / "fr3.srdf"
    package_paths = [roboplan_examples_dir]

    scene = Scene("test_scene", urdf_path, srdf_path, package_paths)

    # Create a redundant Pinocchio model just for visualization.
    # When Pinocchio 4.x releases nanobind bindings, we should be able to directly grab the model from the scene instead.
    model, collision_model, visual_model = pin.buildModelsFromUrdf(
        urdf_path, package_dirs=package_paths
    )
    viz = ViserVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=True, loadModel=True)

    # Set up an RRT and perform path planning.
    options = RRTOptions()
    options.max_connection_distance = 1.0
    options.collision_check_step_size = 0.05
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

    # Visualize the tree and path
    print(path)
    viz.display(start.positions)
    visualizePath(viz, scene, rrt, path, "fr3_hand", 0.05)

    while True:
        time.sleep(10.0)
