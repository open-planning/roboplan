from pathlib import Path
import time

import numpy as np
import pinocchio as pin
from roboplan import (
    computeFramePath,
    get_package_share_dir,
    JointConfiguration,
    JointPath,
    Scene,
    RRTOptions,
    RRT,
)
from roboplan.viser_visualizer import ViserVisualizer


def visualizePath(
    viz: ViserVisualizer,
    scene: Scene,
    rrt: RRT,
    path: JointPath,
    frame_name: str,
    max_step_size: float,
) -> None:
    """
    Helper function to visualize the RRT path.
    TODO: Move this to the actual Python package itself.
    """
    start_nodes, goal_nodes = rrt.getNodes()

    start_segments = []
    for node in start_nodes[1:]:
        q_start = start_nodes[node.parent_id].config
        q_end = node.config
        frame_path = computeFramePath(scene, q_start, q_end, frame_name, max_step_size)
        for idx in range(len(frame_path) - 1):
            start_segments.append([frame_path[idx][:3, 3], frame_path[idx + 1][:3, 3]])

    goal_segments = []
    for node in goal_nodes[1:]:
        q_start = goal_nodes[node.parent_id].config
        q_end = node.config
        frame_path = computeFramePath(scene, q_start, q_end, frame_name, max_step_size)
        for idx in range(len(frame_path) - 1):
            goal_segments.append([frame_path[idx][:3, 3], frame_path[idx + 1][:3, 3]])

    path_segments = []
    for idx in range(len(path.positions) - 1):
        q_start = path.positions[idx]
        q_end = path.positions[idx + 1]
        frame_path = computeFramePath(scene, q_start, q_end, frame_name, max_step_size)
        for idx in range(len(frame_path) - 1):
            path_segments.append([frame_path[idx][:3, 3], frame_path[idx + 1][:3, 3]])

    if start_segments:
        viz.viewer.scene.add_line_segments(
            "/rrt/start_tree",
            points=np.array(start_segments),
            colors=(0, 100, 100),
            line_width=1.0,
        )
    if goal_segments:
        viz.viewer.scene.add_line_segments(
            "/rrt/goal_tree",
            points=np.array(goal_segments),
            colors=(100, 0, 100),
            line_width=1.0,
        )

    viz.viewer.scene.add_line_segments(
        "/rrt/path",
        points=np.array(path_segments),
        colors=(100, 100, 0),
        line_width=3.0,
    )


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

    # Set up an RRT and perform path planning.
    options = RRTOptions()
    options.max_connection_distance = 1.0
    options.collision_check_step_size = 0.05
    options.max_planning_time = 3.0
    options.rrt_connect = True
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
    visualizePath(viz, scene, rrt, path, "tool0", 0.05)

    while True:
        time.sleep(10.0)
