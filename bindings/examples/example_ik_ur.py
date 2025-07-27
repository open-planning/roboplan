from pathlib import Path
import time

import numpy as np
import pinocchio as pin

from roboplan import (
    get_package_share_dir,
    Scene,
    JointConfiguration,
    CartesianConfiguration,
    SimpleIkOptions,
    SimpleIk,
)
from roboplan.viser_visualizer import ViserVisualizer


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

    # Set up an IK solver
    options = SimpleIkOptions()
    options.max_iters = 100
    options.step_size = 0.25
    ik_solver = SimpleIk(scene, options)

    start = JointConfiguration()
    goal = CartesianConfiguration()
    goal.base_frame = "base"
    goal.tip_frame = "tool0"
    solution = JointConfiguration()

    # Create an interactive marker.
    controls = viz.viewer.scene.add_transform_controls(
        "/ik_marker/",
        depth_test=False,
        scale=0.2,
        disable_sliders=True,
        visible=True,
    )

    @controls.on_update
    def solve_ik(_):
        goal.tform = pin.SE3(
            pin.Quaternion(controls.wxyz[[1, 2, 3, 0]]), controls.position
        ).homogeneous
        start.positions = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        result = ik_solver.solveIk(goal, start, solution)
        if result:
            viz.display(solution.positions)
            start.positions = solution.positions

    # Create a marker reset button.
    reset_button = viz.viewer.gui.add_button("Reset Marker")

    @reset_button.on_click
    def reset_position(_):
        fk_tform = scene.forwardKinematics(start.positions, goal.tip_frame)
        controls.position = fk_tform[:3, 3]
        controls.wxyz = pin.Quaternion(fk_tform[:3, :3]).coeffs()[[3, 0, 1, 2]]

    random_button = viz.viewer.gui.add_button("Randomize Pose")

    @random_button.on_click
    def randomize_position(_):
        start.positions = scene.randomCollisionFreePositions()
        viz.display(start.positions)
        reset_position(_)

    # Display the arm and marker at the starting position, then sleep forever.
    randomize_position(None)
    while True:
        time.sleep(10.0)
