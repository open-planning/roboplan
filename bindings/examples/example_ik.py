import sys
import time
import tyro
import xacro

import numpy as np
import pinocchio as pin

from common import MODELS, ROBOPLAN_EXAMPLES_DIR
from roboplan import (
    Scene,
    JointConfiguration,
    CartesianConfiguration,
    SimpleIkOptions,
    SimpleIk,
)
from roboplan.viser_visualizer import ViserVisualizer


def main(
    model: str = "ur5",
    max_iters: int = 100,
    step_size: float = 0.25,
    host: str = "localhost",
    port: str = "8000",
):
    """
    Run the IK example with the provided parameters.


    Parameters:
        model: The name of the model to user (ur5, franka, or dual).
        max_iters: Maximum number of iterations for the IK solver.
        step_size: Integration step size for the IK solver.
        host: The host for the ViserVisualizer.
        port: The port for the ViserVisualizer.
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

    # Create a redundant Pinocchio model just for visualization.
    # When Pinocchio 4.x releases nanobind bindings, we should be able to directly grab the model from the scene instead.
    model = pin.buildModelFromXML(urdf_xml)
    collision_model = pin.buildGeomFromUrdfString(
        model, urdf_xml, pin.GeometryType.COLLISION, package_dirs=package_paths
    )
    visual_model = pin.buildGeomFromUrdfString(
        model, urdf_xml, pin.GeometryType.VISUAL, package_dirs=package_paths
    )

    viz = ViserVisualizer(model, collision_model, visual_model)
    viz.initViewer(open=True, loadModel=True, host=host, port=port)

    # Set up an IK solver
    options = SimpleIkOptions()
    options.max_iters = max_iters
    options.step_size = step_size
    ik_solver = SimpleIk(scene, options)

    start = JointConfiguration()
    start.positions = np.array(model_data.starting_joint_config)
    goal = CartesianConfiguration()
    goal.base_frame = model_data.base_link
    goal.tip_frame = model_data.ee_names[0]
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
    try:
        while True:
            time.sleep(10.0)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    tyro.cli(main)
