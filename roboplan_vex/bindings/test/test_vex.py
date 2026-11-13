"""
Unit tests for the vex trajectory optimizer in RoboPlan.
"""

import numpy as np
import pytest

from roboplan.core import (
    Box,
    JointConfiguration,
    JointPath,
    Scene,
    Sphere,
    computePathLength,
    loadJointLimitsConfig,
    loadUrdfSceneDescription,
)
from roboplan.example_models import get_package_models_dir, get_package_share_dir
from roboplan.rrt import RRT, RRTOptions
from roboplan.vex import Vex, VexOptions, VexStatus


def _tform(xyz: list[float]) -> np.ndarray:
    tform = np.eye(4)
    tform[:3, 3] = xyz
    return tform


@pytest.fixture
def test_scene() -> Scene:
    models_dir = get_package_models_dir() / "ur_robot_model"
    description = loadUrdfSceneDescription(
        models_dir / "ur5_gripper.urdf", [get_package_share_dir()]
    )
    scene = Scene("test_scene", description)
    # The YAML config adds the acceleration limits vex needs.
    scene.importJointLimitsFromConfig(
        loadJointLimitsConfig(models_dir / "ur5_config.yaml")
    )
    scene.importSrdf((models_dir / "ur5_gripper.srdf").read_text())

    # The ur5 obstacles of roboplan_examples/python/common.py.
    color = np.array([0.5, 0.5, 0.5, 0.5])
    scene.addBoxGeometry(
        "test_box", "universe", Box(0.5, 0.5, 0.5), _tform([0.0, 0.0, 1.2]), color
    )
    scene.addSphereGeometry(
        "test_sphere", "universe", Sphere(0.3), _tform([0.75, 0.0, 0.25]), color
    )
    scene.setCollisions("test_sphere", "test_box", False)
    scene.addBoxGeometry(
        "ground_plane",
        "universe",
        Box(1.5, 1.5, 0.2),
        _tform([0.0, 0.0, -0.1]),
        color,
    )
    for body in ["base_link", "test_box", "test_sphere"]:
        scene.setCollisions("ground_plane", body, False)
    return scene


def test_options_are_read_write() -> None:
    options = VexOptions()
    options.group_name = "arm"
    options.num_nodes = 25
    options.samples_per_segment = 0
    assert (options.group_name, options.num_nodes, options.samples_per_segment) == (
        "arm",
        25,
        0,
    )


def test_rrt_path_is_shortened(test_scene: Scene) -> None:
    test_scene.setRngSeed(286)
    start = JointConfiguration()
    start.positions = test_scene.randomCollisionFreePositions()[
        test_scene.getJointGroupInfo("arm").q_indices
    ]
    goal = JointConfiguration()
    goal.positions = test_scene.randomCollisionFreePositions()[
        test_scene.getJointGroupInfo("arm").q_indices
    ]

    rrt_options = RRTOptions()
    rrt_options.group_name = "arm"
    rrt_options.max_connection_distance = 1.0
    rrt_options.collision_check_step_size = 0.05
    rrt = RRT(test_scene, rrt_options)
    rrt.setRngSeed(1234)
    path = rrt.plan(start, goal)

    options = VexOptions()
    options.group_name = "arm"
    # Kept coarse so the test stays fast; the post-check still covers the whole trajectory.
    options.num_nodes = 20
    options.samples_per_segment = 1
    vex = Vex(test_scene, options)
    result = vex.solve(start, goal, 5.0, path)
    assert result.status == VexStatus.Converged
    assert result.continuous_collision_free
    np.testing.assert_array_equal(result.trajectory.positions[0], start.positions)
    np.testing.assert_array_equal(result.trajectory.positions[-1], goal.positions)

    # vex shortens the raw (not shortcutted) RRT path.
    optimized = JointPath()
    optimized.joint_names = result.trajectory.joint_names
    optimized.positions = result.trajectory.positions
    assert computePathLength(test_scene, "arm", optimized) < computePathLength(
        test_scene, "arm", path
    )

    # Same input, same output.
    again = vex.solve(start, goal, 5.0, path)
    np.testing.assert_array_equal(
        np.array(again.trajectory.positions), np.array(result.trajectory.positions)
    )


def test_start_in_collision_raises(test_scene: Scene) -> None:
    # Folding the elbow fully back drives the forearm into the upper arm.
    start = JointConfiguration()
    start.positions = np.array([0.0, -1.57, 3.1, 0.0, 0.0, 0.0])
    goal = JointConfiguration()
    goal.positions = np.array([0.0, -1.57, 1.57, 0.0, 0.0, 0.0])
    seed = JointPath()
    seed.joint_names = test_scene.getJointGroupInfo("arm").joint_names
    seed.positions = [start.positions, goal.positions]

    options = VexOptions()
    options.group_name = "arm"
    with pytest.raises(RuntimeError, match="collision"):
        Vex(test_scene, options).solve(start, goal, 5.0, seed)


def test_endpoint_within_margin_raises(test_scene: Scene) -> None:
    # Collision-free endpoints, but a 0.5 m margin is more than their clearance.
    start = JointConfiguration()
    start.positions = np.array([0.0, -1.57, 1.57, 0.0, 0.0, 0.0])
    goal = JointConfiguration()
    goal.positions = np.array([0.5, -1.57, 1.57, 0.0, 0.0, 0.0])
    seed = JointPath()
    seed.joint_names = test_scene.getJointGroupInfo("arm").joint_names
    seed.positions = [start.positions, goal.positions]

    options = VexOptions()
    options.group_name = "arm"
    options.collision_margin = 0.5
    options.collision_check_distance = 0.6
    with pytest.raises(RuntimeError, match="clearance below collision_margin"):
        Vex(test_scene, options).solve(start, goal, 5.0, seed)
