"""
Unit tests for RRT planners in RoboPlan.
"""

from pathlib import Path

import pytest

from roboplan.core import JointConfiguration, Scene
from roboplan.example_models import get_install_prefix
from roboplan.rrt import RRTOptions, RRT


@pytest.fixture
def test_scene() -> Scene:
    roboplan_examples_dir = Path(get_install_prefix()) / "share"
    roboplan_models_dir = roboplan_examples_dir / "roboplan_example_models" / "models"
    urdf_path = roboplan_models_dir / "ur_robot_model" / "ur5_gripper.urdf"
    srdf_path = roboplan_models_dir / "ur_robot_model" / "ur5_gripper.srdf"
    package_paths = [roboplan_examples_dir]

    return Scene("test_scene", urdf_path, srdf_path, package_paths)


def test_plan(test_scene: Scene) -> None:
    options = RRTOptions()
    options.group_name = "arm"
    options.max_connection_distance = 1.0
    options.collision_check_step_size = 0.05

    rrt = RRT(test_scene, options)
    rrt.setRngSeed(1234)

    start = JointConfiguration()
    start.positions = test_scene.randomCollisionFreePositions()
    assert start.positions is not None

    goal = JointConfiguration()
    goal.positions = test_scene.randomCollisionFreePositions()
    assert goal.positions is not None

    path = rrt.plan(start, goal)
    assert path is not None
    print(path)
