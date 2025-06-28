"""
Unit tests for RRT planners in RoboPlan.
"""

from pathlib import Path

import pytest

from roboplan import (
    get_package_share_dir,
    JointConfiguration,
    Scene,
    RRTOptions,
    RRT,
)


@pytest.fixture
def test_scene() -> Scene:
    roboplan_examples_dir = Path(get_package_share_dir())
    urdf_path = roboplan_examples_dir / "ur_robot_model" / "ur5_gripper.urdf"
    srdf_path = roboplan_examples_dir / "ur_robot_model" / "ur5_gripper.srdf"
    package_paths = [roboplan_examples_dir]

    return Scene("test_scene", urdf_path, srdf_path, package_paths)


def test_plan(test_scene: Scene) -> None:
    options = RRTOptions()
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
