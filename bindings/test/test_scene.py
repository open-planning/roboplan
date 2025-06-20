"""
Unit tests for scenes in RoboPlan.
"""

from pathlib import Path

import pytest
import numpy as np

import roboplan
from roboplan.utils import get_example_resources_directory, get_package_path


@pytest.fixture
def test_scene() -> roboplan.Scene:
    roboplan_examples_dir = Path(get_example_resources_directory())
    urdf_path = roboplan_examples_dir / "ur_robot_model" / "ur5_gripper.urdf"
    srdf_path = roboplan_examples_dir / "ur_robot_model" / "ur5_gripper.srdf"
    package_paths = [get_package_path()]

    return roboplan.Scene("test_scene", urdf_path, srdf_path, package_paths)


def test_scene_properties(test_scene: roboplan.Scene) -> None:
    assert test_scene.getJointNames() == [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]


def test_random_positions(test_scene: roboplan.Scene) -> None:
    # Test subsequent pseudorandom values.
    orig_random_positions = test_scene.randomPositions()
    new_random_positions = test_scene.randomPositions()
    assert np.all(np.not_equal(orig_random_positions, new_random_positions))

    # Test seeded values.
    test_scene.setRngSeed(1234)
    orig_seeded_positions = test_scene.randomPositions()
    test_scene.setRngSeed(1234)  # reset seed
    new_seeded_positions = test_scene.randomPositions()
    assert np.all(np.equal(orig_seeded_positions, new_seeded_positions))
