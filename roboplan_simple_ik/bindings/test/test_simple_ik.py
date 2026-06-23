"""
Unit tests for the simple IK solver in RoboPlan.
"""

import numpy as np
import pytest

from roboplan.core import Box, CartesianConfiguration, JointConfiguration, Scene
from roboplan.example_models import get_package_models_dir, get_package_share_dir
from roboplan.simple_ik import SimpleIkOptions, SimpleIk

GROUP_NAME = "arm"
BASE_FRAME = "base"
TIP_FRAME = "tool0"

# The solver's default time budget is too tight to reliably converge on a slo
# or loaded machine (e.g., a debug CI build), so give it generous headroom.
MAX_SOLVE_TIME = 1.0


@pytest.fixture
def test_scene() -> Scene:
    roboplan_models_dir = get_package_models_dir()
    urdf_path = roboplan_models_dir / "ur_robot_model" / "ur5_gripper.urdf"
    srdf_path = roboplan_models_dir / "ur_robot_model" / "ur5_gripper.srdf"
    package_paths = [get_package_share_dir()]

    return Scene("test_scene", urdf_path, srdf_path, package_paths)


def reachable_goal(scene: Scene) -> CartesianConfiguration:
    """Builds a goal pose by running forward kinematics on a known configuration."""
    q = scene.toFullJointPositions(
        GROUP_NAME, np.array([0.0, -1.0, 1.0, -1.5, -1.5, 0.0])
    )
    tform = scene.forwardKinematics(q, TIP_FRAME, BASE_FRAME)
    return CartesianConfiguration(BASE_FRAME, TIP_FRAME, tform)


def test_solve_ik(test_scene: Scene) -> None:
    # Happy path: solve for a reachable target.
    test_scene.setRngSeed(286)

    options = SimpleIkOptions(group_name=GROUP_NAME, max_time=MAX_SOLVE_TIME)
    ik = SimpleIk(test_scene, options)

    goal = reachable_goal(test_scene)
    start = JointConfiguration()
    start.positions = np.zeros(6)

    solution = JointConfiguration()
    assert ik.solveIk(goal, start, solution)

    # The solution must actually reach the goal pose.
    q_solution = test_scene.toFullJointPositions(GROUP_NAME, solution.positions)
    achieved = test_scene.forwardKinematics(q_solution, TIP_FRAME, BASE_FRAME)
    assert np.allclose(achieved, goal.tform, atol=1e-2)


def test_invalid_group_name(test_scene: Scene) -> None:
    # An unknown joint group must fail to even construct the solver.
    options = SimpleIkOptions(group_name="not_a_group")
    with pytest.raises(RuntimeError):
        SimpleIk(test_scene, options)


def test_invalid_tip_frame(test_scene: Scene) -> None:
    # An unknown tip frame must fail when solving.
    options = SimpleIkOptions(group_name=GROUP_NAME)
    ik = SimpleIk(test_scene, options)

    goal = CartesianConfiguration(BASE_FRAME, "not_a_frame", np.eye(4))
    start = JointConfiguration()
    start.positions = np.zeros(6)

    solution = JointConfiguration()
    with pytest.raises(RuntimeError):
        ik.solveIk(goal, start, solution)


def test_collision_checking(test_scene: Scene) -> None:
    # The same reachable target succeeds when collisions are ignored, but fails
    # once an obstacle makes every candidate configuration collide.
    test_scene.setRngSeed(286)

    goal = reachable_goal(test_scene)
    start = JointConfiguration()
    start.positions = np.zeros(6)
    solution = JointConfiguration()

    # Without collision checking, the solver reaches the target.
    options = SimpleIkOptions(
        group_name=GROUP_NAME,
        check_collisions=False,
        max_restarts=0,
        max_time=MAX_SOLVE_TIME,
    )
    ik = SimpleIk(test_scene, options)
    assert ik.solveIk(goal, start, solution)

    # Wrap the whole robot in a box so any configuration is in collision.
    box_tform = np.eye(4)
    test_scene.addBoxGeometry(
        "wall",
        "universe",
        Box(4.0, 4.0, 4.0),
        box_tform,
        np.array([0.5, 0.5, 0.5, 0.5]),
    )

    options.check_collisions = True
    ik = SimpleIk(test_scene, options)
    assert not ik.solveIk(goal, start, solution)
