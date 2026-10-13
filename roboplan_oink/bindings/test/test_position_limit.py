"""
Unit tests for the OInK PositionLimit constraint bindings in RoboPlan.
"""

import numpy as np
import pytest

from roboplan.core import (
    CartesianConfiguration,
    Scene,
    loadJointLimitsConfig,
    loadUrdfSceneDescription,
)
from roboplan.example_models import get_package_models_dir, get_package_share_dir
from roboplan.optimal_ik import (
    ConfigurationTask,
    FrameTask,
    IterativeSolveOptions,
    Oink,
    PositionLimit,
)

GROUP_NAME = "arm"
TIP_FRAME = "tool0"


@pytest.fixture
def test_scene() -> Scene:
    models_dir = get_package_models_dir() / "ur_robot_model"
    description = loadUrdfSceneDescription(
        models_dir / "ur5_gripper.urdf", [get_package_share_dir()]
    )
    scene = Scene("test_scene", description)
    scene.importJointLimitsFromConfig(
        loadJointLimitsConfig(models_dir / "ur5_config.yaml")
    )
    scene.importSrdf((models_dir / "ur5_gripper.srdf").read_text())
    return scene


@pytest.fixture
def oink(test_scene: Scene) -> Oink:
    return Oink(test_scene, GROUP_NAME)


def test_construction_defaults(oink: Oink) -> None:
    limit = PositionLimit(oink)
    assert limit.config_limit_gain == 1.0
    assert limit.max_pullback == 0.0


def test_construction_with_custom_limits(oink: Oink) -> None:
    n = oink.num_variables
    limit = PositionLimit(
        oink,
        gain=0.5,
        joint_lower_limits=np.full(n, -0.5),
        joint_upper_limits=np.full(n, 0.5),
        max_pullback=0.1,
    )
    assert limit.config_limit_gain == 0.5
    assert limit.max_pullback == 0.1

    limit.max_pullback = 0.2
    assert limit.max_pullback == 0.2


def test_construction_rejects_invalid_arguments(oink: Oink) -> None:
    n = oink.num_variables
    lower = np.full(n, -0.5)
    upper = np.full(n, 0.5)

    with pytest.raises(ValueError):
        PositionLimit(oink, joint_lower_limits=np.zeros(n + 1))
    with pytest.raises(ValueError):
        PositionLimit(oink, joint_upper_limits=np.zeros(n + 1))
    with pytest.raises(ValueError):
        PositionLimit(oink, joint_lower_limits=upper, joint_upper_limits=lower)
    with pytest.raises(ValueError):
        PositionLimit(oink, max_pullback=-0.1)


@pytest.mark.parametrize("max_pullback", [0.0, 0.1])
def test_solve_ik_pulls_back_by_max_pullback(oink: Oink, max_pullback: float) -> None:
    """A joint outside the custom limits moves back by at most max_pullback per step."""
    n = oink.num_variables
    q = np.zeros(n)
    q[0] = 0.8  # 0.3 above the upper limit
    q[1] = 0.55  # 0.05 above the upper limit
    hold = ConfigurationTask(oink, q, np.ones(n))
    limit = PositionLimit(
        oink,
        joint_lower_limits=np.full(n, -0.5),
        joint_upper_limits=np.full(n, 0.5),
        max_pullback=max_pullback,
    )

    delta_q = np.zeros(n)
    oink.solveIk(q, [hold], [limit], delta_q)

    assert delta_q[0] == pytest.approx(-max_pullback, abs=1e-6)
    assert delta_q[1] == pytest.approx(-min(max_pullback, 0.05), abs=1e-6)
    np.testing.assert_allclose(delta_q[2:], 0.0, atol=1e-6)


def test_solve_iterative_ik_respects_custom_limits(
    test_scene: Scene, oink: Oink
) -> None:
    q_center = np.array([0.3, -1.2, 1.4, -1.0, -1.5, 0.2])
    lower = q_center - 0.3
    upper = q_center + 0.3
    q_goal = q_center + 0.1
    goal = CartesianConfiguration(
        "", TIP_FRAME, test_scene.forwardKinematics(q_goal, TIP_FRAME)
    )
    options = IterativeSolveOptions()
    options.max_restarts = 0

    # Starting outside the custom limits, the solver pulls the joint back inside.
    q_start = q_center.copy()
    q_start[0] = upper[0] + 0.2
    limit = PositionLimit(
        oink, joint_lower_limits=lower, joint_upper_limits=upper, max_pullback=0.05
    )
    solution = oink.solveIterativeIk(
        q_start,
        [FrameTask(oink, test_scene, goal)],
        constraints=[limit],
        options=options,
    )
    assert np.all(solution >= lower - 1e-4)
    assert np.all(solution <= upper + 1e-4)
    np.testing.assert_allclose(
        test_scene.forwardKinematics(solution, TIP_FRAME),
        goal.tform,
        atol=1e-3,
    )

    # A goal that is only reachable outside the custom limits is rejected.
    tight_upper = upper.copy()
    tight_upper[0] = q_goal[0] - 0.1
    tight_limit = PositionLimit(
        oink,
        joint_lower_limits=lower,
        joint_upper_limits=tight_upper,
        max_pullback=0.05,
    )
    with pytest.raises(RuntimeError):
        oink.solveIterativeIk(
            q_goal,
            [FrameTask(oink, test_scene, goal)],
            constraints=[tight_limit],
            options=options,
        )
