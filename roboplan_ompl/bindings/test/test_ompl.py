"""
Unit tests for the OMPL planner wrapper in RoboPlan.
"""

import numpy as np
import pytest

from roboplan.core import (
    Box,
    JointConfiguration,
    Scene,
    computePathLength,
    loadUrdfSceneDescription,
)
from roboplan.example_models import get_package_models_dir, get_package_share_dir
from roboplan.ompl import OMPLPlanner, OMPLPlannerOptions


@pytest.fixture
def test_scene() -> Scene:
    roboplan_models_dir = get_package_models_dir()
    urdf_path = roboplan_models_dir / "ur_robot_model" / "ur5_gripper.urdf"
    srdf_path = roboplan_models_dir / "ur_robot_model" / "ur5_gripper.srdf"
    package_paths = [get_package_share_dir()]

    description = loadUrdfSceneDescription(urdf_path, package_paths)
    scene = Scene("test_scene", description)
    scene.importSrdf(srdf_path.read_text())
    scene.setRngSeed(286)
    # A pillar in the sweep between start and goal, so they are not directly connectable.
    tform = np.eye(4)
    tform[:3, 3] = [0.0, 0.4, 0.4]
    scene.addBoxGeometry(
        "pillar", "universe", Box(0.1, 0.1, 0.8), tform, np.array([1.0, 0.0, 0.0, 1.0])
    )
    return scene


def make_problem(scene: Scene) -> tuple[JointConfiguration, JointConfiguration]:
    start = JointConfiguration()
    start.positions = np.array([0.0, -2.0, 2.0, -1.5, -1.5, 0.0])
    goal = JointConfiguration()
    goal.positions = np.array([3.0, -2.0, 2.0, -1.5, -1.5, 0.0])
    return start, goal


@pytest.mark.parametrize("planner_name", ["RRTConnect", "AORRTC", "BITstar"])
def test_plan(test_scene: Scene, planner_name: str) -> None:
    start, goal = make_problem(test_scene)

    options = OMPLPlannerOptions(
        group_name="arm",
        planner_name=planner_name,
        max_planning_time=1.0,
        simplify_path=True,
    )
    planner = OMPLPlanner(test_scene, options)

    path = planner.plan(start, goal)
    assert len(path.positions) > 2
    assert np.allclose(path.positions[0], start.positions)
    assert np.allclose(path.positions[-1], goal.positions)
    assert computePathLength(test_scene, "arm", path) > 0.0

    stats = planner.getLastStats()
    assert stats.planning_time > 0.0
    assert stats.num_state_checks > 0
    assert stats.num_motion_checks > 0
    # AORRTC clears its trees between iterations, so it reports no graph.
    assert (len(planner.getEdges()) == 0) == (planner_name == "AORRTC")
    if planner_name == "RRTConnect":
        nodes = planner.getNodes()
        assert {tag for _, tag in nodes} == {1, 2}
        assert all(i < len(nodes) and j < len(nodes) for i, j in planner.getEdges())


def test_planner_names_and_params(test_scene: Scene) -> None:
    assert {"RRTConnect", "AORRTC"} <= set(OMPLPlanner.plannerNames())

    with pytest.raises(RuntimeError):
        OMPLPlanner(test_scene, OMPLPlannerOptions(group_name="not_a_group"))

    start, goal = make_problem(test_scene)
    planner = OMPLPlanner(
        test_scene,
        OMPLPlannerOptions(group_name="arm", planner_params={"not_a_param": "1"}),
    )
    with pytest.raises(RuntimeError, match="Unknown parameter"):
        planner.plan(start, goal)
