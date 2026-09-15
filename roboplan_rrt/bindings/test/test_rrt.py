"""
Unit tests for RRT planners in RoboPlan.
"""

from pathlib import Path

import numpy as np
import pinocchio as pin
import pytest
from roboplan.core import (
    JointConfiguration,
    Scene,
    UrdfSceneDescription,
    loadUrdfSceneDescription,
    computePathLength,
)
from roboplan.example_models import get_package_models_dir, get_package_share_dir
from roboplan.rrt import (
    RRT,
    ConstraintProjector,
    ConstraintProjectorOptions,
    PoseConstraint,
    RRTOptions,
)


@pytest.fixture
def test_scene() -> Scene:
    roboplan_models_dir = get_package_models_dir()
    urdf_path = roboplan_models_dir / "ur_robot_model" / "ur5_gripper.urdf"
    srdf_path = roboplan_models_dir / "ur_robot_model" / "ur5_gripper.srdf"
    package_paths = [get_package_share_dir()]

    return Scene(
        "test_scene", loadUrdfSceneDescription(urdf_path, srdf_path), package_paths
    )


def test_plan(test_scene: Scene) -> None:
    # Ensure determinism in the test.
    test_scene.setRngSeed(286)

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


def test_plan_rrt_star(test_scene: Scene) -> None:
    # Plan the same problem with and without RRT*. RRT* keeps rewiring and optimizing,
    # so its path must be equal or shorter than plain RRT.

    # Ensure determinism in the test.
    test_scene.setRngSeed(286)

    start = JointConfiguration()
    start.positions = test_scene.randomCollisionFreePositions()
    assert start.positions is not None

    goal = JointConfiguration()
    goal.positions = test_scene.randomCollisionFreePositions()
    assert goal.positions is not None

    def plan_with(rrt_star: bool):
        options = RRTOptions()
        options.group_name = "arm"
        options.max_connection_distance = 1.0
        options.collision_check_step_size = 0.05
        options.rrt_star = rrt_star
        options.rewire_distance = 2.0
        # Disable fast_return so RRT* optimizes, and bound the search by a fixed node budget (not a
        # wall-clock time budget). A node budget makes both runs do exactly the same amount of work
        # for the same seed, so the comparison is deterministic and independent of machine load.
        options.fast_return = False
        options.max_nodes = 500

        rrt = RRT(test_scene, options)
        rrt.setRngSeed(1234)
        return rrt.plan(start, goal)

    star_path = plan_with(rrt_star=True)
    rrt_path = plan_with(rrt_star=False)
    assert star_path is not None
    assert rrt_path is not None
    print(star_path)

    # RRT* must never produce a longer path than plain RRT.
    star_length = computePathLength(test_scene, "arm", star_path)
    rrt_length = computePathLength(test_scene, "arm", rrt_path)
    assert star_length <= rrt_length


@pytest.fixture
def upright_constraint(test_scene: Scene) -> PoseConstraint:
    """A constraint holding the UR5 tool near vertical, with position and spin left free."""
    # The region frame is a half turn about x, so a zero displacement points the tool straight
    # down. Bounding roll and pitch while leaving yaw and all of position unbounded is the usual
    # way to say "hold this axis steady, go where you like".
    tform = np.eye(4)
    tform[:3, :3] = pin.rpy.rpyToMatrix(np.pi, 0.0, 0.0)
    tilt = 0.1
    inf = np.inf
    return PoseConstraint(
        test_scene,
        "arm",
        "tool0",
        lower_bounds=np.array([-inf, -inf, -inf, -tilt, -tilt, -np.pi]),
        upper_bounds=np.array([inf, inf, inf, tilt, tilt, np.pi]),
        tform=tform,
    )


def test_pose_constraint_displacement(
    test_scene: Scene, upright_constraint: PoseConstraint
) -> None:
    test_scene.setRngSeed(13)
    q = test_scene.randomCollisionFreePositions()

    # The displacement is the tool pose in the region frame, so recomposing it against the region
    # transform must reproduce plain forward kinematics.
    displacement = upright_constraint.computeDisplacement(q)
    recomposed = np.eye(4)
    recomposed[:3, :3] = pin.rpy.rpyToMatrix(displacement[3:])
    recomposed[:3, 3] = displacement[:3]
    expected = test_scene.forwardKinematics(q, "tool0")
    assert np.allclose(upright_constraint.tform @ recomposed, expected, atol=1e-9)


def test_projection_leaves_free_coordinates_free(
    test_scene: Scene, upright_constraint: PoseConstraint
) -> None:
    test_scene.setRngSeed(13)
    projector = ConstraintProjector(test_scene, "arm", [upright_constraint])

    q = test_scene.randomCollisionFreePositions()
    q_projected = projector.project(q)
    assert q_projected is not None
    assert projector.satisfies(q_projected)

    # Position is unbounded, so its residual is exactly zero: an unbounded coordinate contributes
    # no rows to the projection, which is what lets the tool translate while its tilt is corrected.
    error = upright_constraint.computeError(q_projected)
    assert np.linalg.norm(error[:3]) == 0.0

    # The tool moved to fix its orientation, rather than being pinned in place.
    before = test_scene.forwardKinematics(q, "tool0")[:3, 3]
    after = test_scene.forwardKinematics(q_projected, "tool0")[:3, 3]
    assert np.linalg.norm(after - before) > 0.0


def test_plan_with_constraint_stays_on_constraint(
    test_scene: Scene, upright_constraint: PoseConstraint
) -> None:
    # Seed 13's first two collision-free draws both project onto this constraint and are
    # connectable under it, which most random pairs are not: the constraint splits the
    # configuration space into components a constrained path cannot cross.
    test_scene.setRngSeed(13)
    projector = ConstraintProjector(test_scene, "arm", [upright_constraint])

    q_start = projector.project(test_scene.randomCollisionFreePositions())
    q_goal = projector.project(test_scene.randomCollisionFreePositions())
    assert q_start is not None and q_goal is not None

    q_indices = test_scene.getJointGroupInfo("arm").q_indices
    start = JointConfiguration()
    start.positions = q_start[q_indices]
    goal = JointConfiguration()
    goal.positions = q_goal[q_indices]

    options = RRTOptions(
        group_name="arm",
        rrt_connect=True,
        max_connection_distance=0.5,
        max_nodes=20000,
        max_planning_time=20.0,
        constraint_projection=ConstraintProjectorOptions(path_step_size=0.1),
    )
    rrt = RRT(test_scene, options)
    rrt.setRngSeed(1234)

    path = rrt.plan(start, goal, [upright_constraint])
    assert len(path.positions) >= 2

    # Every waypoint and every edge between them must satisfy the constraint at the resolution the
    # planner checked. That is the guarantee separating constrained planning from planning that
    # merely starts and ends on the constraint.
    for idx in range(len(path.positions) - 1):
        q_a = test_scene.toFullJointPositions("arm", path.positions[idx])
        q_b = test_scene.toFullJointPositions("arm", path.positions[idx + 1])
        assert projector.satisfies(q_a)
        assert projector.satisfiesAlongPath(q_a, q_b, options.collision_check_step_size)
    assert projector.satisfies(
        test_scene.toFullJointPositions("arm", path.positions[-1])
    )


def test_plan_rejects_endpoints_off_the_constraint(
    test_scene: Scene, upright_constraint: PoseConstraint
) -> None:
    test_scene.setRngSeed(13)
    projector = ConstraintProjector(test_scene, "arm", [upright_constraint])

    q_on = projector.project(test_scene.randomCollisionFreePositions())
    assert q_on is not None

    # Tipping the tool most of the way over puts it far outside a 0.1 rad tilt bound.
    q_off = q_on.copy()
    q_off[4] += 1.5
    assert not projector.satisfies(q_off)

    q_indices = test_scene.getJointGroupInfo("arm").q_indices
    on, off = JointConfiguration(), JointConfiguration()
    on.positions = q_on[q_indices]
    off.positions = q_off[q_indices]

    rrt = RRT(test_scene, RRTOptions(group_name="arm"))
    rrt.setRngSeed(1234)
    with pytest.raises(RuntimeError, match="Start configuration does not satisfy"):
        rrt.plan(off, on, [upright_constraint])
    with pytest.raises(RuntimeError, match="Goal configuration does not satisfy"):
        rrt.plan(on, off, [upright_constraint])
