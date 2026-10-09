import sys
from pathlib import Path

import numpy as np
import pytest

from roboplan.core import CartesianConfiguration, Scene
from roboplan.optimal_ik import (
    FrameTask,
    Oink,
    OinkSettings,
    PositionLimit,
    VelocityLimit,
)

# The examples are not an installed package, so add their directory to the path to import `common`.
examples_dir = Path(__file__).parent.parent / "roboplan_examples" / "python"
sys.path.insert(0, str(examples_dir))

from common import build_scene, get_model_data

# Streaming control-loop rate
CONTROL_DT = 1.0 / 500.0


def solve_many(oink: Oink, scene: Scene, tasks, constraints, qs) -> None:
    """Solves one IK step at each configuration in `qs`, as a streaming controller would."""
    delta_q = np.zeros(oink.num_variables)
    for q_full in qs:
        scene.setJointPositions(q_full)
        oink.solveIk(scene, tasks, constraints, [], delta_q)


@pytest.fixture(scope="session", params=["so101", "kinova", "ur5", "franka", "dual"])
def model_name(request):
    return request.param


@pytest.fixture(scope="session")
def oink_benchmark_setup(model_name):
    model_data = get_model_data()[model_name]
    scene = build_scene(model_name)
    group_name = model_data.default_joint_group
    q_indices = scene.getJointGroupInfo(group_name).q_indices

    scene.setRngSeed(1234)
    q0_full = scene.randomCollisionFreePositions()
    assert q0_full is not None
    q0_group = q0_full[q_indices]
    scene.setJointPositions(q0_full)

    settings = OinkSettings()
    settings.primal_infeasibility_solving = True
    # Replaying a fixed qs sequence each round would carry state across rounds if warm-started.
    # Must be disabled so every call is an independent, reproducible measurement.
    settings.warm_start = False
    oink = Oink(scene, group_name, settings)

    # Regulate the first end-effector to its starting pose.
    ee_name = model_data.ee_names[0]
    q_current = scene.getCurrentJointPositions()
    world_T_base = scene.forwardKinematics(q_current, model_data.base_link)
    world_T_ee = scene.forwardKinematics(q_current, ee_name)
    target = CartesianConfiguration()
    target.base_frame = model_data.base_link
    target.tip_frame = ee_name
    target.tform = np.linalg.inv(world_T_base) @ world_T_ee
    tasks = [FrameTask(oink, scene, target)]

    _, v_upper = scene.getVelocityLimitVectors(group_name)
    constraints = [
        PositionLimit(oink, gain=1.0),
        VelocityLimit(oink, CONTROL_DT, np.abs(v_upper)),
    ]

    # Small joint-space steps around q0, like a streaming controller would take.
    rng = np.random.default_rng(1234)
    lower, upper = scene.getPositionLimitVectors(group_name)
    qs = []
    q_group = q0_group.copy()
    for _ in range(20):
        q_group = np.clip(
            q_group + rng.uniform(-0.02, 0.02, size=len(q_indices)), lower, upper
        )
        qs.append(scene.toFullJointPositions(group_name, q_group))

    return {
        "oink": oink,
        "tasks": tasks,
        "constraints": constraints,
        "scene": scene,
        "qs": qs,
    }


def test_benchmark_oink_solve(benchmark, oink_benchmark_setup):
    s = oink_benchmark_setup
    benchmark(solve_many, s["oink"], s["scene"], s["tasks"], s["constraints"], s["qs"])
