import sys
from pathlib import Path

import numpy as np
import pytest

from roboplan.cartesian_planning import (
    CartesianPathPlanner,
    CartesianPlannerOptions,
    CartesianSpeedMode,
)
from roboplan.core import CartesianPath, JointConfiguration, Scene

# The examples are not an installed package, so add their directory to the path to import `common`.
examples_dir = Path(__file__).parent.parent / "roboplan_examples" / "python"
sys.path.insert(0, str(examples_dir))

from common import build_scene, get_home_configuration, get_model_data

# Trace out a small square path with the tip
PATH_SIZE = 0.05
STEP = np.array([1.0, 0.0, 0.0]) * PATH_SIZE
RISE = np.array([0.0, 1.0, 0.0]) * PATH_SIZE


def make_square_path(
    scene: Scene, base_link: str, tip_frames: list[str], q_full: np.ndarray
) -> CartesianPath:
    """Returns a 4-corner CartesianPath

    The path will just be in the end-effector local frame to keep things simple.
    """
    world_T_base = scene.forwardKinematics(q_full, base_link)
    tforms = []
    for tip_frame in tip_frames:
        world_T_ee = scene.forwardKinematics(q_full, tip_frame)
        base_T_ee = np.linalg.inv(world_T_base) @ world_T_ee
        waypoints = []
        for offset in (np.zeros(3), STEP, STEP + RISE, RISE, np.zeros(3)):
            tform = base_T_ee.copy()
            tform[:3, 3] += base_T_ee[:3, :3] @ offset
            waypoints.append(tform)
        tforms.append(waypoints)
    return CartesianPath([base_link] * len(tip_frames), tip_frames, tforms)


# Only benchmarking a subset of available models, but these give some
# variability and succeed with the paths above.
@pytest.fixture(scope="session", params=["ur5", "franka", "stretch", "tiago_pro"])
def model_name(request):
    return request.param


@pytest.fixture(scope="session")
def cartesian_benchmark_setup(model_name):
    model_data = get_model_data()[model_name]
    scene = build_scene(model_name)
    scene.setRngSeed(1234)

    q0_full = get_home_configuration(scene, model_data)
    scene.setJointPositions(q0_full)

    path = make_square_path(scene, model_data.base_link, model_data.ee_names, q0_full)

    q_start = JointConfiguration()
    q_start.positions = q0_full

    return {"scene": scene, "path": path, "q_start": q_start, "model_data": model_data}


def plan_once(
    planner: CartesianPathPlanner, path: CartesianPath, q_start: JointConfiguration
) -> None:
    planner.plan(path, q_start)


@pytest.mark.parametrize(
    "speed_mode", [CartesianSpeedMode.Bounded, CartesianSpeedMode.TimeOptimal]
)
def test_benchmark_cartesian_planning(benchmark, cartesian_benchmark_setup, speed_mode):
    s = cartesian_benchmark_setup
    options = CartesianPlannerOptions(
        group_name=s["model_data"].default_joint_group,
        speed_mode=speed_mode,
    )
    planner = CartesianPathPlanner(s["scene"], options)
    benchmark(plan_once, planner, s["path"], s["q_start"])
