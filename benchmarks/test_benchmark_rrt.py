import pytest
import sys
import xacro

from roboplan.core import JointConfiguration, Scene
from roboplan.example_models import get_package_share_dir
from roboplan.rrt import RRTOptions, RRT

# We don't build the bindings examples, so we just include the relative
# directory manually.
from pathlib import Path

examples_dir = Path(__file__).parent.parent / "roboplan_examples" / "python"
sys.path.insert(0, str(examples_dir))

from common import get_model_data


def solve(scene: Scene, rrt: RRT, q_indices, seed: int = 1234):
    """
    Runs an RRT test by sampling random, collision-free joint configurations
    then attempting to plan a path between them.

    Returns 1 if planning was successful, 0 otherwise.
    """
    rrt.setRngSeed(seed)

    q_start_full = scene.randomCollisionFreePositions()
    assert q_start_full is not None

    q_goal_full = scene.randomCollisionFreePositions()
    assert q_goal_full is not None

    start = JointConfiguration()
    start.positions = q_start_full[q_indices]

    goal = JointConfiguration()
    goal.positions = q_goal_full[q_indices]

    try:
        path = rrt.plan(start, goal)
    except RuntimeError:
        path = None

    return 0 if path is None else 1


def solve_many(
    scene: Scene, rrt: RRT, q_indices, iterations: int = 10, seed: int = 1234
):
    """
    Runs the specified number of iterations of RRT with a random seed.

    Returns the number of successful solves.
    """
    successes = 0
    for i in range(iterations):
        successes += solve(scene, rrt, q_indices, seed + i)
    return successes


def create_scene(model_name: str) -> Scene:
    model_data = get_model_data()[model_name]
    package_paths = [get_package_share_dir()]

    urdf_xml = xacro.process_file(model_data.urdf_path).toxml()
    srdf_xml = xacro.process_file(model_data.srdf_path).toxml()

    scene = Scene(
        f"{model_name}_benchmark_scene",
        urdf=urdf_xml,
        srdf=srdf_xml,
        package_paths=package_paths,
        yaml_config_path=model_data.yaml_config_path,
    )
    return scene


@pytest.fixture(scope="session", params=["so101", "kinova", "ur5", "franka", "dual"])
def benchmark_setup(request):
    """Scene and RRT configuration aligned with example_rrt.py."""
    model_name = request.param
    model_data = get_model_data()[model_name]
    scene = create_scene(model_name)
    group_info = scene.getJointGroupInfo(model_data.default_joint_group)
    return {
        "model_name": model_name,
        "scene": scene,
        "group_name": model_data.default_joint_group,
        "q_indices": group_info.q_indices,
    }


def test_benchmark_rrt(benchmark, benchmark_setup):
    scene = benchmark_setup["scene"]
    options = RRTOptions()
    options.group_name = benchmark_setup["group_name"]
    options.max_nodes = 100000
    options.max_planning_time = 10.0
    rrt = RRT(scene, options)

    success_rate = benchmark(
        solve_many,
        scene,
        rrt,
        benchmark_setup["q_indices"],
        iterations=10,
    )
    assert success_rate >= 0.95


def test_benchmark_rrt_connect(benchmark, benchmark_setup):
    scene = benchmark_setup["scene"]
    options = RRTOptions()
    options.group_name = benchmark_setup["group_name"]
    options.max_nodes = 100000
    options.rrt_connect = True
    options.max_planning_time = 10.0
    rrt = RRT(scene, options)

    success_rate = benchmark(
        solve_many,
        scene,
        rrt,
        benchmark_setup["q_indices"],
        iterations=10,
    )
    assert success_rate >= 0.95
