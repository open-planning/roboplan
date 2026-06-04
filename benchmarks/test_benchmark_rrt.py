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


# Number of (start, goal) solves timed per benchmark round.
ITERATIONS = 10

# Minimum fraction of solves that must succeed, aggregated across all benchmark
# rounds. This is a sanity floor to catch a broken planner (which would solve ~0%),
# not a tight quality bound: most models solve ~100% of random instances, but the
# dual-arm model with plain RRT has a non-trivial intrinsic timeout rate on hard
# instances (~85% success), so the floor is set comfortably below that to avoid
# flaky failures on small samples.
MIN_SUCCESS_RATE = 0.8


def _make_options(group_name: str, rrt_connect: bool) -> RRTOptions:
    options = RRTOptions()
    options.group_name = group_name
    options.max_nodes = 10000
    options.max_planning_time = 10.0
    options.rrt_connect = rrt_connect
    options.collision_check_use_bisection = True
    options.collision_check_step_size = 0.2
    return options


def _benchmark_planner(benchmark, benchmark_setup, rrt_connect: bool):
    """Times RRT planning and asserts an aggregate success rate.

    To avoid measuring the same handful of (start, goal) instances on every round
    (which makes the result hypersensitive to a single unlucky seed), the seed base
    is rotated on each call so each round samples fresh instances. Successes are
    accumulated across every round and checked against MIN_SUCCESS_RATE.
    """
    scene = benchmark_setup["scene"]
    options = _make_options(benchmark_setup["group_name"], rrt_connect)
    rrt = RRT(scene, options)
    q_indices = benchmark_setup["q_indices"]

    stats = {"seed": 1234, "successes": 0, "total": 0}

    def run():
        seed = stats["seed"]
        stats["seed"] += ITERATIONS
        successes = solve_many(scene, rrt, q_indices, iterations=ITERATIONS, seed=seed)
        stats["successes"] += successes
        stats["total"] += ITERATIONS
        return successes

    benchmark(run)

    success_rate = stats["successes"] / stats["total"]
    assert success_rate >= MIN_SUCCESS_RATE, (
        f"success rate {success_rate:.2f} below floor {MIN_SUCCESS_RATE} "
        f"({stats['successes']}/{stats['total']} solves)"
    )


def test_benchmark_rrt(benchmark, benchmark_setup):
    _benchmark_planner(benchmark, benchmark_setup, rrt_connect=False)


def test_benchmark_rrt_connect(benchmark, benchmark_setup):
    _benchmark_planner(benchmark, benchmark_setup, rrt_connect=True)
