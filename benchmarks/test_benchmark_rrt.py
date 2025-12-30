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

from common import MODELS


def solve(scene: Scene, rrt: RRT, seed: int = 1234):
    """
    Runs an RRT test by sampling random, collision-free joint configurations
    then attempting to plan a path between them.

    Returns 1 if planning was successful, 0 otherwise.
    """
    scene.setRngSeed(seed)

    start = JointConfiguration()
    start.positions = scene.randomCollisionFreePositions()
    assert start.positions is not None

    goal = JointConfiguration()
    goal.positions = scene.randomCollisionFreePositions()
    assert goal.positions is not None

    try:
        path = rrt.plan(start, goal)
    except RuntimeError:
        path = None

    return 0 if path is None else 1


def solve_many(scene: Scene, rrt: RRT, iterations: int = 10, seed: int = 1234):
    """
    Runs the specified number of iterations of RRT with a random seed.

    Returns the number of successful solves.
    """
    successes = 0
    for i in range(iterations):
        successes += solve(scene, rrt, seed + i)
    return successes


def create_scene(model_name: str) -> Scene:
    model_data = MODELS[model_name]
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
def scene(request):
    return create_scene(request.param)


def test_benchmark_rrt(benchmark, scene):
    options = RRTOptions()
    options.max_nodes = 100000
    options.max_planning_time = 10.0
    rrt = RRT(scene, options)

    success_rate = benchmark(solve_many, scene, rrt, iterations=10)
    assert success_rate >= 0.95


def test_benchmark_rrt_connect(benchmark, scene):
    options = RRTOptions()
    options.max_nodes = 100000
    options.rrt_connect = True
    options.max_planning_time = 10.0
    rrt = RRT(scene, options)

    success_rate = benchmark(solve_many, scene, rrt, iterations=10)
    assert success_rate >= 0.95
